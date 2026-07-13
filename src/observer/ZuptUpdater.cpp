#include "hno_vio/observer/ZuptUpdater.h"

#include <algorithm>

namespace hno_vio::observer {

bool ZuptUpdater::update(std::shared_ptr<State> state) const {
    Eigen::Matrix<double, 3, 15> H;
    H.setZero();
    H.block<3, 3>(0, 12).setIdentity();

    const Eigen::Matrix3d R_B2I = state->R_hat_B2I;
    const Eigen::Vector3d residual = -R_B2I.transpose() * state->v_hat;
    const double sigma_v = std::max(1e-4, velocity_noise_);
    const Eigen::Matrix3d measurement_cov =
        sigma_v * sigma_v * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 15> P_prior = state->P;
    for (int i = 12; i < 15; ++i) {
        P_prior(i, i) = std::max(P_prior(i, i), sigma_v * sigma_v);
    }
    const Eigen::Matrix<double, 15, 3> PHT = P_prior * H.transpose();
    const Eigen::Matrix3d innovation_cov = H * PHT + measurement_cov;
    Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_cov);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }

    Eigen::Matrix<double, 15, 3> K =
        PHT * ldlt.solve(Eigen::Matrix3d::Identity());
    K.block<12, 3>(0, 0).setZero();
    if (K.hasNaN()) {
        return false;
    }

    const Eigen::Matrix<double, 15, 1> delta = K * residual;
    if (delta.hasNaN()) {
        return false;
    }

    state->p_hat += R_B2I * delta.segment<3>(0);
    state->e_hat[0] += R_B2I * delta.segment<3>(3);
    state->e_hat[1] += R_B2I * delta.segment<3>(6);
    state->e_hat[2] += R_B2I * delta.segment<3>(9);
    state->v_hat += R_B2I * delta.segment<3>(12);

    const Eigen::Matrix<double, 15, 15> I_KH =
        Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state->P = I_KH * P_prior * I_KH.transpose() +
               K * measurement_cov * K.transpose();
    state->P = 0.5 * (state->P + state->P.transpose());
    for (int i = 0; i < 15; ++i) {
        state->P(i, i) = std::max(state->P(i, i), 1e-9);
    }
    return state->p_hat.allFinite() && state->v_hat.allFinite() && state->P.allFinite();
}

} // namespace hno_vio::observer
