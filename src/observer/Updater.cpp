/*序贯更新 + 卡方检验*/

#include "hno_vio/observer/Updater.h"
#include <algorithm>
#include <vector>

using namespace hno_vio;
using namespace Eigen;

observer::Updater::Updater() {
    // 默认外参为单位阵 (需通过 setExtrinsics 设置)
    R_C2B_left.setIdentity(); pc_left.setZero();
    R_C2B_right.setIdentity(); pc_right.setZero();
}

void observer::Updater::setOptions(const Options& options) {
    options_ = options;
}

void observer::Updater::setExtrinsics(const std::map<size_t, Eigen::Matrix4d>& T_C2B_map) {
    if(T_C2B_map.count(0)) {
        R_C2B_left = T_C2B_map.at(0).block<3,3>(0,0);
        pc_left = T_C2B_map.at(0).block<3,1>(0,3);
    }
    if(T_C2B_map.count(1)) {
        R_C2B_right = T_C2B_map.at(1).block<3,3>(0,0);
        pc_right = T_C2B_map.at(1).block<3,1>(0,3);
    }
    has_stereo_extrinsics = (T_C2B_map.count(0) && T_C2B_map.count(1));
}

Eigen::Matrix3d observer::Updater::project_pi(const Eigen::Vector3d& x) {
    if(x.norm() < 1e-5) return Eigen::Matrix3d::Identity(); // 增加数值保护
    Eigen::Vector3d n = x.normalized();     //确保x是单位向量
    return Eigen::Matrix3d::Identity() - n * n.transpose();
}

bool observer::Updater::update(std::shared_ptr<State> state,
                               const std::vector<VisualObservation>& observations,
                               UpdaterDiagnostics* diagnostics) {
    const Eigen::Matrix3d R_before = state->R_hat_B2I;
    const Eigen::Vector3d p_before = state->p_hat;
    const Eigen::Vector3d v_before = state->v_hat;
    const Eigen::Matrix3d E_before = state->eMatrix();
    int N = observations.size();
    if (diagnostics) {
        *diagnostics = UpdaterDiagnostics{};
        diagnostics->total_observations = N;
        diagnostics->low_observation_streak = low_observation_streak_;
        diagnostics->e_before = E_before;
        diagnostics->e_raw = E_before;
        diagnostics->e_projected = E_before;
    }
    const auto finalize_state = [&]() {
        const Eigen::Matrix3d E_raw = state->eMatrix();
        const double visual_delta_r = std::abs(Eigen::AngleAxisd(
            R_before.transpose() * state->R_hat_B2I).angle());
        const double visual_delta_p = (state->p_hat - p_before).norm();
        const double visual_delta_v = (state->v_hat - v_before).norm();
        const double visual_delta_e = (E_raw - E_before).norm();
        if (options_.enforce_structure_after_update) {
            state->enforce_structure();
        }
        const Eigen::Matrix3d E_projected = state->eMatrix();
        if (diagnostics) {
            diagnostics->visual_delta_r = visual_delta_r;
            diagnostics->visual_delta_p = visual_delta_p;
            diagnostics->visual_delta_v = visual_delta_v;
            diagnostics->visual_delta_e = visual_delta_e;
            diagnostics->projection_correction =
                (E_projected - E_raw).norm();
            diagnostics->e_raw = E_raw;
            diagnostics->e_projected = E_projected;
        }
    };
    if(N == 0) {
        finalize_state();
        return false;
    }

    if(N < options_.min_observations) {
        ++low_observation_streak_;
        if (diagnostics) {
            diagnostics->low_observation = true;
            diagnostics->low_observation_streak = low_observation_streak_;
        }
        if(low_observation_streak_ >= options_.low_observation_hold_frames) {
            if (diagnostics) diagnostics->skipped_for_low_observations = true;
            finalize_state();
            return false;
        }
    } else {
        low_observation_streak_ = 0;
        if (diagnostics) diagnostics->low_observation_streak = 0;
    }

    // 归一化平面像素噪声标准差 Cov(ny)
    double sigma_pix = options_.pixel_noise / options_.focal_length;
    double sigma_pix_sq = sigma_pix * sigma_pix;

    // 统计有效更新点数
    int chi2_passed = 0;
    int applied_observations = 0;
    int reject_chi2 = 0, reject_nan = 0, reject_gain = 0;
    int reject_trunc_p = 0, reject_trunc_r = 0;
    double chi2_max = 0.0, chi2_max_rej = 0.0;
    double delta_p_max = 0.0, delta_r_max = 0.0;
    bool large_delta_warning = false;

    // --- 采用序贯更新 (Sequential Update) ---
    for (int i = 0; i < N; ++i) {
        const auto& feature = observations[i];

        // 1. 获取当前最新状态
        Eigen::Matrix3d R_hat_B2I = state->R_hat_B2I;
        Eigen::Vector3d p_hat = state->p_hat;

        // 2. 准备路标 (Inertial Frame)
        // 使用当前的 e_hat 重构路标位置 p_i_hat = sum(p_ij * e_j_hat)
        double p_i1 = feature.landmark(0);
        double p_i2 = feature.landmark(1);
        double p_i3 = feature.landmark(2);

        Eigen::Vector3d pf_hat_I = p_i1 * state->e_hat[0] +
                                   p_i2 * state->e_hat[1] +
                                   p_i3 * state->e_hat[2];
        // 路标在机体系的估计位置 pf_hat_B = R_hat^T * (p_i_hat - p_hat)
        Eigen::Vector3d pf_hat_B = R_hat_B2I.transpose() * (pf_hat_I - p_hat);


        // 3. 计算残差
        // 左目残差，将相机系归一化坐标投影到机体系 pi(R_c*y_i)
        Eigen::Matrix3d pi_left = R_C2B_left * project_pi(feature.uv_left) * R_C2B_left.transpose();
        Eigen::Vector3d sigma_y_left = pi_left * (pf_hat_B - pc_left);

        // 右目残差，因为右目不一定有，所以必须赋初值
        Eigen::Matrix3d pi_right = Eigen::Matrix3d::Zero();
        Eigen::Vector3d sigma_y_right = Eigen::Vector3d::Zero();
        if (feature.has_right && has_stereo_extrinsics) {
            pi_right = R_C2B_right * project_pi(feature.uv_right) * R_C2B_right.transpose();
            sigma_y_right = pi_right * (pf_hat_B - pc_right);
        }
        // 总残差
        Eigen::Vector3d sigma_y_i = sigma_y_left + sigma_y_right;

        // 总投影算子 公式(27b)用到的
        Eigen::Matrix3d Pi_total = pi_left + pi_right;


        // 4. 计算观测噪声协方差 Q_i (3x3) 实际是Q^-1
        // 公式(27b) Q^-1 = Mt * Cov(ny) * Mt^T
        double dist_sq = pf_hat_B.squaredNorm();
        if(dist_sq < 0.1 || std::isnan(dist_sq)) dist_sq = 0.1;

        Eigen::Matrix3d Q_i = (dist_sq * sigma_pix_sq) * Pi_total;

        // Ensure Q_i is positive definite enough.
        Q_i += 1e-8 * Eigen::Matrix3d::Identity();


        // 5. 构造雅可比 C_i (3x15)
        // 公式(12): C_i = [ Pi_total, -p_i1*Pi_total, -p_i2*Pi_total, -p_i3*Pi_total, 0 ]
        Eigen::Matrix<double, 3, 15> C_i; C_i.setZero();
        C_i.block<3,3>(0, 0) = Pi_total;          // Pos
        C_i.block<3,3>(0, 3) = -p_i1 * Pi_total; // e1
        C_i.block<3,3>(0, 6) = -p_i2 * Pi_total; // e2
        C_i.block<3,3>(0, 9) = -p_i3 * Pi_total; // e3


        // 6. 卡方检验并更新增益 K=[Kp, K1, K2, K3, Kv]
        // S_i = C_i * P * C_i^T + Q_i
        Eigen::Matrix<double, 15, 3> PHT = state->P * C_i.transpose();
        Eigen::Matrix3d S_i = C_i * PHT + Q_i;


        // 检查 NaN (第一道防线)
        if(!S_i.allFinite()) { reject_nan++; continue; }

        // S 求逆 (3x3 矩阵，LDLT 极快且稳定)
        Eigen::LLT<Eigen::Matrix3d> llt(S_i);
        if (llt.info() == Eigen::NumericalIssue) { reject_nan++; continue; } // 矩阵奇异，跳过

        // 计算马氏距离 (Chi-Square)
           double chi2 = sigma_y_i.transpose() * llt.solve(sigma_y_i);
           if(chi2 > chi2_max) chi2_max = chi2;

           if (std::isnan(chi2) || std::isinf(chi2) || chi2 > options_.chi2_gate) {
               reject_chi2++;
               if(chi2 > chi2_max_rej) chi2_max_rej = chi2;
               continue;
           }

        // --- 通过检验，执行更新 ---
        chi2_passed++;

        // 公式(24) K = P * C^T * (C * P * C^T + Q)^-1
        // K = P H^T S^-1
        Eigen::Matrix<double, 15, 3> K = PHT * llt.solve(Eigen::Matrix3d::Identity());

        // 检查 K (第三道防线)
        if(!K.allFinite()) { reject_gain++; continue; }


        // 7. 更新状态
        // 计算状态修正量 K*sigma_y (15x1 Body Frame Error)
        Eigen::VectorXd delta = K * sigma_y_i;
        if (!delta.allFinite()) {
            reject_nan++;
            continue;
        }

        // [CRITICAL FIX: Third Defense Line - Update Truncation]
        // Truncate explosive updates.
        // During stable flight (20Hz), corrections > 0.15m are likely errors.
           double delta_p = delta.segment<3>(0).norm();
           double delta_r = delta.segment<3>(3).norm();
           if(delta_p > delta_p_max) delta_p_max = delta_p;
           if(delta_r > delta_r_max) delta_r_max = delta_r;

           double effective_max_delta_p = options_.max_delta_p;
           double effective_max_delta_r = options_.max_delta_r;
           if(N < 2 * options_.min_observations) {
               effective_max_delta_p *= 0.5;
               effective_max_delta_r *= 0.5;
           }

           if(delta_p > options_.warn_delta_ratio * effective_max_delta_p ||
              delta_r > options_.warn_delta_ratio * effective_max_delta_r) {
               large_delta_warning = true;
           }

           if (delta_p > effective_max_delta_p) {
               reject_trunc_p++;
               continue;
           }
           if (delta_r > effective_max_delta_r) {
               reject_trunc_r++;
               continue;
           }

        // p, e, v 都是定义在 Inertial Frame 的,delta 是定义在 Body Frame 的误差, dx_I = R * dx_B
        state->p_hat    += R_hat_B2I * delta.segment<3>(0);
        state->e_hat[0] += R_hat_B2I * delta.segment<3>(3);
        state->e_hat[1] += R_hat_B2I * delta.segment<3>(6);
        state->e_hat[2] += R_hat_B2I * delta.segment<3>(9);
        state->v_hat    += R_hat_B2I * delta.segment<3>(12);

        // 8. 更新协方差
        // 公式(25) P = (I - K*C) * P * (I-KC)' + KQK'
        Eigen::Matrix<double, 15, 15> I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * C_i;
        state->P = I_KH * state->P * I_KH.transpose() + K * Q_i * K.transpose();

        // 强制对称, 防止长期运行积累不对称误差
        state->P = 0.5 * (state->P + state->P.transpose());

        // 协方差防爆 (第五道防线)
        if (state->P.diagonal().minCoeff() < 0) {
             // 极罕见情况：重置 P
             state->P.setIdentity(); state->P *= 1e-4;
        }
        applied_observations++;
    }

    // 防止P过自信 (Covariance Limiting)，保持对新观测的敏感度
    // 1e-9 对应标准差 0.03mm 或 0.0017度，是一个安全的数值下限
    for(int i=0; i<15; i++) {
        if(state->P(i,i) < 1e-9) {
            state->P(i,i) = 1e-9;
        }
    }

    finalize_state();

    if (diagnostics) {
        diagnostics->chi2_passed_observations = chi2_passed;
        diagnostics->chi2_rejected_observations = reject_chi2;
        diagnostics->applied_observations = applied_observations;
        diagnostics->numerical_rejected_observations = reject_nan;
        diagnostics->kalman_gain_rejected_observations = reject_gain;
        diagnostics->delta_rejected_observations = reject_trunc_p + reject_trunc_r;
        diagnostics->max_chi2 = chi2_max;
        diagnostics->max_rejected_chi2 = chi2_max_rej;
        diagnostics->max_position_delta = delta_p_max;
        diagnostics->max_rotation_delta = delta_r_max;
        diagnostics->large_delta_warning = large_delta_warning;
        diagnostics->update_applied = applied_observations > 0;
    }
    return applied_observations > 0;
}
