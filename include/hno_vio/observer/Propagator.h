#ifndef HNO_VIO_OBSERVER_PROPAGATOR_H
#define HNO_VIO_OBSERVER_PROPAGATOR_H

#include "hno_vio/Diagnostics.h"
#include "hno_vio/State.h"
#include <memory>
#include <tuple>

namespace hno_vio::observer {

class Propagator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IMU 噪声参数结构体
    struct NoiseParams {
        double noise_acc;    // Accelerometer noise density
        double noise_gyro;    // Gyroscope noise density
    };

    // 构造函数，只在程序启动、创建这个类对象的时候运行一次
    Propagator();

    // 设置噪声参数，预计算 Cov_nx
    void setNoiseParams(const NoiseParams& params);
    void setExperimentOptions(bool fix_e_hat, bool force_sigma_r_zero);

    // 4阶龙格-库塔法积分
    Eigen::Matrix<double, 15, 15> RK4(const Eigen::Matrix<double, 15, 15>& A,
                                      const Eigen::Matrix<double, 15, 15>& P,
                                      const Eigen::Matrix<double, 15, 15>& Vt,
                                      double dt);

    // 成员函数，每收到一个 IMU 数据就要运行一次
    void propagate(std::shared_ptr<State> state,
                   const Eigen::Vector3d& omega_m,
                   const Eigen::Vector3d& acc_m,
                   double dt,
                   PropagationDiagnostics* diagnostics = nullptr);

private:
    Eigen::Matrix3d skew(const Eigen::Vector3d& v);

    double k_R = 20.0;
    Eigen::Vector3d rho; // Weights for the 3 axes
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.81);
    bool experiment_fix_e_hat_ = false;
    bool experiment_force_sigma_r_zero_ = false;

    // 原始IMU噪声协方差
    Eigen::Matrix<double, 6, 6> Cov_nx;
};

} // namespace hno_vio::observer
#endif
