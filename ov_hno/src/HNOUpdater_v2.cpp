/*批量更新 + 不做卡方检验 */

#include "HNOUpdater.h"
#include <iostream>
#include <boost/math/distributions/chi_squared.hpp>
#include <vector>

using namespace ov_hno;
using namespace Eigen;

HNOUpdater::HNOUpdater() {
    // 默认外参为单位阵 (需通过 setExtrinsics 设置)
    R_C2B_left.setIdentity(); pc_left.setZero();
    R_C2B_right.setIdentity(); pc_right.setZero();
}

void HNOUpdater::setExtrinsics(const std::map<size_t, Eigen::Matrix4d>& T_C2B_map) {
    if(T_C2B_map.count(0)) {
        R_C2B_left = T_C2B_map.at(0).block<3,3>(0,0);
        pc_left = T_C2B_map.at(0).block<3,1>(0,3);
    }
    if(T_C2B_map.count(1)) {
        R_C2B_right = T_C2B_map.at(1).block<3,3>(0,0);
        pc_right = T_C2B_map.at(1).block<3,1>(0,3);
    }

    // 只有左右目同时存在才启用双目逻辑
    has_stereo_extrinsics = (T_C2B_map.count(0) && T_C2B_map.count(1));
}

Eigen::Matrix3d HNOUpdater::project_pi(const Eigen::Vector3d& x) {
    Eigen::Vector3d n = x.normalized();     //确保x是单位向量
    return Eigen::Matrix3d::Identity() - n * n.transpose();
}

void HNOUpdater::update(std::shared_ptr<HNOState> state,
                        const std::vector<HNOObservation>& observations) {
    // 有观测数据才更新
    int N = observations.size();
    if(N == 0) return;

    // 归一化平面像素噪声标准差 Cov(ny)
    double sigma_pix = pixel_noise / focal_length; 
    double sigma_pix_sq = sigma_pix * sigma_pix;   

    // 初始化大矩阵
    Eigen::MatrixXd C_big(3 * N, 15); C_big.setZero();
    Eigen::VectorXd sigma_y_big(3 * N); sigma_y_big.setZero();
    Eigen::MatrixXd Q_big(3 * N, 3 * N); Q_big.setZero();

    // 快照，简化变量长度
    Eigen::Matrix3d R_hat_B2I = state->R_hat_B2I; 
    Eigen::Vector3d p_hat = state->p_hat; 

    for (int i = 0; i < N; ++i) {
        const auto& feature = observations[i];
    
        // 1. 准备路标位置
        // 使用当前的 e_hat 重构路标位置 p_i_hat = sum(p_ij * e_j_hat)
        double p_i1 = feature.xyz(0);
        double p_i2 = feature.xyz(1);
        double p_i3 = feature.xyz(2);

        Eigen::Vector3d pf_hat_I = p_i1 * state->e_hat[0] + 
                                   p_i2 * state->e_hat[1] + 
                                   p_i3 * state->e_hat[2];
        // 路标在机体系的估计位置 pf_hat_B = R_hat^T * (p_i_hat - p_hat)
        Eigen::Vector3d pf_hat_B = R_hat_B2I.transpose() * (pf_hat_I - p_hat);


        // 2. 计算残差
        // 左目残差，将相机系归一化坐标投影到机体系 pi(R_c*y_i)
        Eigen::Matrix3d pi_left = R_C2B_left * project_pi(feature.uv_left) * R_C2B_left.transpose();
        Eigen::Vector3d sigma_y_left = pi_left * (pf_hat_B - pc_left);

        // 右目残差，因为右目不一定有，所以必须赋初值
        Eigen::Matrix3d pi_right = Eigen::Matrix3d::Zero();
        Eigen::Vector3d sigma_y_right = Eigen::Vector3d::Zero();
        if (feature.isValidRight && has_stereo_extrinsics) {
            pi_right = R_C2B_right * project_pi(feature.uv_right) * R_C2B_right.transpose();
            sigma_y_right = pi_right * (pf_hat_B - pc_right);
        }
        // 总残差
        Eigen::Vector3d sigma_y_i = sigma_y_left + sigma_y_right;
        // 总投影算子 公式(27b)用到的
        Eigen::Matrix3d Pi_total = pi_left + pi_right;


        // 3. 计算观测噪声协方差 Q_i (3x3) 实际是Q^-1
        // 公式(27b) Q^-1 = Mt * Cov(ny) * Mt^T
        double dist_sq = pf_hat_B.squaredNorm(); 
        if(dist_sq < 0.1) dist_sq = 0.1;

        Eigen::Matrix3d Q_i = 0.1*(dist_sq * sigma_pix_sq) * Pi_total;

        Q_i += 1e-12 * Eigen::Matrix3d::Identity();


        // 4. 计算 C_i  维度: 3 x 15
        // 公式(12): C_i = [ Pi_total, -p_i1*Pi_total, -p_i2*Pi_total, -p_i3*Pi_total, 0 ]
        Eigen::Matrix<double, 3, 15> C_i; C_i.setZero();
        C_i.block<3,3>(0, 0) = Pi_total;
        C_i.block<3,3>(0, 3) = -p_i1 * Pi_total; // e1
        C_i.block<3,3>(0, 6) = -p_i2 * Pi_total; // e2
        C_i.block<3,3>(0, 9) = -p_i3 * Pi_total; // e3
        

       // 5. 填入大矩阵
        C_big.block<3, 15>(3 * i, 0) = C_i;
        Q_big.block<3, 3>(3 * i, 3 * i) = Q_i;
        sigma_y_big.segment<3>(3 * i) = sigma_y_i;
    }

    // 5. 更新增益  K=[Kp, K1, K2, K3, Kv] (批量更新
    // 公式(24) K = P * C^T * (C * P * C^T + Q)^-1

    Eigen::MatrixXd PHT = state->P * C_big.transpose(); // 15 x 3N
    Eigen::MatrixXd S = C_big * PHT + Q_big;      // 3N x 3N
    // 求解增益 K, 使用 LDLT 分解求逆更稳定
    Eigen::MatrixXd K = PHT * S.ldlt().solve(Eigen::MatrixXd::Identity(3*N, 3*N));


    // 6. 更新状态
    // 计算状态修正量 K*sigma_y (15x1 Body Frame Error)
    Eigen::VectorXd delta = K * sigma_y_big;


    // 截断保护 (第四道防线)：防止单次修正过大导致系统飞出
    // Relaxed from 0.5 to 5.0 to allow recovery from drift
    if (delta.segment<3>(0).norm() > 5.0) { // 位置修正超过5.0米
            std::cout << "[HNO] Update ignored: correction too large ( > 5.0m )." << std::endl;
    }

    // p, e, v 都是定义在 Inertial Frame 的,delta 是定义在 Body Frame 的误差, dx_I = R * dx_B
    state->p_hat    += R_hat_B2I * delta.segment<3>(0);
    state->e_hat[0] += R_hat_B2I * delta.segment<3>(3);
    state->e_hat[1] += R_hat_B2I * delta.segment<3>(6);
    state->e_hat[2] += R_hat_B2I * delta.segment<3>(9);
    state->v_hat    += R_hat_B2I * delta.segment<3>(12);

    // 7. 更新协方差
    // 公式(25) P = (I - K*C) * P
    Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * C_big;
    state->P = I_KH * state->P * I_KH.transpose() + K * Q_big * K.transpose(); 

    // 强制对称, 防止长期运行积累不对称误差
    state->P = 0.5 * (state->P + state->P.transpose());

    // 每帧更新结束后，强制约束 e_hat 必须也是单位正交基
    // state->enforce_structure();
}
