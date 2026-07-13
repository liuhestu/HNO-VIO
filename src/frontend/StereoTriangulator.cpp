#include "hno_vio/frontend/StereoTriangulator.h"

#include <utility>

namespace hno_vio::frontend {

StereoTriangulator::StereoTriangulator(std::vector<Eigen::Matrix4d> camera_to_body,
                                       const Constraints& constraints)
    : camera_to_body_(std::move(camera_to_body)), constraints_(constraints) {}

bool StereoTriangulator::triangulate(const Eigen::Vector3d& uv_left,
                                     const Eigen::Vector3d& uv_right,
                                     Eigen::Vector3d& p_left_camera,
                                     double* right_reprojection_error) const {
    if (camera_to_body_.size() < 2) {
        return false;
    }
    const Eigen::Matrix4d T_right_left =
        camera_to_body_[1].inverse() * camera_to_body_[0];
    const Eigen::Matrix3d R = T_right_left.block<3, 3>(0, 0);
    const Eigen::Vector3d t = T_right_left.block<3, 1>(0, 3);

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = R * uv_left;
    A.col(1) = -uv_right;
    const Eigen::Vector2d depths =
        (A.transpose() * A).ldlt().solve(A.transpose() * -t);
    const double left_depth = depths(0);
    if (left_depth < constraints_.min_depth || left_depth > constraints_.max_depth) {
        return false;
    }

    p_left_camera = uv_left * left_depth;
    const Eigen::Vector3d p_right = R * p_left_camera + t;
    if (p_right.z() <= 0.0) {
        return false;
    }
    const Eigen::Vector2d projected = (p_right / p_right.z()).head<2>();
    const double error = (projected - uv_right.head<2>()).norm();
    if (right_reprojection_error) {
        *right_reprojection_error = error;
    }
    return error <= constraints_.right_reprojection_threshold;
}

} // namespace hno_vio::frontend
