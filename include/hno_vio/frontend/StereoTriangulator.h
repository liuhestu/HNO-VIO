#ifndef HNO_VIO_FRONTEND_STEREO_TRIANGULATOR_H
#define HNO_VIO_FRONTEND_STEREO_TRIANGULATOR_H

#include <vector>

#include <Eigen/Dense>

namespace hno_vio::frontend {

class StereoTriangulator {
public:
    struct Constraints {
        double min_depth = 0.5;
        double max_depth = 5.0;
        double right_reprojection_threshold = 0.015;
    };

    StereoTriangulator(std::vector<Eigen::Matrix4d> camera_to_body,
                       const Constraints& constraints);

    bool triangulate(const Eigen::Vector3d& uv_left,
                     const Eigen::Vector3d& uv_right,
                     Eigen::Vector3d& p_left_camera,
                     double* right_reprojection_error = nullptr) const;

private:
    std::vector<Eigen::Matrix4d> camera_to_body_;
    Constraints constraints_;
};

} // namespace hno_vio::frontend

#endif
