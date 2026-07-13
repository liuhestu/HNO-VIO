#ifndef HNO_VIO_FRONTEND_FEATURE_MANAGER_H
#define HNO_VIO_FRONTEND_FEATURE_MANAGER_H

#include <memory>
#include <vector>
#include <map>
#include <limits>
#include <Eigen/Dense>

#include "track/TrackKLT.h"
#include "cam/CamBase.h"
#include "hno_vio/observer/Updater.h"
#include "hno_vio/Diagnostics.h"
#include "hno_vio/frontend/FeatureHealth.h"
#include "hno_vio/frontend/LandmarkMap.h"
#include "hno_vio/frontend/StereoTriangulator.h"
#include "hno_vio/State.h"
#include <opencv2/core/types.hpp>

namespace hno_vio::frontend {

class FeatureManager {
public:
    struct Options {
        Options()
            : tracker_num_pts(200),
              tracker_fast_threshold(20),
              tracker_grid_x(5),
              tracker_grid_y(5),
              tracker_min_px_dist(15),
              min_stereo_depth(0.5),
              max_stereo_depth(5.0),
              stereo_reproj_thresh(0.015),
              reproj_thresh(0.08),
              reproj_thresh_low(0.10),
              low_feature_pts(80),
              low_feature_db(60),
              mature_thresh(3),
              mature_thresh_low(2),
              map_jump_thresh(0.5),
              active_mature_thresh(3),
              health_min_stable(20),
              health_min_db(20),
              health_hold_frames(3),
              health_start_frame(60) {}

        int tracker_num_pts;
        int tracker_fast_threshold;
        int tracker_grid_x;
        int tracker_grid_y;
        int tracker_min_px_dist;
        double min_stereo_depth;
        double max_stereo_depth;
        double stereo_reproj_thresh;
        double reproj_thresh;
        double reproj_thresh_low;
        int low_feature_pts;
        int low_feature_db;
        int mature_thresh;
        int mature_thresh_low;
        double map_jump_thresh;
        int active_mature_thresh;
        int health_min_stable;
        int health_min_db;
        int health_hold_frames;
        int health_start_frame;
    };

    FeatureManager(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
               std::vector<Eigen::Matrix4d> T_C_B,
               const Options& options = Options());

    // 核心处理函数
    void processStereo(const ov_core::CameraData& message,
                       const Pose& mapping_pose,
                       std::vector<observer::VisualObservation>& observations,
                       FeatureDiagnostics* diagnostics = nullptr);

    const std::map<size_t, Eigen::Vector3d> get_active_map() const;
    std::shared_ptr<ov_core::TrackKLT> get_tracker() { return tracker; }
    double get_last_median_disparity() const { return last_median_disparity_; }
    int get_last_common_track_count() const { return last_common_track_count_; }

private:
    std::shared_ptr<ov_core::TrackKLT> tracker;
    Options options_;
    std::vector<std::shared_ptr<ov_core::CamBase>> cameras;
    std::vector<Eigen::Matrix4d> T_C_B; // Cam to Body

    LandmarkMap landmark_map_;
    StereoTriangulator triangulator_;
    FeatureHealth feature_health_;

    // RANSAC 辅助
    std::map<size_t, cv::Point2f> history_obs;
    double last_median_disparity_ = std::numeric_limits<double>::infinity();
    int last_common_track_count_ = 0;
    // 将点投影并检查误差
    bool check_reprojection(const Eigen::Vector3d& p_w,
                            const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                            const Eigen::Vector3d& uv_meas_norm,
                            double reproj_thresh,
                            double* reproj_err = nullptr);
};

} // namespace hno_vio::frontend
#endif
