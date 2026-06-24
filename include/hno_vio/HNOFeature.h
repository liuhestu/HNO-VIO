#ifndef HNO_FEATURE_H
#define HNO_FEATURE_H

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "track/TrackKLT.h"
#include "cam/CamBase.h"
#include "hno_vio/HNOUpdater.h"
#include <opencv2/core/types.hpp>

namespace hno_vio {

// 内部管理的特征点结构（简化版：直接维护当前估计的世界坐标）
struct FeatureInfo {
    Eigen::Vector3d p_w;     // 世界系坐标（基于最近一次可信双目观测）
    int track_count;         // 成功跟踪帧计数
    int fail_count;          // 连续失败计数
};

class HNOFeature {
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
              fail_limit(5),
              fail_limit_low(8),
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
        int fail_limit;
        int fail_limit_low;
        double map_jump_thresh;
        int active_mature_thresh;
        int health_min_stable;
        int health_min_db;
        int health_hold_frames;
        int health_start_frame;
    };

    HNOFeature(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
               std::vector<Eigen::Matrix4d> T_C_B,
               const Options& options = Options());

    // 核心处理函数
    void feed_measurement(const ov_core::CameraData& message, 
                          Eigen::Matrix3d R_est, Eigen::Vector3d p_est,
                          Eigen::Matrix3d R_gt,  Eigen::Vector3d p_gt, // 兼容接口，GT仅可选
                          std::vector<HNOObservation>& observations);

    const std::map<size_t, Eigen::Vector3d> get_active_map() const;
    std::shared_ptr<ov_core::TrackKLT> get_tracker() { return tracker; }

private:
    std::shared_ptr<ov_core::TrackKLT> tracker;
    Options options_;
    std::vector<std::shared_ptr<ov_core::CamBase>> cameras;
    std::vector<Eigen::Matrix4d> T_C_B; // Cam to Body

    // 核心数据库: ID -> FeatureInfo
    std::map<size_t, FeatureInfo> feature_db;
    
    // RANSAC 辅助
    std::map<size_t, cv::Point2f> history_obs; 

    // 双目三角化 (返回相机系坐标)
    bool triangulate_stereo(const Eigen::Vector3d& uv_left, 
                           const Eigen::Vector3d& uv_right, 
                           Eigen::Vector3d& p_c_left,
                           double* reproj_err_right = nullptr);
                           
    // 将点投影并检查误差
    bool check_reprojection(size_t id, const Eigen::Vector3d& p_w, 
                            const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                            const Eigen::Vector3d& uv_meas_norm,
                            double reproj_thresh,
                            double* reproj_err = nullptr);
};

}
#endif
