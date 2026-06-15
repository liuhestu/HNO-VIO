#ifndef HNO_FEATURE_H
#define HNO_FEATURE_H

#include <memory>
#include <vector>
#include <map>
#include <deque>
#include <Eigen/Dense>

#include "track/TrackKLT.h"
#include "cam/CamBase.h"
#include "utils/sensor_data.h"
#include "HNOUpdater.h"
#include <opencv2/core/types.hpp>

namespace ov_hno {

/* Removed complex FeaturePerId struct, simplified to direct map */

class HNOFeature {
public:
    HNOFeature(std::vector<std::shared_ptr<ov_core::CamBase>> cams,
               std::vector<Eigen::Matrix4d> T_C_B);

    void feed_measurement(const ov_core::CameraData& message, 
                          Eigen::Matrix3d R_wb, Eigen::Vector3d p_wb,
                          std::vector<HNOObservation>& observations);

    const std::map<size_t, Eigen::Vector3d> get_active_map() const;
    std::shared_ptr<ov_core::TrackKLT> get_tracker() { return tracker; }

private:
    std::shared_ptr<ov_core::TrackKLT> tracker;
    std::vector<std::shared_ptr<ov_core::CamBase>> cameras;
    std::vector<Eigen::Matrix4d> T_C_B; // Cam to Body

    // Core Database: ID -> World Position
    std::map<size_t, Eigen::Vector3d> feature_map;
    
    // For Optical Flow Check / RANSAC
    std::map<size_t, cv::Point2f> history_obs; 

    // Stereo Triangulation
    bool triangulate_stereo(const Eigen::Vector3d& uv_left, 
                           const Eigen::Vector3d& uv_right, 
                           Eigen::Vector3d& p_c_left);
};

}
#endif