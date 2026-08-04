#ifndef HNO_VIO_DIAGNOSTICS_H
#define HNO_VIO_DIAGNOSTICS_H

#include <optional>
#include <string>

#include "hno_vio/State.h"

namespace hno_vio {

struct DiagnosticsOptions {
    bool frontend_print = false;
    bool essential_print = true;
    bool updater_print = false;
    bool zupt_print = false;
    bool pipeline_print = false;
};

struct UpdaterDiagnostics {
    int total_observations = 0;
    int chi2_passed_observations = 0;
    int chi2_rejected_observations = 0;
    int applied_observations = 0;
    int numerical_rejected_observations = 0;
    int kalman_gain_rejected_observations = 0;
    int delta_rejected_observations = 0;
    int low_observation_streak = 0;
    double max_chi2 = 0.0;
    double max_rejected_chi2 = 0.0;
    double max_position_delta = 0.0;
    double max_rotation_delta = 0.0;
    double visual_delta_r = 0.0;
    double visual_delta_p = 0.0;
    double visual_delta_v = 0.0;
    double visual_delta_e = 0.0;
    double projection_correction = 0.0;
    Eigen::Matrix3d e_before = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d e_raw = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d e_projected = Eigen::Matrix3d::Identity();
    bool low_observation = false;
    bool skipped_for_low_observations = false;
    bool large_delta_warning = false;
    bool update_applied = false;
};

struct PropagationDiagnostics {
    Eigen::Vector3d sigma_r_raw = Eigen::Vector3d::Zero();
    Eigen::Vector3d sigma_r_applied = Eigen::Vector3d::Zero();
    double sigma_r_raw_max = 0.0;
    double sigma_r_applied_max = 0.0;
    double sigma_r_raw_integral = 0.0;
    double sigma_r_applied_integral = 0.0;
    int sample_count = 0;
};

struct ZuptDiagnostics {
    double accelerometer_variance = 0.0;
    double gyroscope_variance = 0.0;
    bool stationary_detected = false;
    bool active = false;
    int stationary_streak = 0;
    bool update_applied = false;
    double velocity_residual_norm = 0.0;
};

struct FeatureDiagnostics {
    int tracked_count = 0;
    int common_track_count = 0;
    int new_count = 0;
    int stable_count = 0;
    int landmark_map_size = 0;
    int stereo_passed = 0;
    int stereo_rejected = 0;
    int reprojection_passed = 0;
    int reprojection_rejected = 0;
    int unhealthy_streak = 0;
    double stereo_error_mean = 0.0;
    double stereo_error_max = 0.0;
    double reprojection_error_mean = 0.0;
    double reprojection_error_max = 0.0;
    double median_disparity = 0.0;
    bool low_feature_mode = false;
    bool health_guard_active = false;
    bool health_low_stable = false;
    bool health_low_landmarks = false;
    bool visual_update_allowed = true;
};

struct PipelineDiagnostics {
    bool initialized = false;
    double timestamp = -1.0;
    double committed_timestamp = -1.0;
    double prediction_timestamp = -1.0;
    int frame_index = 0;
    int observation_count = 0;
    bool visual_update_applied = false;
    bool zupt_update_applied = false;
    std::string stage;
    FeatureDiagnostics frontend;
    UpdaterDiagnostics updater;
    PropagationDiagnostics propagation;
    ZuptDiagnostics zupt;
};

class Diagnostics {
public:
    explicit Diagnostics(const DiagnosticsOptions& options = DiagnosticsOptions());
    void report(const PipelineDiagnostics& diagnostics,
                const Pose& estimated_pose,
                const std::optional<Pose>& aligned_ground_truth);

private:
    DiagnosticsOptions options_;
    bool first_low_map_reported_ = false;
    bool first_zero_stable_reported_ = false;
    bool first_low_observations_reported_ = false;
    bool first_large_delta_reported_ = false;
};

} // namespace hno_vio

#endif
