#include "hno_vio/Diagnostics.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace hno_vio {

Diagnostics::Diagnostics(const DiagnosticsOptions& options) : options_(options) {}

void Diagnostics::report(const PipelineDiagnostics& diagnostics,
                         const Pose& estimated_pose,
                         const std::optional<Pose>& aligned_ground_truth) {
    const FeatureDiagnostics& frontend = diagnostics.frontend;
    const UpdaterDiagnostics& updater = diagnostics.updater;

    /* 一次性异常提示 */
    if (frontend.health_guard_active &&
        (frontend.health_low_stable || frontend.health_low_landmarks) &&
        !first_low_map_reported_) {
        first_low_map_reported_ = true;
        std::cout << "[Diagnostics] first_low_map frame " << diagnostics.frame_index
                  << " stable " << frontend.stable_count
                  << " db " << frontend.landmark_map_size
                  << " streak " << frontend.unhealthy_streak
                  << std::endl;
    }
    if (frontend.health_guard_active && frontend.stable_count == 0 &&
        !first_zero_stable_reported_) {
        first_zero_stable_reported_ = true;
        std::cout << "[Diagnostics] first_zero_stable frame " << diagnostics.frame_index
                  << " db " << frontend.landmark_map_size
                  << " pts " << frontend.tracked_count
                  << std::endl;
    }
    if (updater.low_observation && !first_low_observations_reported_) {
        first_low_observations_reported_ = true;
        std::cout << "[Diagnostics] first_low_observations frame "
                  << diagnostics.frame_index
                  << " obs " << updater.total_observations
                  << " streak " << updater.low_observation_streak
                  << std::endl;
    }
    if (updater.large_delta_warning && !first_large_delta_reported_) {
        first_large_delta_reported_ = true;
        std::cout << std::fixed << std::setprecision(3)
                  << "[Diagnostics] first_large_delta frame "
                  << diagnostics.frame_index
                  << " obs " << updater.total_observations
                  << " dP " << updater.max_position_delta
                  << " dR " << updater.max_rotation_delta
                  << std::endl;
    }

    /* 30帧周期日志 */
    if (diagnostics.frame_index <= 0 || diagnostics.frame_index % 30 != 0) return;

    if (options_.essential_print) {
        const double roll = std::atan2(estimated_pose.R(2, 1), estimated_pose.R(2, 2));
        const double sin_pitch = std::clamp(-estimated_pose.R(2, 0), -1.0, 1.0);
        const double pitch = std::asin(sin_pitch);
        const double yaw = std::atan2(estimated_pose.R(1, 0), estimated_pose.R(0, 0));
        const Eigen::Vector3d roll_pitch_yaw_deg =
            Eigen::Vector3d(roll, pitch, yaw) * (180.0 / std::acos(-1.0));
        std::cout << std::fixed << std::setprecision(3)
                  << "[Diagnostics][Essential] updater_feats="
                  << updater.total_observations
                  << " flow_tracks=" << frontend.common_track_count
                  << " rpy_deg=(" << roll_pitch_yaw_deg.x() << ","
                  << roll_pitch_yaw_deg.y() << "," << roll_pitch_yaw_deg.z() << ")"
                  << " position=(" << estimated_pose.p.x() << ","
                  << estimated_pose.p.y() << "," << estimated_pose.p.z() << ")";
        if (aligned_ground_truth) {
            const Eigen::Vector3d error = estimated_pose.p - aligned_ground_truth->p;
            std::cout << " Err=" << error.norm()
                      << " (xyz=(" << error.x() << "," << error.y() << ","
                      << error.z() << "))";
        } else {
            std::cout << " Err=N/A error_xyz=(N/A,N/A,N/A)";
        }
        std::cout << std::endl;
    }
    if (options_.pipeline_print) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[Diagnostics][Pipeline] frame " << diagnostics.frame_index
                  << " stage " << diagnostics.stage
                  << " committed " << diagnostics.committed_timestamp
                  << " prediction " << diagnostics.prediction_timestamp
                  << " obs " << diagnostics.observation_count
                  << " visual " << (diagnostics.visual_update_applied ? "applied" : "not_applied")
                  << std::endl;
    }
    if (options_.frontend_print) {
        std::cout << "[Diagnostics][Frontend] tracked " << frontend.tracked_count
                  << " common " << frontend.common_track_count
                  << " new " << frontend.new_count
                  << " stable " << frontend.stable_count
                  << " db " << frontend.landmark_map_size
                  << " low_mode " << (frontend.low_feature_mode ? "true" : "false")
                  << " stereo " << frontend.stereo_passed << "/"
                  << frontend.stereo_passed + frontend.stereo_rejected
                  << " stereo_err_mean " << frontend.stereo_error_mean
                  << " stereo_err_max " << frontend.stereo_error_max
                  << " reproj " << frontend.reprojection_passed << "/"
                  << frontend.reprojection_passed + frontend.reprojection_rejected
                  << " reproj_err_mean " << frontend.reprojection_error_mean
                  << " reproj_err_max " << frontend.reprojection_error_max
                  << " health " << (frontend.visual_update_allowed ? "healthy" : "suppressed")
                  << " guard " << (frontend.health_guard_active ? "active" : "inactive")
                  << " low_stable " << (frontend.health_low_stable ? "true" : "false")
                  << " low_landmarks " << (frontend.health_low_landmarks ? "true" : "false")
                  << " health_streak " << frontend.unhealthy_streak
                  << " disparity " << frontend.median_disparity
                  << std::endl;
    }
    if (options_.updater_print) {
        std::cout << "[Diagnostics][Updater] total " << updater.total_observations
                  << " chi2_pass " << updater.chi2_passed_observations
                  << " chi2_reject " << updater.chi2_rejected_observations
                  << " numerical_reject " << updater.numerical_rejected_observations
                  << " gain_reject " << updater.kalman_gain_rejected_observations
                  << " delta_reject " << updater.delta_rejected_observations
                  << " chi2_max " << updater.max_chi2
                  << " rejected_chi2_max " << updater.max_rejected_chi2
                  << " dP_max " << updater.max_position_delta
                  << " dR_max " << updater.max_rotation_delta
                  << " low_streak " << updater.low_observation_streak
                  << " low_skip " << (updater.skipped_for_low_observations ? "true" : "false")
                  << " applied " << updater.applied_observations
                  << std::endl;
    }
}

} // namespace hno_vio
