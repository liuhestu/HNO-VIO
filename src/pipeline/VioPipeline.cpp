#include "hno_vio/pipeline/VioPipeline.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace hno_vio::pipeline {

VioPipeline::VioPipeline(std::vector<std::shared_ptr<ov_core::CamBase>> cameras,
                         std::vector<Eigen::Matrix4d> camera_to_body,
                         const VioPipelineOptions& options)
    : committed_state_(std::make_shared<State>()),
      feature_manager_(cameras, camera_to_body, options.frontend),
      zupt_options_(options.zupt),
      experiment_fix_e_hat_(options.experiment_fix_e_hat) {
    initializer_.setOptions(options.initializer_window_size,
                            options.initializer_acc_variance,
                            options.initializer_gyro_variance);
    propagator_.setNoiseParams(options.noise);
    propagator_.setExperimentOptions(options.experiment_fix_e_hat,
                                     options.experiment_force_sigma_r_zero);
    updater_.setOptions(options.updater);
    std::map<size_t, Eigen::Matrix4d> extrinsics;
    for (size_t i = 0; i < camera_to_body.size(); ++i) {
        extrinsics[i] = camera_to_body[i];
    }
    updater_.setExtrinsics(extrinsics);
    zupt_updater_.setVelocityNoise(options.updater.zupt_velocity_noise);
    zupt_options_.imu_window_size = std::max(2, zupt_options_.imu_window_size);
    zupt_options_.min_tracks = std::max(1, zupt_options_.min_tracks);
    zupt_options_.hold_frames = std::max(1, zupt_options_.hold_frames);
}

std::optional<PipelineResult> VioPipeline::processImu(const ov_core::ImuData& imu) {
    imu_buffer_.insert(imu);
    if (!initialized_) {
        initializer_.feedImuData(imu);
        if (initializer_.initialize(committed_state_, committed_time_)) {
            if (experiment_fix_e_hat_) committed_state_->fixEBasis();
            initialized_ = true;
            prediction_state_ = *committed_state_;
            prediction_time_ = committed_time_;
            imu_buffer_.discardBefore(committed_time_, true);
            resetZuptDetector();
        }
        return std::nullopt;
    }

    if (zupt_options_.enabled) {
        zupt_imu_window_.push_back(imu);
        const size_t retained_samples =
            static_cast<size_t>(4 * zupt_options_.imu_window_size);
        while (zupt_imu_window_.size() > retained_samples) {
            zupt_imu_window_.pop_front();
        }
    }

    rebuildPrediction();

    PipelineResult result;
    result.timestamp = prediction_time_;
    result.state = prediction_state_;
    result.camera_frame = false;
    result.diagnostics.initialized = true;
    result.diagnostics.timestamp = prediction_time_;
    result.diagnostics.committed_timestamp = committed_time_;
    result.diagnostics.prediction_timestamp = prediction_time_;
    result.diagnostics.frame_index = frame_index_;
    result.diagnostics.stage = "imu_prediction";
    return result;
}

bool VioPipeline::propagateState(State& state,
                                 double start_time,
                                 double end_time,
                                 PropagationDiagnostics* diagnostics) {
    const auto segment = imu_buffer_.integrationSegment(start_time, end_time);
    if (!segment) return false;
    if (diagnostics) *diagnostics = PropagationDiagnostics{};
    auto propagated = std::make_shared<State>(state);
    for (size_t i = 1; i < segment->size(); ++i) {
        const double dt = segment->at(i).timestamp - segment->at(i - 1).timestamp;
        if (dt > 1e-9) {
            PropagationDiagnostics step_diagnostics;
            propagator_.propagate(propagated,
                                  segment->at(i).wm,
                                  segment->at(i).am,
                                  dt,
                                  diagnostics ? &step_diagnostics : nullptr);
            if (diagnostics) {
                diagnostics->sigma_r_raw = step_diagnostics.sigma_r_raw;
                diagnostics->sigma_r_applied = step_diagnostics.sigma_r_applied;
                diagnostics->sigma_r_raw_max =
                    std::max(diagnostics->sigma_r_raw_max,
                             step_diagnostics.sigma_r_raw_max);
                diagnostics->sigma_r_applied_max =
                    std::max(diagnostics->sigma_r_applied_max,
                             step_diagnostics.sigma_r_applied_max);
                diagnostics->sigma_r_raw_integral +=
                    step_diagnostics.sigma_r_raw_integral;
                diagnostics->sigma_r_applied_integral +=
                    step_diagnostics.sigma_r_applied_integral;
                diagnostics->sample_count += step_diagnostics.sample_count;
            }
        }
    }
    state = *propagated;
    return true;
}

void VioPipeline::rebuildPrediction() {
    prediction_state_ = *committed_state_;
    prediction_time_ = committed_time_;
    if (imu_buffer_.empty() || imu_buffer_.latestTime() <= committed_time_ + 1e-9) return;
    const double target_time = imu_buffer_.latestTime();
    if (propagateState(prediction_state_, committed_time_, target_time)) {
        prediction_time_ = target_time;
    }
}

bool VioPipeline::canProcessStereo(double timestamp) const {
    return initialized_ && timestamp > committed_time_ + 1e-9 &&
           imu_buffer_.covers(committed_time_, timestamp);
}

Pose VioPipeline::selectMappingPose(const std::optional<Pose>& raw_gt_pose,
                                    const State& camera_state) {
    if (!raw_gt_pose) {
        return Pose::FromState(camera_state);
    }
    if (!has_gt_mapping_alignment_) {
        R_estimator_gt_ = camera_state.R_hat_B2I * raw_gt_pose->R.transpose();
        t_estimator_gt_ = camera_state.p_hat - R_estimator_gt_ * raw_gt_pose->p;
        has_gt_mapping_alignment_ = true;
        std::cout << "[VioPipeline] initialized GT mapping-frame alignment" << std::endl;
    }
    return Pose{R_estimator_gt_ * raw_gt_pose->R,
                R_estimator_gt_ * raw_gt_pose->p + t_estimator_gt_};
}

std::optional<PipelineResult> VioPipeline::processStereo(
    const ov_core::CameraData& stereo, const std::optional<Pose>& raw_gt_pose) {
    if (!canProcessStereo(stereo.timestamp)) return std::nullopt;

    auto camera_state = std::make_shared<State>(*committed_state_);
    PropagationDiagnostics propagation_diagnostics;
    if (!propagateState(*camera_state, committed_time_, stereo.timestamp,
                        &propagation_diagnostics)) {
        return std::nullopt;
    }

    std::vector<observer::VisualObservation> observations;
    const Pose mapping_pose = selectMappingPose(raw_gt_pose, *camera_state);
    FeatureDiagnostics frontend_diagnostics;
    feature_manager_.processStereo(stereo, mapping_pose, observations,
                                   &frontend_diagnostics);
    UpdaterDiagnostics updater_diagnostics;
    const bool visual_applied = updater_.update(camera_state, observations, &updater_diagnostics);

    double acc_variance = 0.0;
    double gyro_variance = 0.0;
    const bool stationary = checkZuptStationary(stereo.timestamp,
                                                acc_variance,
                                                gyro_variance);
    if (stationary) {
        ++zupt_stationary_streak_;
    } else {
        zupt_stationary_streak_ = 0;
        zupt_active_ = false;
    }

    bool zupt_applied = false;
    const double velocity_residual_norm = camera_state->v_hat.norm();
    const bool should_enter = stationary &&
        zupt_stationary_streak_ >= zupt_options_.hold_frames &&
        camera_state->v_hat.norm() >= zupt_options_.activation_speed;
    if ((should_enter || zupt_active_) && stationary) {
        zupt_active_ = true;
        const State before = *camera_state;
        zupt_applied = zupt_updater_.update(camera_state);
        const double position_delta = (camera_state->p_hat - before.p_hat).norm();
        const double rotation_delta = std::abs(Eigen::AngleAxisd(
            before.R_hat_B2I.transpose() * camera_state->R_hat_B2I).angle());
        if (!zupt_applied || position_delta > 1e-9 || rotation_delta > 1e-8) {
            *camera_state = before;
            zupt_active_ = false;
            zupt_applied = false;
        }
    }

    if (experiment_fix_e_hat_) {
        camera_state->fixEBasis();
    }
    *committed_state_ = *camera_state;
    committed_time_ = stereo.timestamp;
    imu_buffer_.discardBefore(committed_time_, true);
    rebuildPrediction();

    ++frame_index_;
    PipelineResult result;
    result.timestamp = committed_time_;
    result.state = *committed_state_;
    result.camera_frame = true;
    result.visual_update_applied = visual_applied;
    result.zupt_update_applied = zupt_applied;
    result.observation_count = static_cast<int>(observations.size());
    result.active_landmarks = feature_manager_.get_active_map();
    result.tracker = feature_manager_.get_tracker();
    result.diagnostics.initialized = true;
    result.diagnostics.timestamp = committed_time_;
    result.diagnostics.committed_timestamp = committed_time_;
    result.diagnostics.prediction_timestamp = prediction_time_;
    result.diagnostics.frame_index = frame_index_;
    result.diagnostics.observation_count = result.observation_count;
    result.diagnostics.visual_update_applied = visual_applied;
    result.diagnostics.zupt_update_applied = zupt_applied;
    result.diagnostics.stage = "camera_committed";
    result.diagnostics.frontend = frontend_diagnostics;
    result.diagnostics.updater = updater_diagnostics;
    result.diagnostics.propagation = propagation_diagnostics;
    result.diagnostics.zupt.accelerometer_variance = acc_variance;
    result.diagnostics.zupt.gyroscope_variance = gyro_variance;
    result.diagnostics.zupt.stationary_detected = stationary;
    result.diagnostics.zupt.active = zupt_active_;
    result.diagnostics.zupt.stationary_streak = zupt_stationary_streak_;
    result.diagnostics.zupt.velocity_residual_norm = velocity_residual_norm;
    result.diagnostics.zupt.update_applied = zupt_applied;
    result.latest_prediction = prediction_state_;
    result.latest_prediction_timestamp = prediction_time_;
    return result;
}

void VioPipeline::resetZuptDetector() {
    zupt_imu_window_.clear();
    zupt_stationary_streak_ = 0;
    zupt_active_ = false;
}

bool VioPipeline::checkZuptStationary(double timestamp,
                                      double& acc_variance,
                                      double& gyro_variance) const {
    acc_variance = 0.0;
    gyro_variance = 0.0;
    std::vector<ov_core::ImuData> eligible;
    eligible.reserve(zupt_imu_window_.size());
    for (const auto& imu : zupt_imu_window_) {
        if (imu.timestamp <= timestamp + 1e-9) eligible.push_back(imu);
    }
    if (eligible.size() > static_cast<size_t>(zupt_options_.imu_window_size)) {
        eligible.erase(eligible.begin(),
                       eligible.end() - zupt_options_.imu_window_size);
    }
    if (!zupt_options_.enabled ||
        eligible.size() < static_cast<size_t>(zupt_options_.imu_window_size) ||
        feature_manager_.get_last_common_track_count() < zupt_options_.min_tracks ||
        !std::isfinite(feature_manager_.get_last_median_disparity()) ||
        feature_manager_.get_last_median_disparity() > zupt_options_.max_disparity) {
        return false;
    }

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyro = Eigen::Vector3d::Zero();
    for (const auto& imu : eligible) {
        mean_acc += imu.am;
        mean_gyro += imu.wm;
    }
    mean_acc /= static_cast<double>(eligible.size());
    mean_gyro /= static_cast<double>(eligible.size());
    for (const auto& imu : eligible) {
        acc_variance += (imu.am - mean_acc).squaredNorm();
        gyro_variance += (imu.wm - mean_gyro).squaredNorm();
    }
    acc_variance /= static_cast<double>(eligible.size());
    gyro_variance /= static_cast<double>(eligible.size());
    return acc_variance <= zupt_options_.acc_variance_threshold &&
           gyro_variance <= zupt_options_.gyro_variance_threshold;
}

} // namespace hno_vio::pipeline
