#include "hno_vio/pipeline/VioPipeline.h"

#include <algorithm>
#include <iostream>

namespace hno_vio::pipeline {

VioPipeline::VioPipeline(std::vector<std::shared_ptr<ov_core::CamBase>> cameras,
                         std::vector<Eigen::Matrix4d> camera_to_body,
                         const VioPipelineOptions& options)
    : committed_state_(std::make_shared<State>()),
      feature_manager_(cameras, camera_to_body, options.frontend),
      fix_e_hat_(options.fix_e_hat) {
    initializer_.setOptions(options.initializer_window_size,
                            options.initializer_acc_variance,
                            options.initializer_gyro_variance);
    propagator_.setNoiseParams(options.noise);
    propagator_.setBehaviorOptions(options.fix_e_hat, options.sigma_r_zero);
    updater_.setOptions(options.updater);
    std::map<size_t, Eigen::Matrix4d> extrinsics;
    for (size_t i = 0; i < camera_to_body.size(); ++i) {
        extrinsics[i] = camera_to_body[i];
    }
    updater_.setExtrinsics(extrinsics);
}

std::optional<PipelineResult> VioPipeline::processImu(const ov_core::ImuData& imu) {
    imu_buffer_.insert(imu);
    if (!initialized_) {
        initializer_.feedImuData(imu);
        if (initializer_.initialize(committed_state_, committed_time_)) {
            if (fix_e_hat_) committed_state_->fixEBasis();
            initialized_ = true;
            prediction_state_ = *committed_state_;
            prediction_time_ = committed_time_;
            imu_buffer_.discardBefore(committed_time_, true);
        }
        return std::nullopt;
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

    if (fix_e_hat_) {
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
    result.diagnostics.stage = "camera_committed";
    result.diagnostics.frontend = frontend_diagnostics;
    result.diagnostics.updater = updater_diagnostics;
    result.diagnostics.propagation = propagation_diagnostics;
    result.latest_prediction = prediction_state_;
    result.latest_prediction_timestamp = prediction_time_;
    return result;
}

} // namespace hno_vio::pipeline
