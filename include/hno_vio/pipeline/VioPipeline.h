#ifndef HNO_VIO_PIPELINE_VIO_PIPELINE_H
#define HNO_VIO_PIPELINE_VIO_PIPELINE_H

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "cam/CamBase.h"
#include "track/TrackKLT.h"
#include "utils/sensor_data.h"

#include "hno_vio/Diagnostics.h"
#include "hno_vio/Initializer.h"
#include "hno_vio/State.h"
#include "hno_vio/frontend/FeatureManager.h"
#include "hno_vio/observer/Propagator.h"
#include "hno_vio/observer/Updater.h"
#include "hno_vio/pipeline/ImuBuffer.h"

namespace hno_vio::pipeline {

struct VioPipelineOptions {
    size_t initializer_window_size = 250;
    double initializer_acc_variance = 1.5;
    double initializer_gyro_variance = 0.01;
    observer::Propagator::NoiseParams noise;
    frontend::FeatureManager::Options frontend;
    observer::Updater::Options updater;
    bool fix_e_hat = false;
    bool sigma_r_zero = false;
};

struct PipelineResult {
    double timestamp = -1.0;
    State state;
    bool camera_frame = false;
    bool visual_update_applied = false;
    int observation_count = 0;
    std::map<size_t, Eigen::Vector3d> active_landmarks;
    std::shared_ptr<ov_core::TrackKLT> tracker;
    PipelineDiagnostics diagnostics;
    std::optional<State> latest_prediction;
    double latest_prediction_timestamp = -1.0;
};

class VioPipeline {
public:
    VioPipeline(std::vector<std::shared_ptr<ov_core::CamBase>> cameras,
                std::vector<Eigen::Matrix4d> camera_to_body,
                const VioPipelineOptions& options);

    std::optional<PipelineResult> processImu(const ov_core::ImuData& imu);
    std::optional<PipelineResult> processStereo(const ov_core::CameraData& stereo,
                                                const std::optional<Pose>& raw_gt_pose);

    bool initialized() const { return initialized_; }
    bool canProcessStereo(double timestamp) const;
    const State& committedState() const { return *committed_state_; }
    double committedTime() const { return committed_time_; }

private:
    Pose selectMappingPose(const std::optional<Pose>& raw_gt_pose,
                           const State& camera_state);
    bool propagateState(State& state,
                        double start_time,
                        double end_time,
                        PropagationDiagnostics* diagnostics = nullptr);
    void rebuildPrediction();

    // Camera-corrected state is authoritative. IMU prediction is always rebuilt
    // from this state and may be revised by the next camera correction.
    std::shared_ptr<State> committed_state_;
    State prediction_state_;
    double committed_time_ = -1.0;
    double prediction_time_ = -1.0;
    Initializer initializer_;
    ImuBuffer imu_buffer_;
    observer::Propagator propagator_;
    observer::Updater updater_;
    frontend::FeatureManager feature_manager_;
    bool initialized_ = false;
    int frame_index_ = 0;
    bool fix_e_hat_ = false;

    bool has_gt_mapping_alignment_ = false;
    Eigen::Matrix3d R_estimator_gt_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_estimator_gt_ = Eigen::Vector3d::Zero();
};

} // namespace hno_vio::pipeline

#endif
