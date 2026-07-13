#include "hno_vio/frontend/FeatureHealth.h"

namespace hno_vio::frontend {

FeatureHealth::Status FeatureHealth::update(int stable_count, int landmark_count) {
    ++frame_id_;
    Status status;
    status.frame_id = frame_id_;
    status.stable_count = stable_count;
    status.landmark_count = landmark_count;
    status.guard_active = frame_id_ >= constraints_.start_frame;
    status.low_stable = stable_count < constraints_.min_stable_features;
    status.low_landmarks = landmark_count < constraints_.min_landmark_count;

    if (status.guard_active && (status.low_stable || status.low_landmarks)) {
        ++unhealthy_streak_;
    } else if (!status.low_stable && !status.low_landmarks) {
        unhealthy_streak_ = 0;
    }
    status.unhealthy_streak = unhealthy_streak_;
    status.allow_visual_update =
        !(status.guard_active && unhealthy_streak_ >= constraints_.hold_frames);
    return status;
}

} // namespace hno_vio::frontend
