#ifndef HNO_VIO_FRONTEND_FEATURE_HEALTH_H
#define HNO_VIO_FRONTEND_FEATURE_HEALTH_H

namespace hno_vio::frontend {

class FeatureHealth {
public:
    struct Constraints {
        int start_frame = 60;
        int min_stable_features = 20;
        int min_landmark_count = 20;
        int hold_frames = 3;
    };

    struct Status {
        int frame_id = 0;
        int stable_count = 0;
        int landmark_count = 0;
        int unhealthy_streak = 0;
        bool guard_active = false;
        bool low_stable = false;
        bool low_landmarks = false;
        bool allow_visual_update = true;
    };

    explicit FeatureHealth(const Constraints& constraints) : constraints_(constraints) {}
    Status update(int stable_count, int landmark_count);

private:
    Constraints constraints_;
    int frame_id_ = 0;
    int unhealthy_streak_ = 0;
};

} // namespace hno_vio::frontend

#endif
