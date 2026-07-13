#ifndef HNO_VIO_DIAGNOSTICS_H
#define HNO_VIO_DIAGNOSTICS_H

#include <string>

namespace hno_vio {

struct UpdaterDiagnostics {
    int total_observations = 0;
    int chi2_passed_observations = 0;
    int applied_observations = 0;
    int numerical_rejected_observations = 0;
    int kalman_gain_rejected_observations = 0;
    int delta_rejected_observations = 0;
    bool update_applied = false;
};

struct ZuptDiagnostics {
    bool stationary_detected = false;
    bool update_applied = false;
    double velocity_residual_norm = 0.0;
};

struct FeatureDiagnostics {
    int tracked_count = 0;
    int common_track_count = 0;
    int stable_count = 0;
    int landmark_map_size = 0;
    bool visual_update_allowed = true;
};

struct PipelineDiagnostics {
    bool initialized = false;
    double timestamp = -1.0;
    int frame_index = 0;
    std::string stage;
};

class Diagnostics {
public:
    void printPipeline(const PipelineDiagnostics& diagnostics) const;
};

} // namespace hno_vio

#endif
