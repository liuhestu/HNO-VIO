#include "hno_vio/frontend/LandmarkMap.h"

namespace hno_vio::frontend {

void LandmarkMap::eraseMissing(const std::unordered_set<size_t>& alive_ids) {
    for (auto it = landmarks_.begin(); it != landmarks_.end();) {
        if (alive_ids.count(it->first) == 0) {
            it = landmarks_.erase(it);
        } else {
            ++it;
        }
    }
}

std::map<size_t, Eigen::Vector3d> LandmarkMap::activeMap(int mature_threshold) const {
    std::map<size_t, Eigen::Vector3d> active;
    for (const auto& [id, landmark] : landmarks_) {
        if (landmark.track_count >= mature_threshold) {
            active[id] = landmark.p_world;
        }
    }
    return active;
}

} // namespace hno_vio::frontend
