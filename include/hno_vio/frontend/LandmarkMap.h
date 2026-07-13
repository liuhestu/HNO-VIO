#ifndef HNO_VIO_FRONTEND_LANDMARK_MAP_H
#define HNO_VIO_FRONTEND_LANDMARK_MAP_H

#include <map>
#include <unordered_set>

#include <Eigen/Dense>

namespace hno_vio::frontend {

struct Landmark {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
    int track_count = 0;
    int fail_count = 0;
};

class LandmarkMap {
public:
    bool contains(size_t id) const { return landmarks_.count(id) != 0; }
    Landmark& at(size_t id) { return landmarks_.at(id); }
    const Landmark& at(size_t id) const { return landmarks_.at(id); }
    void insert(size_t id, const Landmark& landmark) { landmarks_[id] = landmark; }
    void eraseMissing(const std::unordered_set<size_t>& alive_ids);
    std::map<size_t, Eigen::Vector3d> activeMap(int mature_threshold) const;
    size_t size() const { return landmarks_.size(); }

private:
    std::map<size_t, Landmark> landmarks_;
};

} // namespace hno_vio::frontend

#endif
