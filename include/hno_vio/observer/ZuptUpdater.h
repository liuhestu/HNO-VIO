#ifndef HNO_VIO_OBSERVER_ZUPT_UPDATER_H
#define HNO_VIO_OBSERVER_ZUPT_UPDATER_H

#include <memory>

#include "hno_vio/State.h"

namespace hno_vio::observer {

class ZuptUpdater {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void setVelocityNoise(double velocity_noise) { velocity_noise_ = velocity_noise; }
    bool update(std::shared_ptr<State> state) const;

private:
    double velocity_noise_ = 0.05;
};

} // namespace hno_vio::observer

#endif
