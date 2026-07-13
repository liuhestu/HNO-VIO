#ifndef HNO_VIO_PIPELINE_IMU_BUFFER_H
#define HNO_VIO_PIPELINE_IMU_BUFFER_H

#include <optional>
#include <vector>

#include "utils/sensor_data.h"

namespace hno_vio::pipeline {

class ImuBuffer {
public:
    void insert(const ov_core::ImuData& imu);
    bool covers(double start_time, double end_time) const;
    std::optional<std::vector<ov_core::ImuData>> integrationSegment(
        double start_time, double end_time) const;
    std::vector<ov_core::ImuData> samplesAfter(double timestamp) const;
    void discardBefore(double timestamp, bool keep_boundary = true);

    bool empty() const { return data_.empty(); }
    double latestTime() const;
    const ov_core::ImuData& back() const { return data_.back(); }
    const std::vector<ov_core::ImuData>& data() const { return data_; }

private:
    std::optional<ov_core::ImuData> sampleAt(double timestamp) const;

    std::vector<ov_core::ImuData> data_;
};

} // namespace hno_vio::pipeline

#endif
