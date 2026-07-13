#include "hno_vio/pipeline/ImuBuffer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hno_vio::pipeline {
namespace {

constexpr double kTimestampEpsilon = 1e-9;

bool sameTimestamp(double lhs, double rhs) {
    return std::abs(lhs - rhs) <= kTimestampEpsilon;
}

} // namespace

void ImuBuffer::insert(const ov_core::ImuData& imu) {
    const auto position = std::lower_bound(
        data_.begin(), data_.end(), imu.timestamp,
        [](const ov_core::ImuData& sample, double timestamp) {
            return sample.timestamp < timestamp;
        });
    if (position != data_.end() && sameTimestamp(position->timestamp, imu.timestamp)) {
        *position = imu;
        return;
    }
    data_.insert(position, imu);
}

std::optional<ov_core::ImuData> ImuBuffer::sampleAt(double timestamp) const {
    if (data_.empty() || timestamp < data_.front().timestamp - kTimestampEpsilon ||
        timestamp > data_.back().timestamp + kTimestampEpsilon) {
        return std::nullopt;
    }

    const auto next = std::lower_bound(
        data_.begin(), data_.end(), timestamp,
        [](const ov_core::ImuData& sample, double value) {
            return sample.timestamp < value;
        });
    if (next != data_.end() && sameTimestamp(next->timestamp, timestamp)) {
        ov_core::ImuData exact = *next;
        exact.timestamp = timestamp;
        return exact;
    }
    if (next == data_.begin() || next == data_.end()) return std::nullopt;

    const auto previous = std::prev(next);
    const double interval = next->timestamp - previous->timestamp;
    if (interval <= kTimestampEpsilon) return std::nullopt;
    const double alpha = (timestamp - previous->timestamp) / interval;
    ov_core::ImuData interpolated;
    interpolated.timestamp = timestamp;
    interpolated.wm = (1.0 - alpha) * previous->wm + alpha * next->wm;
    interpolated.am = (1.0 - alpha) * previous->am + alpha * next->am;
    return interpolated;
}

bool ImuBuffer::covers(double start_time, double end_time) const {
    if (end_time < start_time - kTimestampEpsilon || data_.empty()) return false;
    return sampleAt(start_time).has_value() && sampleAt(end_time).has_value();
}

std::optional<std::vector<ov_core::ImuData>> ImuBuffer::integrationSegment(
    double start_time, double end_time) const {
    if (!covers(start_time, end_time)) return std::nullopt;

    std::vector<ov_core::ImuData> segment;
    segment.push_back(*sampleAt(start_time));
    for (const auto& sample : data_) {
        if (sample.timestamp > start_time + kTimestampEpsilon &&
            sample.timestamp < end_time - kTimestampEpsilon) {
            segment.push_back(sample);
        }
    }
    if (!sameTimestamp(start_time, end_time)) {
        segment.push_back(*sampleAt(end_time));
    }
    return segment;
}

std::vector<ov_core::ImuData> ImuBuffer::samplesAfter(double timestamp) const {
    const auto first = std::upper_bound(
        data_.begin(), data_.end(), timestamp,
        [](double value, const ov_core::ImuData& sample) {
            return value < sample.timestamp;
        });
    return std::vector<ov_core::ImuData>(first, data_.end());
}

void ImuBuffer::discardBefore(double timestamp, bool keep_boundary) {
    const auto boundary = keep_boundary ? sampleAt(timestamp) : std::nullopt;
    data_.erase(std::remove_if(
                    data_.begin(), data_.end(),
                    [timestamp, keep_boundary](const ov_core::ImuData& imu) {
                        return keep_boundary
                            ? imu.timestamp < timestamp - kTimestampEpsilon
                            : imu.timestamp <= timestamp + kTimestampEpsilon;
                    }),
                data_.end());
    if (boundary) insert(*boundary);
}

double ImuBuffer::latestTime() const {
    return data_.empty() ? -std::numeric_limits<double>::infinity()
                         : data_.back().timestamp;
}

} // namespace hno_vio::pipeline
