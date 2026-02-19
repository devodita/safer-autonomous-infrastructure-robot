#include "safer_perception/ultrasonic_filter.h"
#include <algorithm>
#include <numeric>

namespace safer_perception {

UltrasonicFilter::UltrasonicFilter(int window_size, float min_range_m, float max_range_m)
    : window_size_(window_size), min_range_m_(min_range_m), max_range_m_(max_range_m) {}

void UltrasonicFilter::addReading(const sensor_msgs::RangeConstPtr& msg) {
    if (!isValidReading(msg->range)) return;

    auto& buf = buffers_[msg->header.frame_id];
    buf.push_back(msg->range);
    if (static_cast<int>(buf.size()) > window_size_) {
        buf.pop_front();
    }
}

float UltrasonicFilter::getFilteredRange(const std::string& frame_id) const {
    auto it = buffers_.find(frame_id);
    if (it == buffers_.end() || it->second.empty()) return -1.0f;
    return medianOf(it->second);
}

std::vector<UltrasonicReading> UltrasonicFilter::getAllReadings() const {
    std::vector<UltrasonicReading> readings;
    for (const auto& kv : buffers_) {
        UltrasonicReading r;
        r.frame_id = kv.first;
        r.range_m  = medianOf(kv.second);
        r.valid    = (r.range_m > 0.0f);
        r.stamp    = ros::Time::now();
        readings.push_back(r);
    }
    return readings;
}

bool UltrasonicFilter::obstacleDetected(float obstacle_threshold_m) const {
    for (const auto& kv : buffers_) {
        float filtered = medianOf(kv.second);
        if (filtered > 0.0f && filtered < obstacle_threshold_m) return true;
    }
    return false;
}

bool UltrasonicFilter::isValidReading(float range_m) const {
    return (range_m >= min_range_m_ && range_m <= max_range_m_);
}

float UltrasonicFilter::medianOf(std::deque<float> buf) const {
    if (buf.empty()) return -1.0f;
    std::vector<float> sorted(buf.begin(), buf.end());
    std::sort(sorted.begin(), sorted.end());
    size_t mid = sorted.size() / 2;
    return (sorted.size() % 2 == 0)
               ? (sorted[mid - 1] + sorted[mid]) / 2.0f
               : sorted[mid];
}

}  // namespace safer_perception
