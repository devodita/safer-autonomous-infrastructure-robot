#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <deque>
#include <map>
#include <string>

namespace safer_perception {

struct UltrasonicReading {
    std::string frame_id;
    float range_m;
    ros::Time stamp;
    bool valid;
};

/**
 * Median-filter wrapper for multiple ultrasonic range sensors.
 * Maintains a sliding window per sensor frame_id and returns
 * the filtered range on request.
 */
class UltrasonicFilter {
public:
    explicit UltrasonicFilter(int window_size = 5,
                               float min_range_m = 0.02f,
                               float max_range_m = 4.0f);

    /** Feed a new raw reading into the filter. */
    void addReading(const sensor_msgs::RangeConstPtr& msg);

    /**
     * Return the filtered range for a given sensor frame.
     * Returns -1.0 if insufficient data.
     */
    float getFilteredRange(const std::string& frame_id) const;

    /** Return all current filtered readings. */
    std::vector<UltrasonicReading> getAllReadings() const;

    /** True if any sensor reads below obstacle_threshold_m. */
    bool obstacleDetected(float obstacle_threshold_m = 0.5f) const;

private:
    int window_size_;
    float min_range_m_;
    float max_range_m_;

    // Per-sensor circular buffer of valid readings
    std::map<std::string, std::deque<float>> buffers_;

    bool isValidReading(float range_m) const;
    float medianOf(std::deque<float> buf) const;
};

}  // namespace safer_perception
