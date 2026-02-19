#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace safer_perception {

struct DepthAnomaly {
    geometry_msgs::Point position;  // In camera frame
    float width;
    float depth_drop;               // Metres of sudden drop
    float confidence;
};

class DepthProcessor {
public:
    explicit DepthProcessor(ros::NodeHandle& nh);

    /**
     * Process a depth image and return any detected anomalies
     * (sudden depth discontinuities indicative of open manholes / pits).
     */
    std::vector<DepthAnomaly> processDepthImage(const sensor_msgs::ImageConstPtr& depth_msg);

    /**
     * Convert a depth image to a 3-D point in the camera frame given
     * pixel coordinates.
     */
    geometry_msgs::Point pixelToCamera(int u, int v, float depth_m) const;

private:
    ros::NodeHandle& nh_;

    // Camera intrinsics (loaded from param server)
    double fx_, fy_, cx_, cy_;

    // Thresholds
    float depth_drop_threshold_;    // metres — minimum drop to flag as anomaly
    float min_anomaly_width_px_;    // pixels
    float max_anomaly_width_px_;

    void loadParams();
    cv::Mat decodeDepthImage(const sensor_msgs::ImageConstPtr& msg) const;
    std::vector<DepthAnomaly> detectDrops(const cv::Mat& depth_m) const;
};

}  // namespace safer_perception
