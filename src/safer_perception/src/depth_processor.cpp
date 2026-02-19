#include "safer_perception/depth_processor.h"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <algorithm>

namespace safer_perception {

DepthProcessor::DepthProcessor(ros::NodeHandle& nh) : nh_(nh) {
    loadParams();
}

void DepthProcessor::loadParams() {
    nh_.param("camera/fx", fx_, 525.0);
    nh_.param("camera/fy", fy_, 525.0);
    nh_.param("camera/cx", cx_, 319.5);
    nh_.param("camera/cy", cy_, 239.5);
    nh_.param("depth/drop_threshold",      depth_drop_threshold_,   0.25f);
    nh_.param("depth/min_anomaly_width_px", min_anomaly_width_px_,  20.0f);
    nh_.param("depth/max_anomaly_width_px", max_anomaly_width_px_, 200.0f);
}

std::vector<DepthAnomaly> DepthProcessor::processDepthImage(
    const sensor_msgs::ImageConstPtr& depth_msg)
{
    cv::Mat depth_m = decodeDepthImage(depth_msg);
    if (depth_m.empty()) {
        ROS_WARN_THROTTLE(5.0, "[DepthProcessor] Empty depth image received.");
        return {};
    }
    return detectDrops(depth_m);
}

cv::Mat DepthProcessor::decodeDepthImage(const sensor_msgs::ImageConstPtr& msg) const {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // Depth images are typically 16UC1 (millimetres) or 32FC1 (metres)
        if (msg->encoding == "16UC1") {
            cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
            cv::Mat depth_f;
            cv_ptr->image.convertTo(depth_f, CV_32F, 0.001f);  // mm → m
            return depth_f;
        } else {
            cv_ptr = cv_bridge::toCvShare(msg, "32FC1");
            return cv_ptr->image.clone();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("[DepthProcessor] cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

std::vector<DepthAnomaly> DepthProcessor::detectDrops(const cv::Mat& depth_m) const {
    std::vector<DepthAnomaly> anomalies;

    // Scan horizontal rows in the lower half of the image (ground plane region)
    int row_start = depth_m.rows / 2;
    for (int r = row_start; r < depth_m.rows; ++r) {
        const float* row_ptr = depth_m.ptr<float>(r);
        int anomaly_start = -1;
        float ref_depth = 0.0f;

        for (int c = 1; c < depth_m.cols; ++c) {
            float prev = row_ptr[c - 1];
            float curr = row_ptr[c];

            // Skip NaN / invalid pixels
            if (!std::isfinite(prev) || !std::isfinite(curr)) continue;

            float drop = curr - prev;  // Positive = surface drops away (pit / hole)

            if (drop > depth_drop_threshold_ && anomaly_start < 0) {
                anomaly_start = c;
                ref_depth     = prev;
            } else if (anomaly_start >= 0 && (drop < -depth_drop_threshold_ || c == depth_m.cols - 1)) {
                float width_px = static_cast<float>(c - anomaly_start);
                if (width_px >= min_anomaly_width_px_ && width_px <= max_anomaly_width_px_) {
                    int mid_u = anomaly_start + static_cast<int>(width_px / 2);
                    DepthAnomaly a;
                    a.position   = pixelToCamera(mid_u, r, ref_depth);
                    a.width      = width_px * ref_depth / static_cast<float>(fx_);  // metres
                    a.depth_drop = curr - ref_depth;
                    a.confidence = std::min(1.0f, (drop - depth_drop_threshold_) / 0.5f);
                    anomalies.push_back(a);
                }
                anomaly_start = -1;
            }
        }
    }
    return anomalies;
}

geometry_msgs::Point DepthProcessor::pixelToCamera(int u, int v, float depth_m) const {
    geometry_msgs::Point p;
    p.z = depth_m;
    p.x = (u - cx_) * depth_m / fx_;
    p.y = (v - cy_) * depth_m / fy_;
    return p;
}

}  // namespace safer_perception
