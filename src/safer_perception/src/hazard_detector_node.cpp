#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "safer_perception/depth_processor.h"
#include "safer_perception/ultrasonic_filter.h"
#include "safer_msgs/HazardDetection.h"

using namespace safer_perception;

class HazardDetectorNode {
public:
    HazardDetectorNode() : nh_("~"), tf_listener_(tf_buffer_) {
        depth_processor_ = std::make_unique<DepthProcessor>(nh_);
        ultrasonic_filter_ = std::make_unique<UltrasonicFilter>(
            nh_.param("ultrasonic/window_size", 5),
            nh_.param("ultrasonic/min_range", 0.02f),
            nh_.param("ultrasonic/max_range", 4.0f));

        hazard_pub_ = nh_.advertise<safer_msgs::HazardDetection>(
            "/safer/hazard_detections", 10);

        depth_sub_ = nh_.subscribe("/camera/depth/image_rect_raw", 1,
                                   &HazardDetectorNode::depthCallback, this);

        // Subscribe to multiple ultrasonic sensors
        for (const auto& topic : {"/ultrasonic/front_left",
                                   "/ultrasonic/front_right",
                                   "/ultrasonic/rear"}) {
            ultrasonic_subs_.push_back(
                nh_.subscribe<sensor_msgs::Range>(
                    topic, 10,
                    [this](const sensor_msgs::RangeConstPtr& msg) {
                        ultrasonic_filter_->addReading(msg);
                        checkUltrasonicHazards();
                    }));
        }

        publish_rate_   = nh_.param("publish_rate", 10.0);
        base_frame_     = nh_.param<std::string>("base_frame", "base_link");
        camera_frame_   = nh_.param<std::string>("camera_frame", "camera_depth_frame");
        obstacle_thresh_ = nh_.param("ultrasonic/obstacle_threshold", 0.5f);

        publish_timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &HazardDetectorNode::timerCallback, this);

        ROS_INFO("[HazardDetector] Initialised. Publishing on /safer/hazard_detections");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher  hazard_pub_;
    ros::Subscriber depth_sub_;
    std::vector<ros::Subscriber> ultrasonic_subs_;
    ros::Timer      publish_timer_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unique_ptr<DepthProcessor>    depth_processor_;
    std::unique_ptr<UltrasonicFilter>  ultrasonic_filter_;

    double      publish_rate_;
    std::string base_frame_;
    std::string camera_frame_;
    float       obstacle_thresh_;

    // Latest pending anomalies from depth camera (published on timer)
    std::vector<DepthAnomaly> pending_depth_anomalies_;

    void depthCallback(const sensor_msgs::ImageConstPtr& msg) {
        pending_depth_anomalies_ = depth_processor_->processDepthImage(msg);
    }

    void timerCallback(const ros::TimerEvent&) {
        for (const auto& anomaly : pending_depth_anomalies_) {
            publishDepthHazard(anomaly);
        }
        pending_depth_anomalies_.clear();
    }

    void checkUltrasonicHazards() {
        if (!ultrasonic_filter_->obstacleDetected(obstacle_thresh_)) return;

        for (const auto& reading : ultrasonic_filter_->getAllReadings()) {
            if (!reading.valid || reading.range_m >= obstacle_thresh_) continue;

            safer_msgs::HazardDetection msg;
            msg.header.stamp    = ros::Time::now();
            msg.header.frame_id = base_frame_;
            msg.hazard_class    = 2;  // OBSTACLE_STATIC
            msg.hazard_label    = "OBSTACLE_STATIC";
            msg.confidence      = 0.85f;
            msg.position.x      = reading.range_m;
            msg.position.y      = 0.0;
            msg.position.z      = 0.0;
            msg.width           = 0.3f;
            msg.source_sensor   = 1;  // ultrasonic
            msg.severity        = 2;
            hazard_pub_.publish(msg);
        }
    }

    void publishDepthHazard(const DepthAnomaly& anomaly) {
        // Transform anomaly position from camera frame to base_link
        geometry_msgs::Point pos_base;
        try {
            geometry_msgs::PointStamped cam_point, base_point;
            cam_point.header.frame_id = camera_frame_;
            cam_point.header.stamp    = ros::Time(0);
            cam_point.point           = anomaly.position;
            tf_buffer_.transform(cam_point, base_point, base_frame_);
            pos_base = base_point.point;
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[HazardDetector] TF lookup failed: %s", ex.what());
            pos_base = anomaly.position;  // Fallback: publish in camera frame
        }

        safer_msgs::HazardDetection msg;
        msg.header.stamp    = ros::Time::now();
        msg.header.frame_id = base_frame_;
        msg.hazard_class    = 1;  // OPEN_MANHOLE
        msg.hazard_label    = "OPEN_MANHOLE";
        msg.confidence      = anomaly.confidence;
        msg.position        = pos_base;
        msg.width           = anomaly.width;
        msg.depth           = anomaly.depth_drop;
        msg.source_sensor   = 0;  // depth_camera
        msg.severity        = 3;
        hazard_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hazard_detector_node");
    HazardDetectorNode node;
    ros::spin();
    return 0;
}
