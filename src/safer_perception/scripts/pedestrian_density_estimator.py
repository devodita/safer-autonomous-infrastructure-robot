#!/usr/bin/env python3
"""
pedestrian_density_estimator.py
---------------------------------
Estimates pedestrian density from an RGB image using a background
subtraction + contour detection heuristic, or an optional ONNX
people-detection model.

Subscribes:
  /camera/rgb/image_raw   (sensor_msgs/Image)

Publishes:
  /safer/pedestrian_count (std_msgs/UInt16)
  /safer/hazard_detections (safer_msgs/HazardDetection)  — only when density is high
"""

import rospy
import cv2
import numpy as np
from std_msgs.msg import UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from safer_msgs.msg import HazardDetection

HAZARD_CLASS_PEDESTRIAN_DENSE = 4  # From hazard_classes.yaml


class PedestrianDensityEstimator:
    def __init__(self):
        rospy.init_node("pedestrian_density_estimator", anonymous=False)

        self.bridge = CvBridge()

        # ── Params ────────────────────────────────────────────────────────────
        self.high_density_thresh = rospy.get_param("~high_density_threshold", 5)
        self.publish_rate        = rospy.get_param("~publish_rate", 2.0)   # Hz
        self.min_contour_area    = rospy.get_param("~min_contour_area", 800)

        # ── Background subtractor for motion-based pedestrian detection ────────
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=50, detectShadows=False)

        # ── State ─────────────────────────────────────────────────────────────
        self.latest_count = 0

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.count_pub  = rospy.Publisher("/safer/pedestrian_count", UInt16, queue_size=5)
        self.hazard_pub = rospy.Publisher("/safer/hazard_detections", HazardDetection, queue_size=5)

        rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_cb)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_cb)
        rospy.loginfo("[PedestrianDensityEstimator] Ready.")

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[PedestrianDensity] Image decode error: {e}")
            return

        self.latest_count = self._estimate_count(frame)

    def _estimate_count(self, frame: np.ndarray) -> int:
        """
        Background subtraction → morphological cleaning → contour count.
        Each contour above min_contour_area is treated as a potential pedestrian.
        """
        fg_mask = self.bg_subtractor.apply(frame)

        # Remove noise
        kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN,  kernel, iterations=2)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel, iterations=3)

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = sum(1 for c in contours if cv2.contourArea(c) >= self.min_contour_area)
        return count

    def _publish_cb(self, _event):
        self.count_pub.publish(UInt16(data=self.latest_count))

        if self.latest_count >= self.high_density_thresh:
            hazard = HazardDetection()
            hazard.header.stamp    = rospy.Time.now()
            hazard.header.frame_id = "base_link"
            hazard.hazard_class    = HAZARD_CLASS_PEDESTRIAN_DENSE
            hazard.hazard_label    = "PEDESTRIAN_HIGH_DENSITY"
            hazard.confidence      = min(1.0, self.latest_count / (self.high_density_thresh * 2))
            hazard.severity        = 2
            hazard.source_sensor   = 0  # depth_camera / rgb
            self.hazard_pub.publish(hazard)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PedestrianDensityEstimator()
        node.spin()
    except rospy.ROSInterruptException:
        pass
