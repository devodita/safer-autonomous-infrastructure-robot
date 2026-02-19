#!/usr/bin/env python3
"""
hazard_classifier.py
---------------------
Subscribes to raw RGB + depth image pairs and calls the ClassifyHazard
ROS service (or runs a local ONNX model) to refine hazard class labels
produced by the C++ detector.

Subscribes:
  /camera/rgb/image_raw        (sensor_msgs/Image)
  /safer/hazard_detections     (safer_msgs/HazardDetection)

Publishes:
  /safer/hazard_detections/classified  (safer_msgs/HazardDetection)

The node loads a lightweight ONNX model (MobileNetV2-based) for
on-device inference. Falls back to class ID pass-through if no model
is available.
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from safer_msgs.msg import HazardDetection

# Hazard class names — must match hazard_classes.yaml
HAZARD_LABELS = {
    0: "UNKNOWN",
    1: "OPEN_MANHOLE",
    2: "OBSTACLE_STATIC",
    3: "OBSTACLE_DYNAMIC",
    4: "PEDESTRIAN_HIGH_DENSITY",
    5: "WET_SURFACE",
    6: "CONSTRUCTION_ZONE",
}

INPUT_SIZE = (224, 224)


class HazardClassifier:
    def __init__(self):
        rospy.init_node("hazard_classifier", anonymous=False)

        self.bridge       = CvBridge()
        self.latest_rgb   = None
        self.model        = None

        # ── Params ────────────────────────────────────────────────────────────
        model_path        = rospy.get_param("~model_path", "")
        self.conf_thresh  = rospy.get_param("~confidence_threshold", 0.60)

        # ── Load ONNX model if available ──────────────────────────────────────
        if model_path:
            try:
                self.model = cv2.dnn.readNetFromONNX(model_path)
                rospy.loginfo(f"[HazardClassifier] Loaded ONNX model: {model_path}")
            except Exception as e:
                rospy.logwarn(f"[HazardClassifier] Could not load model: {e}. Pass-through mode.")
        else:
            rospy.logwarn("[HazardClassifier] No model_path set — pass-through mode.")

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.classified_pub = rospy.Publisher(
            "/safer/hazard_detections/classified", HazardDetection, queue_size=10)

        rospy.Subscriber("/camera/rgb/image_raw", Image, self._rgb_cb)
        rospy.Subscriber("/safer/hazard_detections", HazardDetection, self._hazard_cb)

        rospy.loginfo("[HazardClassifier] Ready.")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[HazardClassifier] RGB decode error: {e}")

    def _hazard_cb(self, msg: HazardDetection):
        if self.model is None or self.latest_rgb is None:
            # Pass-through: just republish with updated label
            msg.hazard_label = HAZARD_LABELS.get(msg.hazard_class, "UNKNOWN")
            self.classified_pub.publish(msg)
            return

        class_id, confidence = self._classify(self.latest_rgb)

        if confidence >= self.conf_thresh:
            msg.hazard_class  = class_id
            msg.hazard_label  = HAZARD_LABELS.get(class_id, "UNKNOWN")
            msg.confidence    = confidence

        self.classified_pub.publish(msg)

    # ── Inference ──────────────────────────────────────────────────────────────

    def _classify(self, image: np.ndarray):
        """Run forward pass through the ONNX model and return (class_id, confidence)."""
        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1.0 / 255.0,
            size=INPUT_SIZE,
            mean=(0.485, 0.456, 0.406),
            swapRB=True,
            crop=False,
        )
        self.model.setInput(blob)
        output = self.model.forward()             # Shape: (1, num_classes)
        scores = output[0]

        # Softmax
        exp_scores = np.exp(scores - np.max(scores))
        probs = exp_scores / exp_scores.sum()

        class_id   = int(np.argmax(probs))
        confidence = float(probs[class_id])
        return class_id, confidence

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = HazardClassifier()
        node.spin()
    except rospy.ROSInterruptException:
        pass
