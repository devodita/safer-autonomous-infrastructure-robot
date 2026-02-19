#!/usr/bin/env python3
"""
dashboard_publisher.py
------------------------
Sends periodic robot telemetry to a REST dashboard endpoint via HTTP POST.

Subscribes:
  /safer/risk_assessment  (safer_msgs/RiskAssessment)
  /safer/mode_status      (safer_msgs/ModeStatus)
  /odom                   (nav_msgs/Odometry)
"""

import rospy
import json
import threading

try:
    import requests
    _REQUESTS_AVAILABLE = True
except ImportError:
    _REQUESTS_AVAILABLE = False

from nav_msgs.msg import Odometry
from safer_msgs.msg import RiskAssessment, ModeStatus

MODE_NAMES = {0: "NORMAL", 1: "CAUTION", 2: "RESTRICTED", 3: "EMERGENCY_STOP"}


class DashboardPublisher:
    def __init__(self):
        rospy.init_node("dashboard_publisher", anonymous=False)

        # ── Params ────────────────────────────────────────────────────────────
        self.endpoint    = rospy.get_param("~dashboard_endpoint", "http://localhost:8080/api/telemetry")
        self.robot_id    = rospy.get_param("~robot_id",           "safer_robot_01")
        self.publish_rate= rospy.get_param("~publish_rate",       1.0)   # Hz
        self.timeout     = rospy.get_param("~http_timeout",       2.0)   # seconds
        self.sim_mode    = rospy.get_param("~sim_mode",           False)

        # ── State ─────────────────────────────────────────────────────────────
        self.latest_risk  = None
        self.latest_mode  = None
        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self._lock        = threading.Lock()

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/risk_assessment", RiskAssessment, self._risk_cb)
        rospy.Subscriber("/safer/mode_status",     ModeStatus,     self._mode_cb)
        rospy.Subscriber("/odom",                  Odometry,       self._odom_cb)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish_cb)
        rospy.loginfo(f"[DashboardPublisher] Posting to {self.endpoint} @ {self.publish_rate}Hz")

    def _risk_cb(self, msg: RiskAssessment):
        with self._lock:
            self.latest_risk = msg

    def _mode_cb(self, msg: ModeStatus):
        with self._lock:
            self.latest_mode = msg

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _publish_cb(self, _event):
        with self._lock:
            risk = self.latest_risk
            mode = self.latest_mode

        payload = {
            "robot_id":    self.robot_id,
            "timestamp":   rospy.Time.now().to_sec(),
            "position":    {"x": round(self.robot_x, 2), "y": round(self.robot_y, 2)},
            "risk_score":  round(float(risk.risk_score),  3) if risk else None,
            "mode":        MODE_NAMES.get(mode.mode, "UNKNOWN") if mode else "UNKNOWN",
            "estop":       bool(mode.estop_latched)            if mode else False,
            "hazard_count":int(risk.active_hazard_count)       if risk else 0,
        }

        if self.sim_mode or not _REQUESTS_AVAILABLE:
            rospy.loginfo(f"[DashboardPublisher] SIM POST → {json.dumps(payload)}")
            return

        try:
            resp = requests.post(self.endpoint, json=payload, timeout=self.timeout)
            if resp.status_code not in (200, 201, 204):
                rospy.logwarn(f"[DashboardPublisher] HTTP {resp.status_code} from dashboard.")
        except requests.exceptions.RequestException as e:
            rospy.logwarn_throttle(10.0, f"[DashboardPublisher] POST failed: {e}")

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DashboardPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass
