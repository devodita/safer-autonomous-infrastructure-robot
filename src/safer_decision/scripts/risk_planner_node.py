#!/usr/bin/env python3
"""
risk_planner_node.py
----------------------
Main ROS node for the safer_decision package.
Wires together RiskScorer + ModeFSM and publishes RiskAssessment + ModeStatus.

Subscribes:
  /safer/hazard_detections/classified  (safer_msgs/HazardDetection)
  /safer/pedestrian_count              (std_msgs/UInt16)
  /odom                                (nav_msgs/Odometry)
  /safer/mode_command                  (safer_msgs/ModeCommand)

Publishes:
  /safer/risk_assessment               (safer_msgs/RiskAssessment)
  /safer/mode_status                   (safer_msgs/ModeStatus)
  /safer/actuator_command              (safer_msgs/ActuatorCommand)
  /safer/alert_payload                 (safer_msgs/AlertPayload)
"""

import rospy
import yaml
import uuid
from std_msgs.msg import UInt16
from nav_msgs.msg import Odometry
from safer_msgs.msg import (HazardDetection, RiskAssessment,
                             ModeStatus, ModeCommand,
                             ActuatorCommand, AlertPayload)

# Import pure-logic modules from same package
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
from risk_scorer import RiskScorer, MODE_NORMAL, MODE_CAUTION, MODE_RESTRICTED, MODE_ESTOP
from mode_fsm import ModeFSM

# Speed limits per mode (m/s)
MODE_SPEED = {
    MODE_NORMAL:     1.5,
    MODE_CAUTION:    0.8,
    MODE_RESTRICTED: 0.3,
    MODE_ESTOP:      0.0,
}

# Alert levels per mode
MODE_ALERT = {
    MODE_NORMAL:     0,
    MODE_CAUTION:    1,
    MODE_RESTRICTED: 2,
    MODE_ESTOP:      3,
}


class RiskPlannerNode:
    def __init__(self):
        rospy.init_node("risk_planner_node", anonymous=False)

        # ── Load thresholds ───────────────────────────────────────────────────
        thresh_path = rospy.get_param(
            "~thresholds_file",
            os.path.join(os.path.dirname(__file__), "../config/risk_thresholds.yaml"))
        with open(thresh_path, "r") as f:
            thresholds = yaml.safe_load(f)

        hysteresis = rospy.get_param("~hysteresis_duration", 2.0)
        self.scorer = RiskScorer(thresholds)
        self.fsm    = ModeFSM(hysteresis_duration=hysteresis)

        # ── State ─────────────────────────────────────────────────────────────
        self.active_hazards      = []
        self.pedestrian_count    = 0
        self.current_linear_vel  = 0.0
        self.hazard_timeout      = rospy.get_param("~hazard_timeout", 2.0)  # seconds
        self.hazard_timestamps   = {}  # hazard_label → rospy.Time

        # ── Publishers ────────────────────────────────────────────────────────
        self.risk_pub     = rospy.Publisher("/safer/risk_assessment",  RiskAssessment,  queue_size=5)
        self.mode_pub     = rospy.Publisher("/safer/mode_status",      ModeStatus,      queue_size=5)
        self.actuator_pub = rospy.Publisher("/safer/actuator_command", ActuatorCommand, queue_size=5)
        self.alert_pub    = rospy.Publisher("/safer/alert_payload",    AlertPayload,    queue_size=5)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/hazard_detections/classified", HazardDetection, self._hazard_cb)
        rospy.Subscriber("/safer/pedestrian_count",             UInt16,          self._ped_cb)
        rospy.Subscriber("/odom",                               Odometry,        self._odom_cb)
        rospy.Subscriber("/safer/mode_command",                 ModeCommand,     self._mode_cmd_cb)

        # ── Timer ─────────────────────────────────────────────────────────────
        rate = rospy.get_param("~update_rate", 5.0)
        rospy.Timer(rospy.Duration(1.0 / rate), self._update_cb)

        rospy.loginfo("[RiskPlannerNode] Running.")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _hazard_cb(self, msg: HazardDetection):
        key = f"{msg.hazard_class}_{round(msg.position.x,1)}_{round(msg.position.y,1)}"
        self.hazard_timestamps[key] = rospy.Time.now()
        # Upsert hazard in active list
        for i, h in enumerate(self.active_hazards):
            if h.hazard_class == msg.hazard_class:
                self.active_hazards[i] = msg
                return
        self.active_hazards.append(msg)

    def _ped_cb(self, msg: UInt16):
        self.pedestrian_count = msg.data

    def _odom_cb(self, msg: Odometry):
        self.current_linear_vel = msg.twist.twist.linear.x

    def _mode_cmd_cb(self, msg: ModeCommand):
        rospy.loginfo(f"[RiskPlannerNode] Manual mode command: {msg.mode} from {msg.issuer}")
        if msg.mode == 99:  # Convention: mode 99 = reset ESTOP
            self.fsm.manual_reset()
        else:
            self.fsm.manual_command(msg.mode, issuer=msg.issuer)
        self._publish_mode_status()

    # ── Main update loop ───────────────────────────────────────────────────────

    def _update_cb(self, _event):
        self._expire_old_hazards()

        result = self.scorer.compute(
            self.active_hazards,
            self.current_linear_vel,
            self.pedestrian_count)

        transitioned = self.fsm.update(result["recommended_mode"])

        self._publish_risk_assessment(result)
        self._publish_mode_status()
        self._publish_actuator_command()

        if transitioned and self.fsm.current_mode >= MODE_RESTRICTED:
            self._publish_alert(result)

    # ── Publishers ─────────────────────────────────────────────────────────────

    def _publish_risk_assessment(self, result: dict):
        msg = RiskAssessment()
        msg.header.stamp        = rospy.Time.now()
        msg.risk_score          = result["risk_score"]
        msg.proximity_risk      = result["proximity_risk"]
        msg.density_risk        = result["density_risk"]
        msg.velocity_risk       = result["velocity_risk"]
        msg.severity_risk       = result["severity_risk"]
        msg.active_hazard_count = result["active_hazards"]
        msg.recommended_mode    = result["recommended_mode"]
        msg.reasoning           = result["reasoning"]
        self.risk_pub.publish(msg)

    def _publish_mode_status(self):
        s   = self.fsm.status_dict()
        msg = ModeStatus()
        msg.header.stamp    = rospy.Time.now()
        msg.mode            = s["mode"]
        msg.previous_mode   = s["previous_mode"]
        msg.time_in_mode    = s["time_in_mode"]
        msg.estop_latched   = s["estop_latched"]
        msg.status_message  = s["status_message"]
        self.mode_pub.publish(msg)

    def _publish_actuator_command(self):
        mode    = self.fsm.current_mode
        max_vel = MODE_SPEED[mode]
        alert   = MODE_ALERT[mode]

        cmd = ActuatorCommand()
        cmd.header.stamp       = rospy.Time.now()
        cmd.linear_velocity    = min(self.current_linear_vel, max_vel)
        cmd.angular_velocity   = 0.0  # Navigation layer handles angular
        cmd.max_angular_velocity = 1.2
        cmd.alert_level        = alert
        cmd.command_source     = 2   # decision_override
        self.actuator_pub.publish(cmd)

    def _publish_alert(self, result: dict):
        mode    = self.fsm.current_mode
        hazard  = self.active_hazards[0] if self.active_hazards else None
        msg     = AlertPayload()
        msg.header.stamp   = rospy.Time.now()
        msg.severity       = MODE_ALERT[mode]
        msg.hazard_class   = hazard.hazard_class if hazard else 0
        msg.hazard_label   = hazard.hazard_label if hazard else "UNKNOWN"
        msg.risk_score     = result["risk_score"]
        msg.mode           = mode
        msg.message        = result["reasoning"]
        msg.alert_id       = str(uuid.uuid4())[:8]
        self.alert_pub.publish(msg)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _expire_old_hazards(self):
        now = rospy.Time.now()
        fresh = []
        for h in self.active_hazards:
            key = f"{h.hazard_class}_{round(h.position.x,1)}_{round(h.position.y,1)}"
            ts  = self.hazard_timestamps.get(key, now)
            if (now - ts).to_sec() < self.hazard_timeout:
                fresh.append(h)
        self.active_hazards = fresh

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = RiskPlannerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
