#!/usr/bin/env python3
"""
mqtt_alert_bridge.py
---------------------
Bridges ROS AlertPayload messages to an MQTT broker for fleet management
and remote monitoring.

Subscribes:
  /safer/alert_payload   (safer_msgs/AlertPayload)
  /safer/mode_status     (safer_msgs/ModeStatus)

MQTT Topics Published:
  safer/{robot_id}/alerts
  safer/{robot_id}/mode
"""

import rospy
import json
import time

try:
    import paho.mqtt.client as mqtt
    _MQTT_AVAILABLE = True
except ImportError:
    _MQTT_AVAILABLE = False

from safer_msgs.msg import AlertPayload, ModeStatus

MODE_NAMES = {0: "NORMAL", 1: "CAUTION", 2: "RESTRICTED", 3: "EMERGENCY_STOP"}


class MQTTAlertBridge:
    def __init__(self):
        rospy.init_node("mqtt_alert_bridge", anonymous=False)

        # ── Params ────────────────────────────────────────────────────────────
        self.broker_host  = rospy.get_param("~broker_host",  "localhost")
        self.broker_port  = rospy.get_param("~broker_port",  1883)
        self.robot_id     = rospy.get_param("~robot_id",     "safer_robot_01")
        self.keepalive    = rospy.get_param("~keepalive",    60)
        self.qos          = rospy.get_param("~qos",          1)
        self.sim_mode     = rospy.get_param("~sim_mode",     False)

        self.alert_topic  = f"safer/{self.robot_id}/alerts"
        self.mode_topic   = f"safer/{self.robot_id}/mode"

        # ── MQTT client ───────────────────────────────────────────────────────
        self.client = None
        if _MQTT_AVAILABLE and not self.sim_mode:
            self.client = mqtt.Client(client_id=self.robot_id)
            self.client.on_connect    = self._on_mqtt_connect
            self.client.on_disconnect = self._on_mqtt_disconnect
            try:
                self.client.connect(self.broker_host, self.broker_port, self.keepalive)
                self.client.loop_start()
                rospy.loginfo(f"[MQTTBridge] Connecting to {self.broker_host}:{self.broker_port}")
            except Exception as e:
                rospy.logerr(f"[MQTTBridge] Connection failed: {e}")
        else:
            rospy.logwarn("[MQTTBridge] paho-mqtt not installed or sim_mode — logging only.")

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/alert_payload", AlertPayload, self._on_alert)
        rospy.Subscriber("/safer/mode_status",   ModeStatus,   self._on_mode)

        rospy.loginfo("[MQTTBridge] Ready.")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            rospy.loginfo("[MQTTBridge] MQTT connected.")
        else:
            rospy.logerr(f"[MQTTBridge] MQTT connection refused (rc={rc}).")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        rospy.logwarn(f"[MQTTBridge] MQTT disconnected (rc={rc}). Will retry.")

    def _on_alert(self, msg: AlertPayload):
        payload = {
            "timestamp":    msg.header.stamp.to_sec(),
            "alert_id":     msg.alert_id,
            "severity":     int(msg.severity),
            "hazard_class": int(msg.hazard_class),
            "hazard_label": msg.hazard_label,
            "risk_score":   round(float(msg.risk_score), 3),
            "mode":         MODE_NAMES.get(msg.mode, "UNKNOWN"),
            "position":     {"x": round(msg.robot_position.x, 2),
                             "y": round(msg.robot_position.y, 2)},
            "message":      msg.message,
        }
        self._publish(self.alert_topic, payload)

    def _on_mode(self, msg: ModeStatus):
        payload = {
            "timestamp":    msg.header.stamp.to_sec(),
            "mode":         MODE_NAMES.get(msg.mode, "UNKNOWN"),
            "mode_id":      int(msg.mode),
            "time_in_mode": round(float(msg.time_in_mode), 1),
            "estop_latched": bool(msg.estop_latched),
        }
        self._publish(self.mode_topic, payload)

    def _publish(self, topic: str, payload: dict):
        json_str = json.dumps(payload)
        if self.client is not None:
            result = self.client.publish(topic, json_str, qos=self.qos)
            if result.rc != 0:
                rospy.logwarn(f"[MQTTBridge] Publish failed on {topic}")
        else:
            rospy.loginfo(f"[MQTTBridge] SIM publish → {topic}: {json_str}")

    def shutdown(self):
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MQTTAlertBridge()
        rospy.on_shutdown(node.shutdown)
        node.spin()
    except rospy.ROSInterruptException:
        pass
