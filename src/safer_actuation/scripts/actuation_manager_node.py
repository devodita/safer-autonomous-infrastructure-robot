#!/usr/bin/env python3
"""
actuation_manager_node.py
--------------------------
Central actuator dispatcher for the SAFER robot.

Subscribes to:
  /safer/actuator_command  (safer_msgs/ActuatorCommand)
  /safer/mode_status       (safer_msgs/ModeStatus)

Publishes to:
  /safer/actuation/motor_cmd   (geometry_msgs/Twist)
  /safer/actuation/brake_cmd   (std_msgs/Float32)
  /safer/actuation/alert_cmd   (std_msgs/UInt8)
  /safer/actuation/diagnostics (std_msgs/String)

The manager enforces safety interlocks:
  - EMERGENCY_STOP mode overrides all other commands.
  - Brake is automatically engaged when speed drops below threshold.
  - Alert peripherals (horn, lights) are tied to hazard severity level.
"""

import rospy
from std_msgs.msg import Float32, UInt8, String
from geometry_msgs.msg import Twist
from safer_msgs.msg import ActuatorCommand, ModeStatus

# Mode constants — must stay in sync with mode_fsm.py
MODE_NORMAL     = 0
MODE_CAUTION    = 1
MODE_RESTRICTED = 2
MODE_ESTOP      = 3


class ActuationManager:
    def __init__(self):
        rospy.init_node("actuation_manager_node", anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.max_speed_normal     = rospy.get_param("~max_speed_normal",     1.5)   # m/s
        self.max_speed_caution    = rospy.get_param("~max_speed_caution",    0.8)
        self.max_speed_restricted = rospy.get_param("~max_speed_restricted", 0.3)
        self.brake_threshold      = rospy.get_param("~brake_threshold",      0.05)  # m/s
        self.cmd_timeout          = rospy.get_param("~cmd_timeout",          0.5)   # seconds

        # ── State ─────────────────────────────────────────────────────────────
        self.current_mode     = MODE_NORMAL
        self.last_cmd_time    = rospy.Time.now()
        self.estop_active     = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.motor_pub  = rospy.Publisher("/safer/actuation/motor_cmd",   Twist,   queue_size=1)
        self.brake_pub  = rospy.Publisher("/safer/actuation/brake_cmd",   Float32, queue_size=1)
        self.alert_pub  = rospy.Publisher("/safer/actuation/alert_cmd",   UInt8,   queue_size=1)
        self.diag_pub   = rospy.Publisher("/safer/actuation/diagnostics", String,  queue_size=5)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/actuator_command", ActuatorCommand, self._on_actuator_command)
        rospy.Subscriber("/safer/mode_status",      ModeStatus,      self._on_mode_status)

        # ── Watchdog timer — cuts motors if commands stop arriving ─────────────
        rospy.Timer(rospy.Duration(0.1), self._watchdog_cb)

        rospy.loginfo("[ActuationManager] Initialised. Waiting for commands.")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_mode_status(self, msg: ModeStatus):
        self.current_mode  = msg.mode
        self.estop_active  = (msg.mode == MODE_ESTOP)

        if self.estop_active:
            rospy.logwarn("[ActuationManager] EMERGENCY STOP received — engaging brakes.")
            self._engage_full_brake()
            self._send_alert(severity=3)

    def _on_actuator_command(self, msg: ActuatorCommand):
        self.last_cmd_time = rospy.Time.now()

        if self.estop_active:
            # Reject all motion commands during E-STOP
            self._engage_full_brake()
            return

        speed_limit = self._get_speed_limit()
        linear_x    = self._clamp(msg.linear_velocity,  -speed_limit, speed_limit)
        angular_z   = self._clamp(msg.angular_velocity, -msg.max_angular_velocity, msg.max_angular_velocity)

        self._send_motor_cmd(linear_x, angular_z)
        self._send_brake_cmd(linear_x)
        self._send_alert(msg.alert_level)

        self._publish_diag(
            f"mode={self.current_mode} | lin={linear_x:.2f} | ang={angular_z:.2f} | alert={msg.alert_level}"
        )

    def _watchdog_cb(self, event):
        """If no command received within timeout, hold position (brake, zero vel)."""
        if self.estop_active:
            return
        elapsed = (rospy.Time.now() - self.last_cmd_time).to_sec()
        if elapsed > self.cmd_timeout:
            self._send_motor_cmd(0.0, 0.0)
            self._engage_full_brake()

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _get_speed_limit(self) -> float:
        limits = {
            MODE_NORMAL:     self.max_speed_normal,
            MODE_CAUTION:    self.max_speed_caution,
            MODE_RESTRICTED: self.max_speed_restricted,
            MODE_ESTOP:      0.0,
        }
        return limits.get(self.current_mode, self.max_speed_restricted)

    def _send_motor_cmd(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self.motor_pub.publish(twist)

    def _send_brake_cmd(self, linear_x: float):
        """
        Proportional brake: full brake at standstill, released proportionally
        as speed command increases.
        """
        brake_force = 1.0 if abs(linear_x) < self.brake_threshold else 0.0
        self.brake_pub.publish(Float32(data=brake_force))

    def _engage_full_brake(self):
        self._send_motor_cmd(0.0, 0.0)
        self.brake_pub.publish(Float32(data=1.0))

    def _send_alert(self, severity: int):
        """
        Alert levels (must stay in sync with actuation_params.yaml):
          0 — off
          1 — visual only (LEDs)
          2 — visual + audible (horn)
          3 — full emergency (strobe + continuous horn)
        """
        self.alert_pub.publish(UInt8(data=max(0, min(severity, 3))))

    def _publish_diag(self, message: str):
        self.diag_pub.publish(String(data=message))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ActuationManager()
        node.spin()
    except rospy.ROSInterruptException:
        pass
