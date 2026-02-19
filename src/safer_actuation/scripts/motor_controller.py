#!/usr/bin/env python3
"""
motor_controller.py
--------------------
Translates ROS Twist commands into low-level motor driver instructions.

Subscribes to:
  /safer/actuation/motor_cmd  (geometry_msgs/Twist)

Publishes to:
  /safer/actuation/motor_feedback  (std_msgs/String)  — encoder / RPM telemetry

Hardware interface:
  Writes PWM duty-cycle bytes over serial to the STM32 motor bridge
  (see firmware/stm32/main.c). Baud rate and port are ROS params.

Differential drive model:
  left_vel  = linear_x - (angular_z * wheel_base / 2)
  right_vel = linear_x + (angular_z * wheel_base / 2)
"""

import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MotorController:
    # Serial frame format: [0xAA, left_pwm_byte, right_pwm_byte, checksum, 0x55]
    FRAME_START = 0xAA
    FRAME_END   = 0x55
    MAX_PWM     = 255

    def __init__(self):
        rospy.init_node("motor_controller", anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.wheel_base   = rospy.get_param("~wheel_base",   0.45)    # metres
        self.max_speed    = rospy.get_param("~max_speed",    1.5)     # m/s
        self.serial_port  = rospy.get_param("~serial_port",  "/dev/ttyUSB0")
        self.baud_rate    = rospy.get_param("~baud_rate",    115200)
        self.sim_mode     = rospy.get_param("~sim_mode",     False)

        # ── Serial connection ─────────────────────────────────────────────────
        self.serial_conn = None
        if not self.sim_mode:
            try:
                self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                rospy.loginfo(f"[MotorController] Serial connected: {self.serial_port} @ {self.baud_rate}")
            except serial.SerialException as e:
                rospy.logerr(f"[MotorController] Serial open failed: {e}. Running in degraded mode.")
        else:
            rospy.logwarn("[MotorController] sim_mode=True — no serial output.")

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.feedback_pub = rospy.Publisher("/safer/actuation/motor_feedback", String, queue_size=5)
        rospy.Subscriber("/safer/actuation/motor_cmd", Twist, self._on_motor_cmd)

        rospy.loginfo("[MotorController] Ready.")

    def _on_motor_cmd(self, msg: Twist):
        left_vel, right_vel = self._diff_drive(msg.linear.x, msg.angular.z)
        left_pwm  = self._vel_to_pwm(left_vel)
        right_pwm = self._vel_to_pwm(right_vel)

        self._send_serial_frame(left_pwm, right_pwm)
        self.feedback_pub.publish(
            String(data=f"left_pwm={left_pwm} right_pwm={right_pwm}")
        )

    def _diff_drive(self, linear_x: float, angular_z: float):
        half_base = self.wheel_base / 2.0
        left_vel  = linear_x - (angular_z * half_base)
        right_vel = linear_x + (angular_z * half_base)
        return left_vel, right_vel

    def _vel_to_pwm(self, velocity: float) -> int:
        """Map [-max_speed, +max_speed] → [0, 255] with 127 as stop."""
        normalised = velocity / self.max_speed
        normalised = max(-1.0, min(1.0, normalised))
        return int((normalised + 1.0) / 2.0 * self.MAX_PWM)

    def _send_serial_frame(self, left_pwm: int, right_pwm: int):
        if self.serial_conn is None:
            return
        checksum = (left_pwm + right_pwm) & 0xFF
        frame = struct.pack("BBBBB",
                            self.FRAME_START,
                            left_pwm, right_pwm,
                            checksum,
                            self.FRAME_END)
        try:
            self.serial_conn.write(frame)
        except serial.SerialException as e:
            rospy.logerr(f"[MotorController] Serial write error: {e}")

    def spin(self):
        rospy.spin()

    def shutdown(self):
        if self.serial_conn and self.serial_conn.is_open:
            # Zero out motors on shutdown
            self._send_serial_frame(127, 127)
            self.serial_conn.close()


if __name__ == "__main__":
    try:
        node = MotorController()
        rospy.on_shutdown(node.shutdown)
        node.spin()
    except rospy.ROSInterruptException:
        pass
