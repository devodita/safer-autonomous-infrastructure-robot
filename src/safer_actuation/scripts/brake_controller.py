#!/usr/bin/env python3
"""
brake_controller.py
--------------------
Manages the electromagnetic / servo braking system.

Subscribes to:
  /safer/actuation/brake_cmd  (std_msgs/Float32)  — 0.0 = released, 1.0 = full brake

Publishes to:
  /safer/actuation/brake_status  (std_msgs/Float32)  — confirmed applied force

The brake engages via a GPIO pin on the onboard SBC (Raspberry Pi / Jetson).
In sim_mode the GPIO calls are replaced with log statements.
"""

import rospy
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class BrakeController:
    def __init__(self):
        rospy.init_node("brake_controller", anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.brake_gpio_pin  = rospy.get_param("~brake_gpio_pin",  18)
        self.engage_high     = rospy.get_param("~engage_high",     True)   # HIGH = brake ON
        self.sim_mode        = rospy.get_param("~sim_mode",        False)

        # ── GPIO setup ────────────────────────────────────────────────────────
        if _GPIO_AVAILABLE and not self.sim_mode:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.brake_gpio_pin, GPIO.OUT)
            GPIO.output(self.brake_gpio_pin, GPIO.HIGH if self.engage_high else GPIO.LOW)
            rospy.loginfo(f"[BrakeController] GPIO pin {self.brake_gpio_pin} initialised (brake engaged).")
        else:
            rospy.logwarn("[BrakeController] GPIO not available or sim_mode — brake in simulation.")

        # ── State ─────────────────────────────────────────────────────────────
        self.current_brake_force = 1.0  # Start with brake engaged

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.status_pub = rospy.Publisher("/safer/actuation/brake_status", Float32, queue_size=1)
        rospy.Subscriber("/safer/actuation/brake_cmd", Float32, self._on_brake_cmd)

        rospy.loginfo("[BrakeController] Ready — brake engaged at startup.")

    def _on_brake_cmd(self, msg: Float32):
        force = max(0.0, min(1.0, msg.data))
        self._apply_brake(force)
        self.current_brake_force = force
        self.status_pub.publish(Float32(data=force))

    def _apply_brake(self, force: float):
        """
        Simple binary brake: any force > 0.5 = fully engaged.
        A future revision can drive a servo for proportional braking.
        """
        engage = force > 0.5
        if _GPIO_AVAILABLE and not rospy.get_param("~sim_mode", False):
            pin_state = GPIO.HIGH if (engage == self.engage_high) else GPIO.LOW
            GPIO.output(self.brake_gpio_pin, pin_state)
        else:
            state_str = "ENGAGED" if engage else "RELEASED"
            rospy.logdebug(f"[BrakeController] SIM — brake {state_str} (force={force:.2f})")

    def shutdown(self):
        rospy.loginfo("[BrakeController] Shutdown — engaging brake.")
        self._apply_brake(1.0)
        if _GPIO_AVAILABLE:
            GPIO.cleanup()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = BrakeController()
        rospy.on_shutdown(node.shutdown)
        node.spin()
    except rospy.ROSInterruptException:
        pass
