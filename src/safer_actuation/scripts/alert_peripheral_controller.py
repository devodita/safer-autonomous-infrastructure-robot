#!/usr/bin/env python3
"""
alert_peripheral_controller.py
--------------------------------
Controls visual and audible alert peripherals based on hazard severity.

Subscribes to:
  /safer/actuation/alert_cmd  (std_msgs/UInt8)

Alert level semantics (keep in sync with actuation_manager_node.py and actuation_params.yaml):
  0 — all off
  1 — steady amber LED
  2 — flashing amber LED + short horn beep
  3 — strobe red LED + continuous horn (emergency)

Hardware:
  - LEDs driven via GPIO (PWM for flash/strobe)
  - Horn driven via GPIO relay
  In sim_mode, all output is logged only.
"""

import rospy
import threading
from std_msgs.msg import UInt8

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class AlertPeripheralController:
    def __init__(self):
        rospy.init_node("alert_peripheral_controller", anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.led_gpio_pin  = rospy.get_param("~led_gpio_pin",  23)
        self.horn_gpio_pin = rospy.get_param("~horn_gpio_pin", 24)
        self.sim_mode      = rospy.get_param("~sim_mode",      False)

        # ── GPIO setup ────────────────────────────────────────────────────────
        if _GPIO_AVAILABLE and not self.sim_mode:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.led_gpio_pin,  GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.horn_gpio_pin, GPIO.OUT, initial=GPIO.LOW)
            self.led_pwm = GPIO.PWM(self.led_gpio_pin, 2)   # 2 Hz flash
            self.led_pwm.start(0)

        # ── State / threading ─────────────────────────────────────────────────
        self.current_level = 0
        self._lock         = threading.Lock()

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/actuation/alert_cmd", UInt8, self._on_alert_cmd)
        rospy.loginfo("[AlertPeripheralController] Ready.")

    def _on_alert_cmd(self, msg: UInt8):
        level = max(0, min(int(msg.data), 3))
        with self._lock:
            if level == self.current_level:
                return
            self.current_level = level
            self._apply_alert(level)

    def _apply_alert(self, level: int):
        if self.sim_mode or not _GPIO_AVAILABLE:
            labels = {0: "ALL OFF", 1: "AMBER LED", 2: "FLASHING + BEEP", 3: "STROBE + HORN"}
            rospy.loginfo(f"[AlertPeripheralController] SIM — {labels.get(level, 'UNKNOWN')}")
            return

        # Clear state first
        self.led_pwm.ChangeDutyCycle(0)
        GPIO.output(self.horn_gpio_pin, GPIO.LOW)

        if level == 0:
            pass  # everything already off
        elif level == 1:
            self.led_pwm.ChangeDutyCycle(100)   # Steady on
        elif level == 2:
            self.led_pwm.ChangeDutyCycle(50)    # 50% duty = flash
            GPIO.output(self.horn_gpio_pin, GPIO.HIGH)
            # Beep for 0.3 s then off
            threading.Timer(0.3, lambda: GPIO.output(self.horn_gpio_pin, GPIO.LOW)).start()
        elif level == 3:
            self.led_pwm.ChangeFrequency(8)     # Fast strobe
            self.led_pwm.ChangeDutyCycle(50)
            GPIO.output(self.horn_gpio_pin, GPIO.HIGH)  # Continuous horn

    def shutdown(self):
        rospy.loginfo("[AlertPeripheralController] Shutdown — clearing all alerts.")
        self._apply_alert(0)
        if _GPIO_AVAILABLE and not self.sim_mode:
            self.led_pwm.stop()
            GPIO.cleanup()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = AlertPeripheralController()
        rospy.on_shutdown(node.shutdown)
        node.spin()
    except rospy.ROSInterruptException:
        pass
