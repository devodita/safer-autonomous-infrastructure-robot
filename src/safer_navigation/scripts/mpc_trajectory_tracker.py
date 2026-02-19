#!/usr/bin/env python3
"""
mpc_trajectory_tracker.py
---------------------------
A simplified Model Predictive Controller that tracks a reference trajectory
while respecting mode-dependent speed limits.

In a full deployment this would be backed by a proper QP solver (e.g. OSQP).
This implementation uses a receding-horizon proportional tracker as a
structurally correct placeholder that can be swapped for a full MPC solver.

Subscribes:
  /safer/mode_status          (safer_msgs/ModeStatus)
  /move_base/current_goal     (geometry_msgs/PoseStamped)
  /odom                       (nav_msgs/Odometry)

Publishes:
  /cmd_vel                    (geometry_msgs/Twist)
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from safer_msgs.msg import ModeStatus

MODE_NORMAL     = 0
MODE_CAUTION    = 1
MODE_RESTRICTED = 2
MODE_ESTOP      = 3

MODE_MAX_SPEED = {
    MODE_NORMAL:     1.5,
    MODE_CAUTION:    0.8,
    MODE_RESTRICTED: 0.3,
    MODE_ESTOP:      0.0,
}


class MPCTrajectoryTracker:
    def __init__(self):
        rospy.init_node("mpc_trajectory_tracker", anonymous=False)

        # ── MPC params ────────────────────────────────────────────────────────
        self.horizon       = rospy.get_param("~horizon",       10)
        self.dt            = rospy.get_param("~dt",            0.1)    # seconds per step
        self.k_linear      = rospy.get_param("~k_linear",      0.8)    # proportional gain
        self.k_angular     = rospy.get_param("~k_angular",     1.5)
        self.goal_tolerance= rospy.get_param("~goal_tolerance", 0.2)   # metres

        # ── State ─────────────────────────────────────────────────────────────
        self.current_mode  = MODE_NORMAL
        self.robot_x       = 0.0
        self.robot_y       = 0.0
        self.robot_yaw     = 0.0
        self.goal          = None   # geometry_msgs/PoseStamped

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/safer/mode_status",      ModeStatus,   self._mode_cb)
        rospy.Subscriber("/move_base/current_goal", PoseStamped,  self._goal_cb)
        rospy.Subscriber("/odom",                   Odometry,     self._odom_cb)

        control_rate = rospy.get_param("~control_rate", 10.0)  # Hz
        rospy.Timer(rospy.Duration(1.0 / control_rate), self._control_cb)

        rospy.loginfo("[MPCTracker] Initialised.")

    def _mode_cb(self, msg: ModeStatus):
        self.current_mode = msg.mode

    def _goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def _odom_cb(self, msg: Odometry):
        self.robot_x   = msg.pose.pose.position.x
        self.robot_y   = msg.pose.pose.position.y
        q              = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny_cosp      = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp      = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _control_cb(self, _event):
        cmd = Twist()
        max_speed = MODE_MAX_SPEED.get(self.current_mode, 0.0)

        if self.current_mode == MODE_ESTOP or self.goal is None:
            self.cmd_pub.publish(cmd)  # Zero velocity
            return

        gx  = self.goal.pose.position.x
        gy  = self.goal.pose.position.y
        dx  = gx - self.robot_x
        dy  = gy - self.robot_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < self.goal_tolerance:
            self.cmd_pub.publish(cmd)  # At goal
            return

        # Bearing error
        desired_yaw   = math.atan2(dy, dx)
        heading_error = self._normalise_angle(desired_yaw - self.robot_yaw)

        # Proportional control (placeholder for full QP-MPC)
        linear_vel    = self.k_linear  * dist
        angular_vel   = self.k_angular * heading_error

        # Reduce forward speed if heading error is large
        linear_vel   *= max(0.0, 1.0 - abs(heading_error) / math.pi)

        # Clamp to mode speed limit
        linear_vel    = max(-max_speed, min(linear_vel,  max_speed))
        angular_vel   = max(-1.2,       min(angular_vel, 1.2))

        cmd.linear.x  = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MPCTrajectoryTracker()
        node.spin()
    except rospy.ROSInterruptException:
        pass
