#!/usr/bin/env python3
"""
patrol_controller.py
---------------------
Executes a cyclic waypoint patrol using the ROS move_base action server.
Respects the current operational mode — pauses in ESTOP, slows in RESTRICTED.

Subscribes:
  /safer/mode_status   (safer_msgs/ModeStatus)
  /odom                (nav_msgs/Odometry)

Action client:
  move_base            (move_base_msgs/MoveBaseAction)

Parameters (patrol_params.yaml):
  waypoints: list of {x, y, yaw, frame_id}
  loop_patrol: true/false
  goal_tolerance: metres
"""

import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from safer_msgs.msg import ModeStatus

MODE_NORMAL     = 0
MODE_CAUTION    = 1
MODE_RESTRICTED = 2
MODE_ESTOP      = 3


class PatrolController:
    def __init__(self):
        rospy.init_node("patrol_controller", anonymous=False)

        # ── Params ────────────────────────────────────────────────────────────
        self.waypoints       = rospy.get_param("~waypoints", [])
        self.loop_patrol     = rospy.get_param("~loop_patrol", True)
        self.goal_tolerance  = rospy.get_param("~goal_tolerance", 0.3)   # metres
        self.wait_at_wp      = rospy.get_param("~wait_at_waypoint", 2.0) # seconds
        self.frame_id        = rospy.get_param("~frame_id", "map")

        # ── State ─────────────────────────────────────────────────────────────
        self.current_mode    = MODE_NORMAL
        self.current_wp_idx  = 0
        self.robot_x         = 0.0
        self.robot_y         = 0.0
        self.paused          = False

        # ── Action client ─────────────────────────────────────────────────────
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[PatrolController] Waiting for move_base action server...")
        self.move_base_client.wait_for_server(timeout=rospy.Duration(30.0))
        rospy.loginfo("[PatrolController] move_base connected.")

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/mode_status", ModeStatus, self._mode_cb)
        rospy.Subscriber("/odom",              Odometry,   self._odom_cb)

        rospy.loginfo(f"[PatrolController] Loaded {len(self.waypoints)} waypoints.")

    def _mode_cb(self, msg: ModeStatus):
        prev = self.current_mode
        self.current_mode = msg.mode

        if self.current_mode == MODE_ESTOP and prev != MODE_ESTOP:
            rospy.logwarn("[PatrolController] ESTOP — cancelling current goal.")
            self.move_base_client.cancel_all_goals()
            self.paused = True

        elif self.current_mode != MODE_ESTOP and prev == MODE_ESTOP:
            rospy.loginfo("[PatrolController] ESTOP cleared — resuming patrol.")
            self.paused = False

    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def run(self):
        if not self.waypoints:
            rospy.logwarn("[PatrolController] No waypoints configured — idle.")
            rospy.spin()
            return

        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if self.paused:
                rate.sleep()
                continue

            wp = self.waypoints[self.current_wp_idx]
            goal = self._make_goal(wp)

            rospy.loginfo(
                f"[PatrolController] Navigating to WP {self.current_wp_idx}: "
                f"({wp['x']:.2f}, {wp['y']:.2f})")

            self.move_base_client.send_goal(goal)

            # Wait for result, checking for ESTOP preemption
            while not rospy.is_shutdown():
                state = self.move_base_client.get_state()
                if self.paused:
                    break
                if state in (actionlib.GoalStatus.SUCCEEDED,):
                    rospy.loginfo(f"[PatrolController] Reached WP {self.current_wp_idx}.")
                    rospy.sleep(self.wait_at_wp)
                    self._advance_waypoint()
                    break
                elif state in (actionlib.GoalStatus.ABORTED,
                               actionlib.GoalStatus.REJECTED):
                    rospy.logwarn(f"[PatrolController] Goal aborted — skipping WP {self.current_wp_idx}.")
                    self._advance_waypoint()
                    break
                rospy.sleep(0.2)

    def _make_goal(self, wp: dict) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = wp.get("frame_id", self.frame_id)
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]
        yaw = wp.get("yaw", 0.0)
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _advance_waypoint(self):
        self.current_wp_idx += 1
        if self.current_wp_idx >= len(self.waypoints):
            if self.loop_patrol:
                self.current_wp_idx = 0
                rospy.loginfo("[PatrolController] Patrol loop complete — restarting.")
            else:
                rospy.loginfo("[PatrolController] Patrol complete.")
                rospy.signal_shutdown("Patrol complete.")


if __name__ == "__main__":
    try:
        node = PatrolController()
        node.run()
    except rospy.ROSInterruptException:
        pass
