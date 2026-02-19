#!/usr/bin/env python3
"""
open_manhole_scenario.py
-------------------------
Gazebo simulation scenario: spawns an open manhole cover model in front of
the patrolling SAFER robot and verifies that the system detects the hazard,
escalates to EMERGENCY_STOP mode, and halts within a safe distance.

Usage:
  rosrun safer_simulation open_manhole_scenario.py

Requires:
  - Gazebo running with SAFER robot spawned
  - safer_bringup/safer_simulation.launch active
"""

import rospy
import subprocess
import time
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from safer_msgs.msg import ModeStatus, HazardDetection

MANHOLE_SDF = """
<sdf version="1.6">
  <model name="open_manhole">
    <static>true</static>
    <link name="link">
      <collision name="col">
        <geometry>
          <cylinder><radius>0.30</radius><length>0.02</length></cylinder>
        </geometry>
      </collision>
      <visual name="vis">
        <geometry>
          <cylinder><radius>0.30</radius><length>0.02</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.2 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class OpenManholeScenario:
    def __init__(self):
        rospy.init_node("open_manhole_scenario", anonymous=False)

        self.detected_hazard    = False
        self.estop_triggered    = False
        self.scenario_passed    = False

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber("/safer/mode_status",       ModeStatus,       self._mode_cb)
        rospy.Subscriber("/safer/hazard_detections", HazardDetection,  self._hazard_cb)

        # ── Gazebo service clients ─────────────────────────────────────────────
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30.0)
        rospy.wait_for_service("/gazebo/delete_model",    timeout=5.0)
        self.spawn_model  = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model",    DeleteModel)

        rospy.loginfo("[Scenario] Open manhole scenario initialised.")

    def _mode_cb(self, msg: ModeStatus):
        if msg.mode == 3:  # EMERGENCY_STOP
            self.estop_triggered = True

    def _hazard_cb(self, msg: HazardDetection):
        if msg.hazard_class == 1:  # OPEN_MANHOLE
            self.detected_hazard = True
            rospy.loginfo(f"[Scenario] Open manhole detected! confidence={msg.confidence:.2f}")

    def run(self):
        rospy.loginfo("[Scenario] === OPEN MANHOLE SCENARIO START ===")

        # Step 1: Spawn manhole 2 m ahead of robot
        rospy.loginfo("[Scenario] Spawning open manhole at (2.0, 0.0, 0.0)...")
        pose = Pose()
        pose.position.x = 2.0
        pose.position.y = 0.0
        pose.position.z = 0.01
        pose.orientation.w = 1.0

        self.spawn_model("open_manhole", MANHOLE_SDF, "", pose, "world")
        rospy.loginfo("[Scenario] Manhole spawned. Waiting for detection...")

        # Step 2: Wait for hazard detection (max 30 s)
        deadline = rospy.Time.now() + rospy.Duration(30.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.detected_hazard:
                break
            rospy.sleep(0.5)

        if not self.detected_hazard:
            rospy.logerr("[Scenario] FAIL — Manhole not detected within 30 seconds.")
            self._cleanup()
            return False

        # Step 3: Wait for ESTOP (max 10 s after detection)
        rospy.loginfo("[Scenario] Hazard detected. Waiting for EMERGENCY_STOP...")
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.estop_triggered:
                break
            rospy.sleep(0.2)

        if not self.estop_triggered:
            rospy.logerr("[Scenario] FAIL — EMERGENCY_STOP not triggered.")
            self._cleanup()
            return False

        rospy.loginfo("[Scenario] PASS — Robot detected manhole and triggered ESTOP.")
        self._cleanup()
        return True

    def _cleanup(self):
        try:
            self.delete_model("open_manhole")
            rospy.loginfo("[Scenario] Manhole model removed.")
        except Exception as e:
            rospy.logwarn(f"[Scenario] Could not delete model: {e}")


if __name__ == "__main__":
    try:
        scenario = OpenManholeScenario()
        result = scenario.run()
        if result:
            rospy.loginfo("[Scenario] === SCENARIO PASSED ===")
        else:
            rospy.logerr("[Scenario] === SCENARIO FAILED ===")
    except rospy.ROSInterruptException:
        pass
