#!/usr/bin/env python3
"""
costmap_hazard_layer.py
------------------------
Subscribes to HazardDetection messages and inflates cost around detected
hazard positions in the move_base costmap via the /move_base/global_costmap/update
interface (costmap_2d OccupancyGrid update).

Subscribes:
  /safer/hazard_detections/classified  (safer_msgs/HazardDetection)

Publishes:
  /safer/costmap_hazard_updates  (nav_msgs/OccupancyGrid)
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from safer_msgs.msg import HazardDetection

# Cost values for costmap
COST_FREE       = 0
COST_INFLATED   = 128
COST_LETHAL     = 254


class CostmapHazardLayer:
    def __init__(self):
        rospy.init_node("costmap_hazard_layer", anonymous=False)

        # ── Params ────────────────────────────────────────────────────────────
        self.resolution       = rospy.get_param("~resolution",        0.05)  # m/cell
        self.width            = rospy.get_param("~width",             100)   # cells
        self.height           = rospy.get_param("~height",            100)   # cells
        self.inflation_radius = rospy.get_param("~inflation_radius",  1.0)   # metres
        self.hazard_timeout   = rospy.get_param("~hazard_timeout",    5.0)   # seconds
        self.frame_id         = rospy.get_param("~frame_id",          "map")

        # ── Local hazard store: {key: (HazardDetection, timestamp)} ───────────
        self.hazards = {}

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.grid_pub = rospy.Publisher(
            "/safer/costmap_hazard_updates", OccupancyGrid, queue_size=1)

        rospy.Subscriber("/safer/hazard_detections/classified",
                         HazardDetection, self._hazard_cb)

        rate = rospy.get_param("~update_rate", 2.0)
        rospy.Timer(rospy.Duration(1.0 / rate), self._publish_grid)

        rospy.loginfo("[CostmapHazardLayer] Ready.")

    def _hazard_cb(self, msg: HazardDetection):
        key = f"{msg.hazard_class}_{round(msg.position.x, 1)}_{round(msg.position.y, 1)}"
        self.hazards[key] = (msg, rospy.Time.now())

    def _publish_grid(self, _event):
        self._expire_hazards()

        grid_data = np.full(self.width * self.height, COST_FREE, dtype=np.int8)
        origin_x  = -(self.width  * self.resolution / 2.0)
        origin_y  = -(self.height * self.resolution / 2.0)

        for key, (hazard, _ts) in self.hazards.items():
            hx = hazard.position.x
            hy = hazard.position.y
            cost = COST_LETHAL if hazard.severity >= 3 else COST_INFLATED

            # Inflate a circle around the hazard
            radius_cells = int(self.inflation_radius / self.resolution)
            cx = int((hx - origin_x) / self.resolution)
            cy = int((hy - origin_y) / self.resolution)

            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    dist = math.sqrt(dr**2 + dc**2) * self.resolution
                    if dist > self.inflation_radius:
                        continue
                    r = cy + dr
                    c = cx + dc
                    if 0 <= r < self.height and 0 <= c < self.width:
                        idx = r * self.width + c
                        # Decay cost with distance from centre
                        decayed = int(cost * (1.0 - dist / self.inflation_radius))
                        grid_data[idx] = max(grid_data[idx], decayed)

        msg                     = OccupancyGrid()
        msg.header.stamp        = rospy.Time.now()
        msg.header.frame_id     = self.frame_id
        msg.info.resolution     = self.resolution
        msg.info.width          = self.width
        msg.info.height         = self.height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data                = grid_data.tolist()
        self.grid_pub.publish(msg)

    def _expire_hazards(self):
        now = rospy.Time.now()
        self.hazards = {
            k: v for k, v in self.hazards.items()
            if (now - v[1]).to_sec() < self.hazard_timeout
        }

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = CostmapHazardLayer()
        node.spin()
    except rospy.ROSInterruptException:
        pass
