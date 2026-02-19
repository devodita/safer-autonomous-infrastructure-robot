#!/usr/bin/env python3
"""
risk_scorer.py
---------------
Pure-logic module (no ROS) that computes a composite risk score [0.0, 1.0]
from a list of HazardDetection messages and the robot's current velocity.

Weights and thresholds are injected at construction time from risk_thresholds.yaml.
"""

import math
from typing import List, Dict, Any


# Mode constants — must stay in sync with mode_fsm.py and safer_msgs/ModeStatus.msg
MODE_NORMAL     = 0
MODE_CAUTION    = 1
MODE_RESTRICTED = 2
MODE_ESTOP      = 3


class RiskScorer:
    def __init__(self, thresholds: Dict[str, Any]):
        """
        Args:
            thresholds: dict loaded from risk_thresholds.yaml
        """
        t = thresholds

        # Weights must sum to 1.0
        self.w_proximity = t.get("weight_proximity", 0.40)
        self.w_severity  = t.get("weight_severity",  0.35)
        self.w_density   = t.get("weight_density",   0.15)
        self.w_velocity  = t.get("weight_velocity",  0.10)

        # Distance thresholds (metres)
        self.proximity_critical  = t.get("proximity_critical",  0.5)
        self.proximity_high      = t.get("proximity_high",      1.5)
        self.proximity_medium    = t.get("proximity_medium",    3.0)

        # Mode thresholds on final risk score
        self.thresh_caution    = t.get("threshold_caution",    0.30)
        self.thresh_restricted = t.get("threshold_restricted",  0.55)
        self.thresh_estop      = t.get("threshold_estop",       0.80)

        # Max pedestrian count before density risk = 1.0
        self.ped_max = t.get("pedestrian_density_max", 10)

    # ── Public API ─────────────────────────────────────────────────────────────

    def compute(self, hazards: list, linear_velocity: float,
                pedestrian_count: int = 0) -> Dict[str, Any]:
        """
        Compute risk and recommended mode.

        Returns a dict with:
          risk_score, proximity_risk, severity_risk, density_risk,
          velocity_risk, recommended_mode, reasoning
        """
        proximity_risk = self._proximity_risk(hazards)
        severity_risk  = self._severity_risk(hazards)
        density_risk   = self._density_risk(pedestrian_count)
        velocity_risk  = self._velocity_risk(linear_velocity)

        risk_score = (
            self.w_proximity * proximity_risk +
            self.w_severity  * severity_risk  +
            self.w_density   * density_risk   +
            self.w_velocity  * velocity_risk
        )
        risk_score = max(0.0, min(1.0, risk_score))

        recommended_mode, reasoning = self._recommend_mode(
            risk_score, hazards)

        return {
            "risk_score":       risk_score,
            "proximity_risk":   proximity_risk,
            "severity_risk":    severity_risk,
            "density_risk":     density_risk,
            "velocity_risk":    velocity_risk,
            "recommended_mode": recommended_mode,
            "reasoning":        reasoning,
            "active_hazards":   len(hazards),
        }

    # ── Internal scoring ───────────────────────────────────────────────────────

    def _proximity_risk(self, hazards: list) -> float:
        if not hazards:
            return 0.0
        scores = []
        for h in hazards:
            dist = math.sqrt(h.position.x**2 + h.position.y**2 + h.position.z**2)
            if dist <= self.proximity_critical:
                scores.append(1.0)
            elif dist <= self.proximity_high:
                scores.append(0.75)
            elif dist <= self.proximity_medium:
                scores.append(0.40)
            else:
                scores.append(0.10)
        return max(scores)

    def _severity_risk(self, hazards: list) -> float:
        if not hazards:
            return 0.0
        max_sev = max(h.severity for h in hazards)
        return max_sev / 3.0  # Severity scale is 0-3

    def _density_risk(self, pedestrian_count: int) -> float:
        return min(1.0, pedestrian_count / max(self.ped_max, 1))

    def _velocity_risk(self, linear_velocity: float) -> float:
        """Higher speed = less time to react = higher risk contribution."""
        max_speed = 1.5  # m/s (from robot_params.yaml)
        return min(1.0, abs(linear_velocity) / max_speed)

    def _recommend_mode(self, risk_score: float, hazards: list):
        # Hard override: any severity-3 hazard → ESTOP immediately
        if any(h.severity >= 3 for h in hazards):
            return MODE_ESTOP, "Severity-3 hazard detected — emergency stop."

        if risk_score >= self.thresh_estop:
            return MODE_ESTOP, f"Risk score {risk_score:.2f} exceeds E-STOP threshold."
        elif risk_score >= self.thresh_restricted:
            return MODE_RESTRICTED, f"Risk score {risk_score:.2f} — restricted mode."
        elif risk_score >= self.thresh_caution:
            return MODE_CAUTION, f"Risk score {risk_score:.2f} — caution mode."
        else:
            return MODE_NORMAL, f"Risk score {risk_score:.2f} — normal operation."
