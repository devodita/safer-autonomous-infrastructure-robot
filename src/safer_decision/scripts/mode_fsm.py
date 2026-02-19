#!/usr/bin/env python3
"""
mode_fsm.py
------------
Finite State Machine governing the SAFER robot's operational mode.

States:
  NORMAL      — full speed, no alerts
  CAUTION     — reduced speed, visual alert
  RESTRICTED  — slow speed, audible + visual alert
  EMERGENCY_STOP — zero speed, full alert, latched until manual reset

Transitions are driven by RiskAssessment messages.
Manual ModeCommand messages can force any state (with operator authority).
"""

import rospy
import time
from typing import Optional

# Mode IDs — must stay in sync with ModeStatus.msg
MODE_NORMAL     = 0
MODE_CAUTION    = 1
MODE_RESTRICTED = 2
MODE_ESTOP      = 3

MODE_NAMES = {
    MODE_NORMAL:     "NORMAL",
    MODE_CAUTION:    "CAUTION",
    MODE_RESTRICTED: "RESTRICTED",
    MODE_ESTOP:      "EMERGENCY_STOP",
}

# Valid transitions: from_state → set of reachable states
VALID_TRANSITIONS = {
    MODE_NORMAL:     {MODE_NORMAL, MODE_CAUTION, MODE_RESTRICTED, MODE_ESTOP},
    MODE_CAUTION:    {MODE_NORMAL, MODE_CAUTION, MODE_RESTRICTED, MODE_ESTOP},
    MODE_RESTRICTED: {MODE_NORMAL, MODE_CAUTION, MODE_RESTRICTED, MODE_ESTOP},
    MODE_ESTOP:      {MODE_ESTOP},  # ESTOP is latched — only manual_reset() clears it
}


class ModeFSM:
    def __init__(self, hysteresis_duration: float = 2.0):
        """
        Args:
            hysteresis_duration: seconds a mode must be continuously recommended
                                 before the FSM steps DOWN (safety: step up immediately,
                                 step down slowly).
        """
        self.current_mode     = MODE_NORMAL
        self.previous_mode    = MODE_NORMAL
        self.mode_entry_time  = time.monotonic()
        self.estop_latched    = False
        self.hysteresis_dur   = hysteresis_duration

        # Track how long the recommended mode has been stable
        self._pending_mode      : Optional[int] = None
        self._pending_since     : float         = 0.0

    # ── Public API ─────────────────────────────────────────────────────────────

    def update(self, recommended_mode: int) -> bool:
        """
        Feed the FSM a new recommended mode from the risk scorer.
        Returns True if a state transition occurred.
        """
        if self.estop_latched:
            return False

        # Step UP immediately (safety-critical direction)
        if recommended_mode > self.current_mode:
            return self._transition(recommended_mode)

        # Step DOWN only after hysteresis window
        if recommended_mode < self.current_mode:
            now = time.monotonic()
            if self._pending_mode != recommended_mode:
                self._pending_mode  = recommended_mode
                self._pending_since = now
                return False
            elif (now - self._pending_since) >= self.hysteresis_dur:
                self._pending_mode = None
                return self._transition(recommended_mode)

        return False

    def manual_command(self, mode: int, issuer: str = "operator") -> bool:
        """Force a mode transition via operator command."""
        if mode == MODE_ESTOP:
            self.estop_latched = True
            rospy.logwarn(f"[ModeFSM] E-STOP latched by {issuer}.")
        return self._transition(mode)

    def manual_reset(self) -> bool:
        """Clear ESTOP latch and return to NORMAL. Requires operator action."""
        if not self.estop_latched:
            return False
        rospy.loginfo("[ModeFSM] E-STOP latch cleared by operator reset.")
        self.estop_latched = False
        return self._transition(MODE_NORMAL)

    @property
    def time_in_mode(self) -> float:
        return time.monotonic() - self.mode_entry_time

    def status_dict(self) -> dict:
        return {
            "mode":          self.current_mode,
            "previous_mode": self.previous_mode,
            "time_in_mode":  self.time_in_mode,
            "estop_latched": self.estop_latched,
            "status_message": MODE_NAMES[self.current_mode],
        }

    # ── Internal ───────────────────────────────────────────────────────────────

    def _transition(self, target_mode: int) -> bool:
        if target_mode == self.current_mode:
            return False
        if target_mode not in VALID_TRANSITIONS.get(self.current_mode, set()):
            rospy.logwarn(f"[ModeFSM] Blocked invalid transition "
                          f"{MODE_NAMES[self.current_mode]} → {MODE_NAMES.get(target_mode)}")
            return False

        rospy.loginfo(f"[ModeFSM] {MODE_NAMES[self.current_mode]} → {MODE_NAMES[target_mode]}")
        self.previous_mode   = self.current_mode
        self.current_mode    = target_mode
        self.mode_entry_time = time.monotonic()
        return True
