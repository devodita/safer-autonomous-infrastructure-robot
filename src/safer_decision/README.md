# safer_decision

Risk scoring and finite state machine for the SAFER robot.

## Modules

| File | Role |
|---|---|
| `risk_scorer.py` | Pure-logic composite risk score computation |
| `mode_fsm.py` | FSM with hysteresis (NORMAL → CAUTION → RESTRICTED → ESTOP) |
| `risk_planner_node.py` | ROS node wiring the two together |

## Key Design Decisions

- **Step up immediately, step down slowly** — the FSM transitions to a higher-risk mode on any single recommendation, but requires `hysteresis_duration` seconds of sustained lower-risk readings before stepping down.
- **ESTOP is latched** — once triggered, only a manual `ModeCommand` with issuer authority can clear it.
- **Hazard expiry** — detections older than `hazard_timeout` seconds are silently dropped.
