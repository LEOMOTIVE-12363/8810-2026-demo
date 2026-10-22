# 0008 — Dual-controller routing with demo-port priority

- Date: 2026-10-22
- Status: Accepted

## Context

Demo sessions need a simplified controller that a guest can hold without
triggering calibration or tuning bindings, while the full control scheme must
remain available as a fallback. Priority must be unambiguous and must not
depend on stick movement ("whoever moved last wins" is not acceptable for a
demo handoff). Until now the robot ran a single `CommandXboxController` on DS
port 1 carrying the full binding set.

## Decision

`util/DriverControls` owns two `XboxController` instances (ports 0 and 1) and
routes exclusively by connection state: port 1 (simplified demo controller)
wins whenever it is connected, port 0 (full controls) is used only while port
1 is absent, and with neither connected the drive axes read zero. The
`Action` enum maps each physical input once and marks actions as shared or
full-only:

- Shared (both ports): left-stick drive / right-stick-X turn, LT intake, LB
  full-chain reverse eject, RB `Shooting(false)`, D-pad left inward intake
  homing, B reset heading (blue 0° / red 180°, position kept).
- Full-only (port 0): A `Shooting(true)` current-feed, X `Manual`, Y hood
  zero + outward intake zero, RT feeder/indexer-only reverse, D-pad up/down
  flywheel percent, Start sim-only field reset.

Every binding creates one command instance per source port, gated by
`activePort() == source`; press/hold commands are wrapped in `onlyWhile`, so a
held command — including one-shot homing runs — is interrupted the moment its
controller disconnects or loses priority (both homing commands already stop
their motors in `end(interrupted)`).

The LB/RT reverse keeps its original momentary semantics: press writes the
reverse voltages once, a release binding zeroes them (fires on release,
disconnect, or priority loss), and a requirement-free heartbeat feeds the LED
ball-flow layer and operator panel. No subsystem ownership is taken, so a
concurrently running Shooting loop overwrites the reverse writes — that
documented conflict is intentionally preserved (2026-10-22 user decision;
automatic UNJAM covers real clearing needs).

The operator panel is published once, from `Robot`'s 10 Hz block (the
duplicate call in `RobotContainer.updateDashboardOutputs` from parallel
development was removed). New read-only keys: `Controller 0 Connected`,
`Active Controller` (-1 = neither) under `/SmartDashboard`, plus the
`Controls/ActiveController` log key; widgets added to the Elastic layout,
which moves to a dated file name (`elastic-layout-1022.json`) from this
change onward.

## Consequences

Two controllers can stay plugged in during demos: the guest controller (port
1) always wins, and unplugging it hands control to port 0 without restarting
anything. Drive scaling (left axes ×0.9), deadbands, thresholds, and every
command class are unchanged from the single-controller scheme. Verified by
`DriverControlsTest` (10 tests: connection priority, axis routing, per-port
firing, full-only isolation on port 1, release/disconnect/priority-loss
cancellation, disable cancellation, one-shot homing cancellation) and the
full `./gradlew build` + `simulateJava` smoke. Not verified on the real
robot: real DS port assignment, handoff feel, and eject effect on hardware.
