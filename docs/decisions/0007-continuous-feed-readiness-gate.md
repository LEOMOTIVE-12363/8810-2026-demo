# 0007 — Continuous feed readiness gate

- Date: 2026-10-22
- Status: Accepted

## Context

Feeding previously started on a one-time readiness check in SPIN_UP: once
the volley began, nothing re-checked the shot. A mid-volley flywheel or hood
loss — or dropping the Flywheel Speed Limit to 0% mid-demo — kept the
feeder, indexer, and intake roller running, pushing fuel into a shooter that
could not launch it. Manual had its own readiness copy with a hood tolerance
of 0.1 rot (36°) — wider than the hood's whole 0–25° range, so it never
gated.

## Decision

Gate feeding continuously in every feeding state through one shared rule,
`util/ShotReadiness` (flywheel limit > 0 AND drum at speed AND hood at
angle; tolerances are the existing `ShootingConstants` values). Pausing
stops the feed path but keeps driving the drum and hood to target (new
PAUSED state in Shooting); the volley auto-resumes on stable readiness
(0.15 s debounce) without a button re-press, and the retract timer freezes
while paused. UNJAM is never suppressed by the gate — the back-off always
runs to completion — but re-gates through PAUSED before forward feeding
resumes. Manual is unified onto the same rule (SHOOT falls back to PREPARE
on raw-not-ready, re-enters on stable-ready), and the new operator panel
(`util/OperatorStatus`, command-heartbeat based, published to Elastic at
10 Hz) reads the same evaluation, so display and control cannot drift.

## Consequences

Behavior changes, all deliberate: UNJAM no longer resumes feeding directly
but passes the readiness gate first; paused/unjam time no longer counts
toward the retract delay (previously a >0.2 s unjam meant an instant RETRACT
on resume); Manual's PREPARE→SHOOT went from a single sample to the 0.15 s
debounce; Manual's hood tolerance is now the shared 0.02 rot (7.2°). That
hood tolerance is 29% of the 25° range and may be too loose — tightening is
pending real-robot validation on Saturday. No new tuning parameters were
introduced. New read-only NetworkTables keys under /SmartDashboard
(`Robot Action`, `Feed Allowed`, `Feed Blocked Reason`,
`Drum Target Speed (rot/s)`, `Drum Actual Speed (rot/s)`,
`Battery Voltage (V)`, `Controller 1 Connected`) plus the
`Shooting/stableReady` log key are external interfaces consumed by
`elastic-layout.json` and AdvantageScope layouts.
