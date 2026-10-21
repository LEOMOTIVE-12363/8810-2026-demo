# 0001 — The demo branch is teleop-only

- Date: 2026-10-21
- Status: Accepted

## Context

`demo` exists to run simple, reliable robot demonstrations, where autonomous
routines add failure modes without adding value.

## Decision

Remove the entire autonomous stack from `demo` (e390e9d): auto routines and
chooser, PathPlanner / Choreo builders, Autopilot trench, vision, and the
`PathplannerLib` / `Autopilot` vendordeps. Drivetrain-internal odometry stays
(field display and heading-zero use it); deploy assets are left in place but
unused.

## Consequences

- `demo` has 5 vendordeps; `main` has 7.
- Anything autonomous lives on `main` only.
