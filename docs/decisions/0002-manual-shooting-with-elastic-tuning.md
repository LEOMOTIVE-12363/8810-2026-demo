# 0002 — Manual shooting with Elastic-tunable limits

- Date: 2026-10-21
- Status: Accepted

## Context

For demos, operators need quick, safe control of shot power without
recompiling.

## Decision

Make shooting fully manual on `demo` (8f34ab8) and expose four tunables in
Elastic (PR #3): Drive / Turn / Flywheel Speed Limit (%) and Hood Angle (%),
with a dedicated `demo` tab layout.

## Consequences

Speed and hood can be trimmed from the dashboard mid-demo. The NetworkTables
key names are an external interface consumed by `elastic-layout.json`.
