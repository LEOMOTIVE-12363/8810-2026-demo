# 0003 — MIT LICENSE on both branches

- Date: 2026-10-21
- Status: Accepted

## Context

GitHub's sidebar license detection scans only the default branch (`main`)
root. Without a standard LICENSE it showed an unidentified "Other /
NOASSERTION", matching only the bundled AdvantageKit / WPILib notices.

## Decision

Add the standard MIT text under "FRC Team 8810 Alphabots" on `demo` (#4) and
on `main` (#5). The copy on `main` is an intentional, permanent fork-only
commit ahead of the upstream team repository.

## Consequences

The repo sidebar shows "MIT license". Upstream syncs must preserve this
commit.
