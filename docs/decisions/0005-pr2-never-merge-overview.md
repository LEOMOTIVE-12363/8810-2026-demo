# 0005 — PR #2 (demo → main) stays open forever

- Date: 2026-10-21
- Status: Superseded by [0009](0009-close-pr2-overview-use-compare.md) (2026-10-23)

> **Superseded on 2026-10-23 by [0009 — close PR #2, use GitHub Compare](0009-close-pr2-overview-use-compare.md).**
> PR #2 has been closed (not merged). The branch model is unchanged: `demo`
> still never merges into `main`. This record is kept for history only.

## Context

`demo` must never merge into `main`, but a live view of the total diff between
the two lines is useful. GitHub allows only one open PR per (head, base) pair.

## Decision

Keep PR #2 (demo → main) permanently open as the overview of everything
`demo` changes relative to `main`. It is never merged and never closed;
per-task work merges into `demo` via short-lived feature branches instead.

## Consequences

Direct pushes to `demo` would silently feed into PR #2's diff, so all demo
changes go through feature branches.
