# 0009 — Close PR #2; GitHub Compare replaces the standing overview

- Date: 2026-10-23
- Status: Accepted
- Supersedes: [0005](0005-pr2-never-merge-overview.md)

## Context

0005 kept PR #2 (demo → main) permanently open as a live overview of the
demo↔main diff. The maintainer no longer wants a standing overview page. The
underlying branch model is unchanged: `demo` is the demo development line and
its changes never merge into `main`.

## Decision

Close PR #2 without merging (done 2026-10-23, with an explanatory comment on
the PR). Do not reopen it and do not create a replacement overview PR. To
inspect the demo↔main difference, use GitHub Compare:
https://github.com/LEOMOTIVE-12363/Alphabots-2026-lm/compare/main...demo

Per-task work keeps the existing flow unchanged: short-lived feature branch →
PR with base `demo` → merge into `demo` once self-checks pass. README changes
are still mirrored to `main` through separate documentation-only PRs; no demo
functional code lands on `main`.

## Consequences

The diff overview is no longer a PR page; link the Compare URL above instead.
Direct pushes to `demo` no longer feed any open PR, but the one-PR-per-task
convention is kept for its review and CI trail. This supersedes 0005 only;
the "demo never merges into main" rule from the branch model stands.
