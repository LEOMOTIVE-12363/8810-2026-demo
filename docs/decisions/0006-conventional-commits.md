# 0006 — Conventional Commits on the demo line

- Date: 2026-10-21
- Status: Accepted

## Context

A small multi-contributor line with AI-assisted development benefits from a
machine-parseable history; `git log --oneline` should read like a changelog.

## Decision

Commit messages on `demo` follow Conventional Commits: `feat:`, `fix:`,
`docs:`, `chore:` (plus `refactor:` / `test:` as needed), with an optional
scope — e.g. `feat(drive): add heading zero on B`.

## Consequences

PR titles and squash-merge commits inherit the prefix, keeping the demo
history scannable by humans and AI tools alike.
