# 0004 — Untrack the inherited CLAUDE.md

- Date: 2026-10-21
- Status: Accepted

## Context

`CLAUDE.md` arrived from the upstream team repository: it documents the
upstream team's development process, which does not apply to this fork and
misled readers (human or AI agents) of the demo line.

## Decision

Untrack `CLAUDE.md` with `git rm --cached` — on `demo` (#7) and on `main`.
No history rewrite; the old blob stays in history. Maintainers keep local,
private copies (gitignored).

## Consequences

AI tools reading either branch no longer see upstream-team instructions
presented as this repo's rules.
