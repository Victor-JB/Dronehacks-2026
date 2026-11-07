# Shared Context Folder

This folder holds each team member's personal agent context file (e.g. your `AGENTS.md` / `CLAUDE.md` equivalent) for Dead Reckoning's Dronehacks 2026 project. The goal is for everyone's AI agent to be able to see what everyone else is working on, in real time.

## Rules

1. **One file per person.** Edit only your own file (`firstname-lastname.md`). Don't edit someone else's.
2. **Update anytime you make a change.** Whenever you make progress, change direction, or have your agent do something notable, update your file immediately. Don't batch updates — do it live, as you go.
3. **Never be behind local.** Your pushed file in this folder should always reflect the current state of your local context. If your local agent context has changed, this folder is already out of date — fix that before doing anything else.
4. **Always push directly to `main`.** No branches, no PRs for this folder. `git add shared-context/<your-file>.md && git commit -m "update context" && git push origin main`.
5. **Pull before you edit.** Since everyone pushes straight to main, run `git pull` before editing to avoid conflicts.

## Files

- `aryan-mehra.md` — Aryan Mehra (Lead)
- `rohan-tummala.md` — Rohan Tummala
- `tanooj-kanike.md` — Tanooj Kanike
- `victor-joulin-batejat.md` — Victor Joulin-Batejat
- `saketh-desu.md` — Saketh Desu
