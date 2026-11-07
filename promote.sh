#!/usr/bin/env bash
# Promote a policy from the sim to the drone: merge a branch into `main` once it has passed.
#
#   ./promote.sh <branch>
#
# A policy is a branch off `main` (the flight code): refnav changes, a new modes.json mode, or both.
# Fly it here with ./fly-sim.sh --code <branch> [--mode N]; when a run ends "complete", promote it.
# This script checks, and stops at the first failure:
#   1. the branch contains main (otherwise: merge main into it, fly it again)
#   2. its tests pass (python3 -m pytest -q tests, on that exact commit)
#   3. a sim run of that exact commit ended "complete" (sim/artifacts/skybeta/*/result.json)
# then merges it into main (a merge commit naming the sim run) and prints the push commands.
# It never pushes, and never touches the drone.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$REPO/sim/artifacts/skybeta"
die() { echo "[promote] $*" >&2; exit 1; }
say() { echo "[promote] $*"; }

b="${1:-}"
[[ -n "$b" && "$b" != -h && "$b" != --help ]] || { sed -n '2,13p' "$0"; exit 2; }
[[ "$b" != main ]] || die "main is already main"
commit="$(git -C "$REPO" rev-parse --verify -q "$b^{commit}")" || die "no branch or commit '$b'"
short="$(git -C "$REPO" rev-parse --short "$commit")"

# 1. based on the current main
! git -C "$REPO" merge-base --is-ancestor "$commit" main || die "$b ($short) is already in main"
git -C "$REPO" merge-base --is-ancestor main "$commit" \
  || die "$b ($short) does not contain the current main. On $b: git merge main, then fly it again."
n="$(git -C "$REPO" rev-list --count "main..$commit")"
(( n > 0 )) || die "$b adds nothing to main"
say "$b @ $short: $n commit(s) on top of main"

# 2. its tests, on exactly that commit
python3 -m pytest --version >/dev/null 2>&1 || die "needs pytest: python3 -m pip install pytest numpy"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
git -C "$REPO" archive "$commit" | tar -xf - -C "$tmp"
[[ -d "$tmp/tests" ]] || die "$b has no tests/ folder"
( cd "$tmp" && python3 -m pytest -q tests ) || die "tests fail on $b @ $short"
say "tests pass"

# 3. a completed sim run of exactly that commit
run="$(python3 - "$RUNS" "$commit" <<'PY'
import glob, json, os, sys
runs, commit = sys.argv[1:]
ok = []
for f in glob.glob(os.path.join(runs, "*", "result.json")):
    r = json.load(open(f))
    if r.get("commit") == commit and r.get("result") == "complete":
        ok.append((os.path.getmtime(f), os.path.basename(os.path.dirname(f)), r.get("mode") or "route.json"))
if ok:
    _, name, mode = max(ok)
    print(f"{name} (mode {mode})")
PY
)"
[[ -n "$run" ]] || die "no sim run of $b @ $short ended \"complete\". Fly it: ./fly-sim.sh --code $b [--mode N]"
say "sim: $run ended complete"

# merge, where main is checked out (or in a temporary checkout)
wt="$(git -C "$REPO" worktree list --porcelain | awk '/^worktree /{sub(/^worktree /, ""); w=$0} /^branch refs\/heads\/main$/{print w}')"
made=""
if [[ -z "$wt" ]]; then
  wt="$tmp/main"
  git -C "$REPO" worktree add -q "$wt" main
  made=1
fi
git -C "$wt" diff --quiet && git -C "$wt" diff --cached --quiet \
  || die "main's checkout ($wt) has uncommitted changes; commit or stash them first"
git -C "$wt" merge -q --no-ff -m "Promote $b to main

Tests pass; sim run $run ended complete on $short." "$commit" \
  || die "merge conflict in $wt: resolve it there (git status), or git -C '$wt' merge --abort"
[[ -z "$made" ]] || git -C "$REPO" worktree remove "$wt"
say "main is now $(git -C "$REPO" rev-parse --short main). To ship it:"
echo "    git push origin main          # GitHub"
echo "    git push drone main           # the drone (on the ground); builds on the next fly.sh start"
