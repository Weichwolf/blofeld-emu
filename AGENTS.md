# blofeld-emu

Goal: OBJECTIVE.md. Target design: doc/architecture.md. Build the complete
Blofeld Desktop SL instrument from unchanged original 1.25 firmware at >=4x
offline in every situation on the i5 reference hosts (>=10x as direction on
faster hosts) and 2-5 ms live latency (max 10 ms) on Linux x86-64 and macOS
ARM64. Decide everything else yourself.

## Fixed limits

- Execute unchanged firmware and preserve its quirks. No patches, skipped
  routines, invented replies or host synthesis. Unknown accesses fail with
  address, operation, PC and context: exactly per word in the checked
  diagnostic build, and at host-page granularity in the product build, where
  memory is protected by the host and not by checks in translated code. The
  pages in which the two differ are listed in `doc/architecture.md`.
- One native path per core: LLVM 19 AOT through cpu12 and dsp56300. Incomplete
  preparation is a failure, not a fallback. Delete what the target does not need;
  no archives or compatibility layers. Git keeps history.
- Audio and MIDI callbacks never allocate, lock, wait, throw or access files.
- Manufacturer inputs and private analysis stay untracked in build/local/,
  generated artifacts in build/. GPL-3.0-only with SPDX; keep dependency notices;
  no GPL-2.0-only octa-capture code. Do not alter reference checkouts. Run as a
  regular user. No GitHub workflows. Physical flashing requires established
  recovery.

## How work is judged

- By whole-instrument throughput with bit-identical original PCM on the reference
  workloads, measured by the developers; startup and preparation are reported
  separately. Component numbers are supporting evidence, not results.
  ACCEPTANCE.tsv records qualification.
- Less code is better. Report the src/ and test/ line delta with each published
  increment; see the reduction plan in Weichwolf/blofeld-src#31.

## Freedom

APIs, state ownership and layouts, LLVM composition, scheduling design, tools,
test strategy, caching of validated code in development, and the order of work
are your choice. Change a core interface when it obstructs the instrument, but
the cores stay generic for other devices built on the same chips: all Blofeld
knowledge (code inventory, entries, variants, memory layout, devices, timing)
lives here and reaches the cores through their public interfaces.

## Working together

- German with the user; English code, comments, docs and commits. C++23, LLVM 19,
  CMake/Ninja, warnings as errors, clang-format, clang-tidy clean.
- Commit reviewed increments. Push only upstream/main here and origin/main in the
  cores; no force pushes or backup branches. Validate your native host without
  breaking the other; both platforms use identical firmware, images and settings.
  Shared presets come from ../blofeld-src/tools/blofeld_image.py.
- Coordinate in private Weichwolf/blofeld-src issues. Comments starting with
  [Lead] come from the technical lead and set priorities: #30 experiments,
  #31 reduction, #23 whole-instrument performance, #28 DSP AOT and backend,
  #25 arithmetic/filter/LFO, #26 MIDI/SysEx, #27 MCU IRQ/display/USB.
  Write short, result-first comments: table, what changed, what is open.
- Bound probes with explicit timeouts and stop promptly on faults. Telemetry is
  selectable, off by default and absent from the normal hot path.
- Sanitizer qualification (ASan/UBSan, separate TSan) of own and generated code
  happens at the goal, earlier only for a concrete risk.

## Branches

- `main` is the only long-lived branch, locally and on the remote. The remote
  holds `main` and nothing else: no work, review or backup branches, no tags
  used as backups.
- Local `main` equals the remote `main` (`upstream/main` here). Fast-forward it (`git fetch --prune`,
  then `git merge --ff-only`) before starting work, before any measurement that
  names a published commit, and before every push. Nothing is committed on
  local `main` that is not pushed in the same step.
- Unpublished work lives on at most one work branch per open step, named
  `work/<issue>-<step>` (for example `work/37-r1`), rebased on the remote
  `main`, without merge commits. Publishing is: fast-forward `main` to it,
  push, delete the work branch, all in one step.
- A measurement baseline may be a detached worktree at a published commit. It
  is removed when the result is posted.
- A branch, worktree or stash is orphaned when its step is published, cancelled
  or superseded, or when no open issue step names it. Orphans are deleted, not
  archived: history on `main` and the issue record are the archive. A stash is
  never storage.
- At every publication `git branch` shows `main` plus the active work branches
  and `git worktree list` shows nothing else. No force pushes to `main`.
