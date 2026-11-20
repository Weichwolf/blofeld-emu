# blofeld-emu

Goal: OBJECTIVE.md. Target design: doc/architecture.md. Highest priority is uninterrupted
live playback of unchanged original 1.25 firmware on A18 Pro, i5-1135G7 and
i5-10400 under simultaneous 16-part Multi, 25-voice and dense MIDI load.
Qualify the i5-1135G7 at its 2.4 GHz maximum. Physical key-to-analog latency
targets 2-5 ms (maximum 10 ms).
Decide everything else yourself.

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

- By on-time live buffers and exact original PCM under the combined worst-case
  load on each reference host. Offline throughput diagnoses headroom. Report
  startup and preparation separately.
  ACCEPTANCE.tsv records qualification.
- Less code is better. Report the src/ and test/ line delta with each published
  increment.

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
- Use GitHub issues for concrete remaining work. Keep one issue active across
  blofeld-emu and dsp56300. Keep closed issues with useful technical evidence
  and write short, result-first comments. Keep original firmware, private traces
  and raw disassembly local.
- Bound probes with explicit timeouts and stop promptly on faults. Telemetry is
  selectable, off by default and absent from the normal hot path.
- Sanitizer qualification (ASan/UBSan, separate TSan) of own and generated code
  happens at the goal, earlier only for a concrete risk.

## Branches

- `main` is the only long-lived branch, locally and on the remote. The remote
  holds `main`; no review or backup branches, no tags used as backups. One
  exception: to hand unpublished work to another developer or host, push the
  step's work branch (`work/<issue>-<step>`) and name it in the issue. Its
  owner deletes it on the remote when the step is published or dropped. Code
  is handed over as a branch, never as a patch pasted into an issue.
- Local `main` equals the remote `main` (`upstream/main` here). Fast-forward it (`git fetch --prune`,
  then `git merge --ff-only`) before starting work, before any measurement that
  names a published commit, and before every push. Nothing is committed on
  local `main` that is not pushed in the same step.
- Keep at most one `work/<issue>-<step>` branch across blofeld-emu and dsp56300
  for the active issue. Rebase it on remote `main` without merge commits.
  Publish by fast-forwarding `main`, pushing and deleting the work branch in
  the same step.
- A measurement baseline may be a detached worktree at a published commit. It
  is removed when the result is posted.
- A branch, worktree or stash is orphaned when its step is published, cancelled
  or superseded, or when no open issue step names it. Orphans are deleted, not
  archived: history on `main` and the issue record are the archive. A stash is
  never storage.
- At every publication `git branch` shows `main` plus any active work branch
  and `git worktree list` shows nothing else. No force pushes to `main`.
