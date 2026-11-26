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
- Close each issue on its own live, functional and qualification evidence.
  Track 2× and 4× offline rendering as separate follow-ups in #103 and #104.
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
- Run compiler builds and AOT preparation with `nice -n 15`. Keep builds
  outside live measurement intervals.
- Commit reviewed increments. Validate your native host without breaking the
  other. Both platforms use identical firmware, images and settings from the
  public Desktop profile recipe.
- Use GitHub issues for concrete remaining work. Keep closed issues with useful
  technical evidence and write short, result-first comments. Keep original
  firmware, private traces and raw disassembly local.
- Bound probes with explicit timeouts and stop promptly on faults. Telemetry is
  selectable, off by default and absent from the normal hot path.
- Sanitizer qualification (ASan/UBSan, separate TSan) of own and generated code
  happens at the goal, earlier only for a concrete risk.

## Branches

- Each host or agent may work on one `{hostname}/{issueid}_{issuename}` branch
  with one open issue at a time. The owner creates and merges the pull request
  with `gh` after required checks and approvals pass.
- `main` is the only long-lived branch. Keep local `main` synchronized with
  `upstream/main` by fetching and fast-forwarding before work, before measuring
  a published revision and after a PR merge. Never force-push `main`.
- Remove work branches when merged, cancelled or superseded. Remove detached
  measurement worktrees when the result is posted. Git history and technical
  issue results retain the evidence.
