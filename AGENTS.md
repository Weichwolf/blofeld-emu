# blofeld-emu

- Build the complete Desktop SL instrument in OBJECTIVE.md. Execute unchanged
  firmware; preserve quirks. No patches, skipped routines, invented replies or
  host synthesis. Unknown accesses fail with address, operation, PC and context.
- Core repositories are organizational. Choose APIs, state ownership/layouts and
  LLVM integration for maximum complete-instrument performance and efficiency.
- Follow doc/architecture.md: complete known AOT/variants before playback, joint
  host/firmware LLVM19 optimization, direct RAM and typed device contracts.
  Keep proved checks in preparation/binding, Debug instrumentation out of Release,
  DSP56 arithmetic inline and registers live across native successors.
- Each core has one independent execution owner. Couple through bounded timed
  device/audio channels with proved causality and progress; no global stepping,
  arbitrary time slices, electrical models or local-I/O rendezvous.
- vi/ owns instrument behavior, host/ reusable transport, app/ composition/UI.
  Cores own state; VmContext binds memory/channels. test/ mirrors src/; use the
  normal instrument for integration, never a parallel firmware-test runtime.
- Query actual audio/MIDI grants. Callbacks never allocate, lock, wait, throw or
  access files. Viewer/publication is live-only in the standard runtime directory;
  offline MIDI-to-WAV reports statistics without viewer bindings or path overrides.
- Target >=10x sustained original stereo output, >=4x intermediate; physical
  key-to-analog latency 2-5 ms/max 10 ms. Startup is separate; microbenchmarks are no
  instrument proof. Development runs full AOT; deployment may cache validated code.

## Work

- German with the user; English code, comments, docs and commits. C++23, LLVM19,
  CMake/Ninja, strict warnings/linking, clang-format, zero-finding clang-tidy,
  functions <=120 lines. Never weaken checks or change correct tests to pass.
- Prioritize functionality/performance with focused independent Release checks.
  Defer routine own-code/generated-JIT ASan/UBSan and separate TSan until the goal;
  earlier runs need a concrete reason. Use packaged foreign dependencies without
  source builds or sanitizer instrumentation. No GitHub workflows.
- Bound probes to their purpose with explicit timeouts and prompt fault shutdown.
  Allow original startup; no fixed ten-second cap or unchanged failed repetition.
  Telemetry is selectable, off by default, absent from the normal hot path.
- Keep README for use, OBJECTIVE for acceptance, doc/architecture.md for the
  current target and source manifests for references. Code describes current
  implementation; ACCEPTANCE.tsv holds qualification results. No diaries, obsolete
  attempts, history archives or status narratives; history belongs in Git.
- Keep manufacturer inputs/private analysis untracked in build/local/, generated
  artifacts in build/. Preserve source/size/hash manifests and dependency notices.
  GPL-3.0-only with SPDX; no GPL-2.0-only octa-capture code. Do not alter references.
  Run as a regular user.
- Keep canonical main current; commit reviewed increments. Push only upstream/main
  here and origin/main in cores. Publish every qualified core increment; no backup
  branches, force pushes, --all/--mirror or unrelated edits. Coordinate dependency
  pins; each developer validates their native macOS ARM64 or Linux x86-64 host.
- Use identical firmware/bootstrap/program/sample/settings contracts. Shared
  presets come from ../blofeld-src/tools/blofeld_image.py; host differences stay
  in bindings/backends. Physical flashing requires established recovery.
- Coordinate only in private Weichwolf/blofeld-src: #23 instrument performance and
  platform ownership; #28 DSP AOT/entries/backend; #25 arithmetic/filter/LFO;
  #26 MIDI/SysEx; #27 MCU IRQ/display/USB ordering; #29 documentation cleanup.
  Consult existing specialist issues and local Gearmulator sources; no status spam.
