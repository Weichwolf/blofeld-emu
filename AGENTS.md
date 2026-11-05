# blofeld-emu

- Execute unmodified original 1.25 firmware from reset to a complete standalone
  instrument. The supplied acceptance scope includes Desktop and Keyboard; keep
  profile work explicit. Preserve original DSP audio, MIDI/USB, display, storage,
  active SL samples and update/recovery behavior. See OBJECTIVE.md.
- Correctness concerns instrument behavior. Reconstruct missing contracts without
  physical disassembly; preserve firmware quirks. No firmware patches, skipped
  routines, invented success responses or replacement host synthesizer.
- Use our independent cpu12 and dsp56300 cores; fix generic core defects there.
  Separate board model, core execution, transport and audio. Unknown accesses
  fail with address, operation, PC and context. Run as a regular user.
- Query audio/MIDI capabilities separately and read actual grants; keep bounded
  queues, visible overload/disconnect errors and unchanged virtual clocks. Audio
  callbacks must not allocate, throw, lock or access files. Native SLEIGH requires
  exceptions outside that callback. Hardware flashing needs established recovery.
- Keep the canonical checkout on main. Publish only to configured upstream/main.
  All issue communication belongs in private Weichwolf/blofeld-src; no emulator
  issues or routine status spam. Gearmulator is read-only reference; copy no code.
  Preserve GPL dependency notices; do not import GPL-2.0-only octa-capture code.

## Working rules

- Speak German with the user; write code, comments, documentation and commits in English.
- Prefer simple native execution, direct memory and large bounded runs. Keep device
  effects and event boundaries explicit; avoid per-clock work and unnecessary copies.
  Telemetry is runtime-selectable and off by default; keep its work out of hot paths.
- Use C++23, LLVM 19 and CMake/Ninja. Keep strict compiler/linker flags, warnings
  as errors, clang-format, clang-tidy with zero findings and functions <=120 lines.
  Never weaken checks or hide findings with suppressions or diagnostic baselines.
- Release is the performance build. Preserve ASan/UBSan for project code and native
  dependencies, and separate TSan for concurrent code; preserve dependency notices.
- Until the instrument reaches real time, suspend component, functional, regression
  and diagnostic execution tests. Only short optimized startup/throughput probes
  may establish that milestone: stop after at most 10 host seconds, fail unless
  playable, and never extend or automatically repeat a failed probe. Compilation,
  formatting and static analysis remain active. Run fuzzing only on explicit request.
- Afterwards validate affected behavior against independent expectations; avoid
  duplicate or full-suite runs after small changes. Report gaps and measured scope.
- Keep doc/ for one target architecture and important primary references only.
  Requirements belong in OBJECTIVE.md and test/; build metadata belongs in cmake/.
  No activity diaries or historical result manifests. Issue communication belongs
  in private Weichwolf/blofeld-src, with concrete consolidated problems only.
- Keep firmware, samples, manuals, disassemblies and private analysis untracked.
  Preserve source URLs, sizes and hashes in reference manifests; never publish assets.
  Local reference inputs live under ignored build/local/; run artifacts belong
  under ignored build/. Preserve inputs when cleaning generated files.
- Original code is GPL-3.0-only; add matching SPDX identifiers to new source.
- Commit reviewed, verified increments; push only to the configured project remote
  on main. Never publish backup branches or use --all/--mirror. Coordinate core
  revisions with consumer dependency pins; preserve unrelated work.
