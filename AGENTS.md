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
- Prioritize the connected instrument. Fix and optimize the responsible core for
  generic defects or measured instrument bottlenecks; core benchmark percentages
  do not block integration. SIMD is optional and needs measured benefit.
- External transport is USB-MIDI <-> instrument -> audio. Use direct audio RAM
  and USB/MIDI buffer/FIFO mappings with correct flags, lengths and IRQs. No
  external DIN/virtual MIDI ports, UART-bit path or serial-PCM decoding for output.
- Query audio/MIDI capabilities separately and read actual grants; keep bounded
  queues, visible overload/disconnect errors and unchanged virtual clocks. Audio
  callbacks must not allocate, throw, lock or access files. Native SLEIGH requires
  exceptions outside that callback. Hardware flashing needs established recovery.
- Keep the canonical checkout on main. Publish only to configured upstream/main.
  All issue communication belongs in private Weichwolf/blofeld-src; no emulator
  issues or routine status spam. Gearmulator is read-only reference; copy no code.
  Preserve GPL dependency notices; do not import GPL-2.0-only octa-capture code.

- The instrument CLI defaults to live MIDI/audio with the default local profile.
  MIDI-file plus WAV selects unpaced offline rendering with unchanged musical
  event timing. Profiles select firmware, memory images and persistent identity.
  No instruction-budget, wiring or chip-clock parameters in blofeld-headless.

## Working rules

- Speak German with the user; write code, comments, documentation and commits in English.
- Prefer simple native execution, direct memory and large bounded runs. Keep device
  effects and event boundaries explicit; avoid per-clock work and unnecessary copies.
  Telemetry is runtime-selectable and off by default; keep its work out of hot paths.
- Run builds, tests and benchmarks locally; do not add GitHub workflows or pipelines.
- Use C++23, LLVM 19 and CMake/Ninja. Keep strict compiler/linker flags, warnings
  as errors, clang-format, clang-tidy with zero findings and functions <=120 lines.
  Never weaken checks or hide findings with suppressions or diagnostic baselines.
- Release is the performance build. Preserve ASan/UBSan for project code and native
  dependencies, and separate TSan for concurrent code; preserve dependency notices.
- Run affected cpu12/dsp56300 correctness tests for core changes, including
  independent instruction/state expectations and appropriate sanitizer coverage.
  Core tests are not subject to the instrument's real-time milestone restriction.
  Avoid duplicate or full-suite runs after small changes; report gaps and scope.
- Until the instrument reaches real time, suspend long Blofeld firmware runs.
  Startup/throughput probes stop after at most 10 host seconds and fail unless
  playable; never extend or automatically repeat a failed probe. Keep compilation,
  formatting and static analysis active.
- README describes current supported behavior, setup and use. Keep progress,
  measurements and coordination in commit messages, not README.
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
