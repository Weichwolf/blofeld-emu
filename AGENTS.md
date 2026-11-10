# blofeld-emu

- Build the complete instrument in OBJECTIVE.md with our cpu12/dsp56300 JITs.
  Execute original firmware; preserve its quirks. No firmware patches, skipped
  routines, invented replies or host synthesis. Unknown accesses fail with
  address, operation, PC and context.
- Each core has its own execution owner and runs natively at maximum throughput.
  Couple through audioframes, bounded buffers and actual device transactions.
  Never synchronize the DSP merely because an MCU instruction or local I/O ran.
- Use direct memory and functional register/FIFO/IRQ contracts. Remove global
  clock stepping, electrical wiring models and redundant simulation layers.
- VmContext binds concrete cores, memory and channels; cores own their state.
  vi/ owns instrument behavior, host/ reusable transport, app/ composition/UI.
- test/ mirrors src/ and contains simple unit tests. The normal emulator is the
  integration test; do not recreate a parallel firmware-test runtime.
- Preserve musical time, message ordering, backpressure, reset and recovery.
  Query actual audio/MIDI grants. Audio callbacks never allocate, lock, throw
  or access files; hardware flashing requires established recovery.
- Emulator and display viewer share the standard user runtime display directory.
  Do not add profile or viewer display-path overrides.
  Bind the viewer and publish LCD snapshots only in live mode. Offline MIDI-to-WAV
  has no viewer binding and reports rendering statistics in the terminal.
- Target >=4x sustained offline instrument throughput; design for 10x. Measure
  valid original stereo frames, startup separately; microbenchmarks are no proof.

## Working rules

- Speak German with the user; code, comments, documentation and commits are English.
- C++23, LLVM 19, CMake/Ninja; strict warnings/linking, clang-format, zero-finding
  clang-tidy, functions <=120 lines. Never suppress or weaken checks.
- Functionality and instrument performance take priority. Use Release and focused
  independent checks for affected own-code semantics while iterating. Defer routine
  ASan/UBSan and separate TSan qualification until the goal is reached; run them
  earlier only for a concrete technical reason. Sanitize our code/generated JIT
  code, not foreign libraries. Use packaged dependencies without source builds.
  Keep checks focused; no GitHub workflows.
- Bound firmware probes to the duration needed for their stated purpose, with
  explicit timeout guards and prompt fault shutdown. A run fails unless playable;
  do not repeat unchanged failed probes without new evidence.
- Telemetry is runtime-selectable and off by default; no hot-path statistics.
- README describes implemented use; OBJECTIVE defines acceptance; doc/ contains
  one target architecture and primary references. No diaries or result archives.
- Keep firmware, samples, manuals, disassemblies and private analysis untracked.
  Reference inputs: ignored build/local/; artifacts: build/. Preserve inputs and
  source/size/hash manifests. Original source is GPL-3.0-only with SPDX notices.
- Keep canonical main current. Commit reviewed increments; push only configured
  upstream/main (blofeld-emu) or origin/main (cores). No backup branches or
  --all/--mirror; preserve unrelated work and coordinate consumer dependency pins.
  Push every qualified JIT-core increment; never leave core commits unpublished.
- Issues belong only in private Weichwolf/blofeld-src; no routine status spam.
  Consult local implementations such as Gearmulator for known register maps,
  protocols and algorithms; write our implementation independently. Keep
  reference checkouts unchanged and preserve existing dependency notices.
  Do not import GPL-2.0-only octa-capture code. Run as a regular user.
- Each developer validates their native host: ARM64 on macOS, x86-64 on Linux.
  Both use the same firmware, complete bootstrap, programs, samples, settings
  and image contracts. Shared presets come from ../blofeld-src/tools/blofeld_image.py;
  platform differences belong only to host bindings and native JIT backends.
  Coordinate cross-platform optimizations in private blofeld-src
  [issue #23](https://github.com/Weichwolf/blofeld-src/issues/23).
  Communicate dependency revisions in commits.
