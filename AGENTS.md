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
- Target >=4x sustained offline instrument throughput; design for 10x. Measure
  valid original stereo frames, startup separately; microbenchmarks are no proof.

## Working rules

- Speak German with the user; code, comments, documentation and commits are English.
- C++23, LLVM 19, CMake/Ninja; strict warnings/linking, clang-format, zero-finding
  clang-tidy, functions <=120 lines. Never suppress or weaken checks.
- Use Release for performance. Test affected core semantics independently and with
  ASan/UBSan, including native dependencies; use separate TSan for concurrent code.
  Keep checks focused; no GitHub workflows.
- Blofeld firmware probes stop within 10 host seconds and fail unless playable.
  Until real time, no long firmware runs or unchanged failed-probe repetitions.
- Telemetry is runtime-selectable and off by default; no hot-path statistics.
- README describes implemented use; OBJECTIVE defines acceptance; doc/ contains
  one target architecture and primary references. No diaries or result archives.
- Keep firmware, samples, manuals, disassemblies and private analysis untracked.
  Reference inputs: ignored build/local/; artifacts: build/. Preserve inputs and
  source/size/hash manifests. Original source is GPL-3.0-only with SPDX notices.
- Keep canonical main current. Commit reviewed increments; push only configured
  upstream/main (blofeld-emu) or origin/main (cores). No backup branches or
  --all/--mirror; preserve unrelated work and coordinate consumer dependency pins.
- Issues belong only in private Weichwolf/blofeld-src; no routine status spam.
  Consult local implementations such as Gearmulator for known register maps,
  protocols and algorithms; write our implementation independently. Keep
  reference checkouts unchanged and preserve existing dependency notices.
  Do not import GPL-2.0-only octa-capture code. Run as a regular user.
- Validate ARM64 on macOS and x86-64 on Linux; communicate revisions in commits.
