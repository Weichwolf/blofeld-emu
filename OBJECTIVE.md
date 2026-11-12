# Objective

Deliver the complete Blofeld Desktop with active SL on macOS ARM64 and Linux
x86-64, using unchanged original 1.25 firmware and our cpu12/dsp56300 translators.
Preserve original sound, quirks, musical time and device effects. No firmware
patches, skipped routines, invented replies, host synthesis or electrical models.
Optimize all three projects as one instrument. Choose their interfaces, state
layouts and LLVM composition for maximum end-to-end performance and efficiency.

## Instrument

- Original boot/reset, synthesis, wavetables/samples, filters, effects, modulation,
  voice allocation, sixteen-part Multi, controls, display and persistent programs.
- External MIDI keyboard, bidirectional DIN and real host-visible USB-MIDI with
  original descriptors/endpoints/status/IRQs. Qualify DAWs, controllers, ordinary
  MIDI tools and Spectre under Wine, including sample transfers and failures.
  No duplicate routing, USB Audio or Blofeld Keyboard/keybed emulation.
- Verified local identity/device/sample images with original active SL and
  authentic Waldorf standard-library playback. Preserve reset, valid/damaged user
  firmware updates, interruption/power loss, protection/security and recovery.
  Physical flashing requires established recovery independently of software tests.
- No-argument headless live mode from the standard profile. SMF plus WAV selects
  streamed offline rendering with original tempo, ordering, SysEx and tails.
  Query actual host grants; callbacks never allocate, lock, wait, throw or access files.
- Live-only independent LCD viewer in the standard runtime directory, explicit
  unknown state and stale-image removal on exit/crash without emulator heartbeats.
  Offline opens no viewer or host audio and reports actual render statistics.

## Native execution

Follow [doc/architecture.md](doc/architecture.md). Complete known AOT/variants,
loader outputs and indirect/callback/IRQ entries before playback; fail gaps with
identity, address and reason. Preserve observable program reads/writes while
selecting prepared native code. Startup loading and build-time IR/object export
share one semantic pipeline. No first-use compilation or private-layout copies.

Fuse eligible host IR before LLVM19 O3. Keep registers live, RAM direct and DSP56
arithmetic/rounding/saturation/flags inline. Validate proved invariants during
AOT/stopped binding, revalidating relevant revisions. Release omits optional Debug/
sanitizer work and repeated proved checks; both modes preserve required dynamic
effects/faults. Preparation has no fixed budget.
Caches hold validated relocatable code, never device snapshots, and may be used
in development and operation.
Keep manufacturer inputs/generated artifacts local with source/size/hash identity.

## Performance and qualification

- Sustain **>=10x offline**, >=4x intermediate, with sixteen active parts, up to
  twenty-five actual voices, distinct programs, original effects and authentic SL
  samples together. Include heavier combinations and record resource limits,
  allocation/release/stealing. At 44.1 kHz, 4x = 176400 and 10x = 441000 stereo frames/s.
- Count valid original frames once against monotonic host processing time. Report
  compilation, boot/readiness, warm-up, active rendering, pacing and conversion
  separately. No lost, repeated or retimed frames; core/silent-startup results do
  not qualify. Compare authored core algorithms with equivalent Clang19 O3 C++.
- Physical keyboard-actuation-to-analog latency targets **2-5 ms**, maximum **10 ms**,
  including scan, transport, firmware, queues, driver and DAC. Qualify an immediate-
  attack original sound under representative load; identify devices and report
  distribution, maximum/jitter and every underrun/deadline miss.
- Prove independent owners' bidirectional event horizons, ordering and progress
  under changed host scheduling. Use queried resources; no arbitrary time slices.
- ACCEPTANCE.tsv defines complete profile-D scope and separate native-platform
  results. Pending proves nothing; grouped rows need complete independent evidence
  and fixed tolerances. Optional stock comparisons are not an extra gate. Source
  M/S/U/F-D resolves through doc/downloads.json and function-references.json.
- Use focused independent units and the normal instrument for reset-to-playable,
  load/long-run and recovery acceptance. Prioritize Release functionality/
  performance; complete own-code/generated-JIT ASan/UBSan and separate concurrent
  TSan qualification at the goal, earlier for concrete risks. Never instrument/
  rebuild foreign libraries or add GitHub workflows. Each developer validates
  their native host with identical inputs/settings. Probes use purpose-specific
  finite guards, allow startup, fail unless playable and stop promptly on faults;
  no unchanged failed repetition without new evidence.
