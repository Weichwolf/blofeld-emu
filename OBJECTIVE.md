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

Fuse eligible host IR before final LLVM 19 optimization. Keep registers live, RAM direct and DSP56
arithmetic/rounding/saturation/flags inline. Validate proved invariants during
AOT/stopped binding, revalidating relevant revisions. Release omits optional Debug/
sanitizer work and repeated proved checks; both modes preserve required dynamic
effects/faults. Preparation has no fixed budget.
Caches hold validated relocatable code, never device snapshots, and may be used
in development and operation.
Keep manufacturer inputs/generated artifacts local with source/size/hash identity.

## Performance and qualification

- **Highest priority is uninterrupted live playback in the worst case** on Apple
  A18 Pro, Intel Core i5-1135G7 and i5-10400. Run the normal instrument with
  dense MIDI on all sixteen active Multi parts while the original allocator
  sustains the full twenty-five voices, with distinct sounds, effects and
  authentic SL samples. Record voice allocation, release and stealing.
- Qualify each host with the same reproducible combined workload. Record the
  actual audio backend, sample rate, buffer sizes, run duration, callback
  deadlines and effective clock under the host's normal clock policy. Deliver
  every buffer on time with continuous, correctly ordered original PCM,
  including during MIDI bursts and first use of sounds. Require zero underruns,
  missed deadlines, lost frames and repeated frames.
  Require real-time playback in every reference scenario. Once each scenario
  meets that floor, use W5 to decide performance trade-offs.
- On the local Linux host, qualify A013 and W1-W5 on the OCTA-CAPTURE at
  44.1 kHz and 64 JACK frames with the normal clock policy. Repeat W5 for
  30 minutes in one live session and record minimum PCM publication margin,
  effective clock, PCM underruns and JACK xruns in #17. Preserve exact PCM on
  replay of the admitted MIDI timeline and preserve every MIDI event.
- Measure settled idle on the same JACK setup after release and effect tails.
  Report raw CPU usage, effective clock, per-thread CPU time and wakeups. Label
  the estimate normalized to 4.2 GHz separately. Preserve exact PCM, MIDI
  timing, retrigger latency and the W5 publication margin with the same buffer
  settings. Record the result in #52.
- Close issues against their scoped live, functional and qualification criteria.
  The deferred throughput and idle milestones have their own acceptance in #103
  and #104.
- Count valid original frames once against monotonic host processing time. Report
  compilation, boot/readiness, warm-up, active rendering, pacing and conversion
  separately. Require exact PCM with every frame delivered once at its original
  timestamp. Compare authored core algorithms with equivalent Clang19 O3 C++.
- Physical keyboard-actuation-to-analog latency targets **2-5 ms**, maximum **10 ms**,
  including scan, transport, firmware, queues, driver and DAC. Qualify an immediate-
  attack original sound under representative load; identify devices and report
  distribution, maximum/jitter and every underrun/deadline miss.
- Prove independent owners' bidirectional event horizons, ordering and progress
  under changed host scheduling. Use queried resources; no arbitrary time slices.
- ACCEPTANCE.tsv defines complete profile-D scope and separate native-platform
  results. Record the tested emulator and core revisions for each result. Pending
  proves nothing; grouped rows need complete independent evidence and fixed
  tolerances. Optional stock comparisons are not an extra gate. Source
  M/S/U/F-D resolves through doc/downloads.json and function-references.json.
- Use focused independent units and the normal instrument for reset-to-playable,
  load/long-run and recovery acceptance. Prioritize Release functionality/
  performance; complete own-code/generated-JIT ASan/UBSan and separate concurrent
  TSan qualification at the goal, earlier for concrete risks. Never instrument/
  rebuild foreign libraries or add GitHub workflows. Each developer validates
  their native host with identical inputs/settings. Probes use purpose-specific
  finite guards, allow startup, fail unless playable and stop promptly on faults;
  no unchanged failed repetition without new evidence.

## Deferred throughput and idle targets

After the remaining live, functional and qualification issues are closed, qualify
sustained 2× W5 headless rendering on the local Linux i5-1135G7 under its normal
clock policy in #103. Then qualify 4× W5 rendering on A18 Pro in #104.
The expected settled idle range for both milestones is 4–6% of total host CPU
capacity after release and effect tails. Any value at or below 6% meets the CPU
target. Apply it to measured CPU usage under each host's normal clock policy.
Use the JACK setup from #52 for Linux. Use native CoreAudio on A18 Pro and
record the actual buffer size.
At 44.1 kHz, the render targets are 88,200 and 176,400 valid stereo frames per
host second.
Use exact original PCM and guest timing over the full workload. Report W1–W5
and A013 throughput with startup and preparation measured separately.
