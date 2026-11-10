# Objective

A complete standalone Blofeld Desktop SL instrument executing unchanged original
1.25 firmware from reset, using our independent cpu12 and dsp56300 JITs, on
macOS ARM64 and Linux x86-64. An external MIDI keyboard supplies live notes;
emulating the Blofeld Keyboard model and its keybed controller is outside scope.
Correctness is instrument behavior: sound, musical time and documented functions.
Electrical wiring, bus phases and cycle-by-cycle lockstep are not objectives.
See [target architecture](doc/architecture-design.txt).

## Acceptance

- Original DSP synthesis, all oscillators/wavetables, filters, effects, voices,
  modulation, multitimbrality, controls, MIDI/SysEx and display functions.
- Bidirectional classic MIDI and an actual host-visible USB-MIDI device using
  original descriptors, endpoints, status and IRQ behavior. No duplicate routing
  or USB Audio. Qualify ordinary MIDI tools, DAW/controller routing and original
  Spectre under Wine, including sample transfers, cancellation and failures.
- Persistent unit identity, programs and selectable memory/sample images.
  Build a local vanilla image from verified device inputs and Waldorf's standard
  library; original firmware must recognize active SL and play actual samples.
  Establish required ROM/wavetables from accesses; keep missing inputs explicit.
- Original boot, reset, valid/damaged firmware updates and recovery, including
  interrupted transfers and power loss during every erase/program phase, flash
  protection and security state. Support user firmware through those paths.
  Reload creates/rebinds core translations; no update logic belongs in the cores.
- Headless standalone live MIDI/audio with no arguments and a default local
  profile. Profiles select firmware, persistent images and identity. SMF plus
  WAV selects streamed offline rendering at maximum speed, preserving tempo,
  ordering, SysEx, musical timestamps and release/effect tails. No user-facing
  instruction-budget, wiring or chip-clock options; no separate control CLI.
- Use the session's normal audio output and actual capability/latency grants.
  Bounded queues report overload, underruns and disconnects. Audio callbacks
  never allocate, lock, throw or access files.
- In live mode, independent GTK display from original LCD state. Atomically publish PBM
  under $XDG_RUNTIME_DIR/blofeld-emu/; follow directory
  recreation. Detect publisher exit/crash without periodic emulator writes; never
  present stale images as live. Unknown display contents remain explicit.
  Offline MIDI-to-WAV has no viewer binding; report rendering statistics in the console.
- Simple unit tests mirror src/. The normal emulator provides reset-to-instrument,
  long-run/load, trace and memory-inspection acceptance runs. Cover every documented
  Desktop SL function on both host platforms in ACCEPTANCE.tsv, with fixed scope,
  tolerances and independent references; incomplete scope never counts as passed.

No firmware patches, skipped routines, invented success replies or substitute
host synthesizer. Preserve firmware quirks and distinguish evidence from
hypotheses. Reconstruct missing contracts independently; physical disassembly
or hardware access is not a prerequisite. Emulator success does not prove
physical flashing safety; hardware programming needs established recovery.
Manufacturer assets remain local, with source URLs, sizes and hashes recorded.

## Acceptance catalogue

ACCEPTANCE.tsv is the project-owned inventory of required instrument functions,
host operation and qualification gates, not an executable test or a manufacturer
certification. Profile D means Desktop with active SL. The macos_arm64 and
linux_x86_64 columns record separate emulator results; the Linux developer owns
Linux validation. Grouped rows pass only when their complete stated scope passes
with identified inputs, independent references and fixed tolerances. Unit tests
and isolated helpers cannot establish a playable-instrument pass.

Status values are pending, failed, passed and not_applicable. Pending is no
evidence of completion. hardware_reference records optional comparisons with
a stock Blofeld; it is not an additional completion gate. The physical latency
measurement required below still belongs in each platform's emulator result.

Source M cites pages of the German Blofeld manual, version 1.1; S cites the German
Spectre manual; U cites the firmware-update instructions. doc/downloads.json pins
their manufacturer URLs, sizes and hashes. F-D cites the Desktop FAQ snapshot
identified in doc/function-references.json. OBJECTIVE cites this specification.

## Performance and operation

Require >=4x sustainable offline throughput under representative high polyphony,
effects and sample use on this host. Pursue 10x as the optimization target;
reaching 4x does not end performance work. Quantify the remaining gap and limiting
costs with reproducible instrument measurements. Keep instrument behavior, guest
time, complete audio and bounded live latency. Change the >=4x target only with
quantitative reproducible feasibility evidence.

The combined load includes Multi mode with sixteen simultaneously active parts
and up to twenty-five total voices, with distinct programs, oscillators, filters,
modulation, original effects and authentic SL sample playback together. Include
programs that attain twenty-five allocated voices and heavier sound combinations;
record actual voice allocation, release and stealing, preserving the firmware's
resource-dependent limits. Measure fixed instrument cost and added voice/part,
effect and sample costs separately before extrapolating capacity.

Live operation must deliver every audio buffer on time without host-induced gaps,
lost or repeated frames, including first use of firmware paths and JIT translation.
Average throughput or offline reserve cannot compensate for a missed live deadline.

Prioritize optimized host execution over translation speed. Both cpu12 and
dsp56300 require an explicit preparation/optimization phase before run: analyze
the loaded executable program, build semantic data/control flow and optimize it
across instructions and blocks before native lowering. Apply constant/range
propagation, redundant computation/load/store elimination, register value
propagation and appropriate loop/vector optimization with exact guest semantics.
Host C++ -O3 alone does not optimize machine code emitted at runtime.

Allow substantial startup preparation without a fixed time budget; the suggested
one second is not a limit. Prepare the established executable working set before
playback, and support subsequently loaded/modified code with exact invalidation
and optimized retranslation. Preparation performs no guest execution, real I/O
or device-time advance. Measure coverage and first use, including interrupt
entries, indirect targets and runtime-loaded code. Report preparation separately
from original boot, warm execution and total readiness. It does not extend a
failed firmware probe. A persistent native-code cache may amortize validated
preparation; it never substitutes for optimized execution.

Use heterogeneous host cores deliberately: prioritize MCU/DSP execution on
performance resources and place preparation/background work on efficiency
resources using supported host scheduling contracts. Query actual topology and
verify placement behavior; a successful QoS request alone is not proof of core
assignment. Keep host scheduling out of guest ISA and device-time semantics.
Do not decode arbitrary data or unknown memory as established executable code.
If caching is needed, key it by executable-input hashes, exact JIT revision, host
architecture/features, code-generation options and binding ABI. Relocate instance
addresses on load; never restore device state from a code cache. Reject damaged or
incompatible entries and rebuild before playback. Preparation and caching never
substitute for removing measured runtime overhead.

Live key-to-sound latency targets 2-5 ms, with a hard upper bound of 10 ms,
on both macOS ARM64 and Linux x86-64. Measure from physical keyboard actuation
to the synthesized sound at the analog audio output, including keyboard scan,
MIDI transport, original firmware/synthesis, queues, driver and D/A conversion.
Qualify with a specified immediate-attack original sound and representative
polyphony, effects and sample load. Identify the keyboard, transport and audio
device; report the latency distribution, maximum and jitter. Buffer duration,
MIDI-arrival-to-render timing and driver latency estimates are separate partial
metrics and cannot establish this end-to-end requirement. No measured response
in the qualified live load scope may exceed 10 ms; retain underruns and misses
as failures. Keep startup separate from live response latency.

RTF = valid delivered stereo frames / (44100 * monotonic host seconds).
4x is 176400 frames/s; 10x is 441000. Count each actual frame once. Report startup,
warm-up, output rate/conversion, active processing, pacing, underruns and MIDI/audio
latency separately. Live status is bounded-rate readable text; offline completion
reports actual totals. Isolated JIT speed and sampled CPU shares are not instrument
throughput. The cores' >=70% matched host-ASM target does not gate integration.

Milestones remain 1x/99%, 10x/99.9% and 100x/99.99% playable-instrument coverage;
these do not replace complete acceptance or the >=4x sustained requirement.
Choose explicit probe guards for the stated diagnostic or qualification purpose;
allow the original startup to complete. A probe fails unless playable; no
unchanged failed repetition without new evidence. Qualify MIDI-triggered original
sound and at least ten seconds of audio at
>=44100 stereo frames per host second. Stop immediately on runtime faults.

Future application: rapid original-synthesis rendering for automatic native
program design, including acoustic instruments. Preserve musical time and export
programs usable on hardware; this follows complete instrument acceptance.
