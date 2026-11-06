# Objective

Complete acceptance objective

Build a complete Blofeld Desktop and Keyboard instrument emulator with active SL sample
capability, initially headless and standalone. Unmodified original 1.25 firmware is the known-working
compatibility reference: tests verify and correct our emulator, not the
manufacturer's firmware. Emulate the MCU, DSP, ROM/RAM/
flash/NAND, clocks, interrupts, buses and required peripherals so that actual
reset, loading and initialization lead to normal instrument operation with
original DSP audio, bidirectional MIDI and complete persistent device state.
Correctness means equivalent behavior as an instrument, including sound, MIDI/
SysEx, voice management, musical timing, controls, display, persistence and update/
recovery. Identical electrical wiring, internal bus phases and silicon pipeline
timing are not acceptance requirements. Functional models of chip-internal boot,
peripheral state machines and coarser scheduling are allowed when their tested
contracts preserve those observable results. Execute the original Blofeld firmware
and DSP synthesis; firmware patches, skipped firmware routines, fabricated
instrument results and a host synthesizer do not satisfy this objective.
Prefer the simplest functional model that preserves instrument behavior. Refine
internal timing or wiring only when needed for firmware execution or observable
instrument effects; physical implementation detail alone does not justify it.
Reconstruct missing hardware properties independently from primary documentation,
actual firmware accesses, available PCB photographs and reproducible offline
experiments. Physical disassembly and measurement are unavailable and must not
be prerequisites. Investigate explicit hypotheses without a preset experiment
limit; retain their evidence, contradictions and visible unresolved accesses.
Missing hardware information is a research task, not a reason to abandon the
complete instrument objective.
The working original firmware may contain bugs, accidental accesses and tolerated
initialization sequences. Preserve their device-visible behavior. An executed
access constrains the model; it does not prove intent, implemented storage or
any particular undocumented feature. Documentation conflicts are research inputs,
not a reason to patch firmware or freeze one chip/decoder hypothesis. Compare
explicit alternatives across connected startup, operation and update paths;
advancing past one refusal alone does not establish correct instrument behavior.
Executing user-supplied firmware through the modeled boot/update paths is an
acceptance requirement. Developing a new instrument firmware remains separate
future work. Emulator results must identify supported behavior and unresolved
assumptions for compatibility assessment; simulation alone does not prove that
an image is safe on physical hardware. Component fixtures support full-device
acceptance and cannot replace it.

Future use case: faster-than-real-time offline rendering for automated native
Blofeld program design, including accurate acoustic-instrument approximation.
Run repeatable MIDI performances through original DSP synthesis, compare tone,
transients, decay and velocity response, then refine candidate programs through
normal instrument interfaces. Reuse deterministic starting states and export
programs usable by the physical instrument. Preserve emulated musical time;
rendering throughput is a host-performance property to measure after functional
instrument audio works. This later workflow does not replace initial instrument
acceptance or require development of alternative manufacturer firmware.

Current order: optimize CPU12 first to >=70% of matched optimized host-ASM
throughput on each representative warm fixture, then optimize and simplify
DSP56300 to the same threshold. Preserve equivalent inputs, ISA effects and
verified outputs; trivial cases and averages cannot establish this gate.
Defer further emulator integration, optimization and dependency-pin adoption
until both cores pass. Keep strict checks, short Release comparisons and focused
semantic validation; broaden validation as performance improves. Keep code small,
with complexity justified by correctness or measured speed.

Then complete the regular standalone/GUI path with the independent native cores,
removing foreign-core and transitional integration. Construct each core with
memory/peripherals and run to bounded event limits. Use direct storage and bounded
register/FIFO state for MCU/DSP communication; preserve peripheral side effects,
original DSP audio, MIDI and guest clocks without per-cycle replay. Core defects
belong in the independent libraries; application workarounds do not replace them.
Core benchmarks alone do not establish connected-instrument performance.

The complete instrument must sustain >=4x offline real-time throughput on this
host under representative high load; 10x is the stretch target. Preserve guest
time, every required audio frame and bounded live latency. Full Desktop/Keyboard
functionality, unmodified firmware and the acceptance scope remain required.
After the old backend is removed, replace transitional OwnDsp/own_dsp names in
classes, files, build targets, configuration and documentation with normal DSP
names. No permanent parallel implementation or backend-selection option is part
of the final device.

Performance and playable-instrument milestones

Milestone 1: whole-device real-time factor >= 1 and playable-instrument score >= 99%.
Milestone 2: whole-device real-time factor >= 10 and playable-instrument score >= 99.9%.
Milestone 3: whole-device real-time factor >= 100 and playable-instrument score >= 99.99%.
These scores concern the playable instrument: original DSP sound, MIDI/USB-MIDI,
voice behavior, musical timing, effects, controls/display, persistence and the
other documented instrument functions on the Desktop instrument. They do not measure
internal wiring fidelity, instruction-test pass rates or electrical similarity.
Each milestone requires an actually playable instrument; a boot screen or a
large set of passing component tests cannot substitute for that requirement.
Use a versioned instrument acceptance catalogue and published tolerances, fixed
weights/denominator and independent references. Keep untested instrument scope
visible; do not inflate the score by duplicating low-level cases or removing
failing features. These are intermediate milestones, not a waiver of complete
instrument acceptance. Internal wiring/timing detail is investigated only when
needed to preserve firmware execution and observable instrument behavior.

Measure real-time factor as emulated common-timebase seconds divided by
monotonic host seconds over a declared complete-device workload. Preserve guest
clock rates, MIDI timestamps and every required audio frame. Report warm-up,
host/build configuration, output mode, overload and the measured interval.
Faster offline processing does not run a physical audio device at 100x its rate.

Use the fastest validated optimized Release build for visual/audio bring-up,
long firmware experiments and performance measurements. ASan/UBSan and separate
TSan builds are for targeted tests and occasional broader regressions; do not
use an instrumented long run as the default development or throughput path.
Telemetry is a runtime choice, disabled by default. Prefer startup dispatch to
specialized policies/concepts so disabled telemetry adds no condition, capture,
formatting or allocation to instruction/audio hot paths. Sanitizers remain
separate compile-time build configurations. Required emulation, correctness
checks and contextual unsupported-access failures are not optional telemetry.

The current supplied acceptance scope includes Desktop and Keyboard. Keep profile
coverage explicit; the Desktop SL path remains the immediate integration target.
External instrument flow is MIDI <-> blofeld-emu -> audio, including USB-MIDI.
SL-capable factory storage and original sample behavior must be reconstructed;
no firmware patch or host-provided license-success response substitutes for it.
The independent CPU12/HC12 native JIT core is developed in the private cpu12
repository; board peripherals and original manufacturer programs stay here.

Acceptance requirements (current coverage catalogue: test/function-acceptance.tsv):
Instrument-visible results and musical timing, including USB, storage, update
and recovery behavior, are the accuracy criteria. Internal abstractions must
preserve those effects. Validate implementation contracts and every documented
instrument function; internal timing/decoder hypotheses are not extra acceptance
gates merely because they differ from a possible physical implementation.


1. Keep verified official firmware, manuals and the official standard sample
   library as local references. Track download URLs, sizes and hashes, without
   publishing proprietary assets. Establish required wavetables, internal ROM
   and external flash contents from actual firmware accesses; identify missing
   data explicitly rather than supplying guessed values.
2. Implement correct CPU/DSP execution, functional memory/peripherals for the Desktop SL instrument. Separate cores, board model, host transport and time-critical audio.
   Unknown/unsupported accesses fail with address, operation, PC and context;
   observed behavior, hypotheses and hardware-proven facts remain distinct.
   Reconstruct the behavior needed by the firmware from its accesses and available
   evidence. Keep simplifications and their instrument-level qualification explicit;
   obtaining a schematic is not a prerequisite for this work.
3. Provide reproducible tests for CPU/DSP instructions, flags/addressing/state,
   memory maps, interrupts, timers, interprocessor communication, cold start,
   reset, MIDI, voice management, audio and persistent storage. Independent
   expectations and recovered firmware behavior supplement round-trip checks.
4. Execute complete original update and recovery loaders. Test valid/damaged
   updates, interrupted transmission, power loss and restart during every erase
   and programming phase, checking the bootloader, reset vectors, flash
   protection and security bits. Component milestones do not prove whole-update
   recovery or physical-device flash safety.
5. Maintain automatic deterministic regressions with preserved regression inputs,
   long-run and real-time load tests. Maintain ASan/UBSan builds
   for project/native dependencies and separate TSan once concurrency exists;
   run targeted sanitizer tests and occasional broader regressions. Use optimized
   Release for visual/audio/long runs. Retain strict LLVM 19/C++23, warning,
   format and zero-finding analysis gates in every build configuration.
6. Provide reproducible execution traces and memory snapshots suitable for
   diagnosing failures and replaying firmware behavior. Document coverage,
   deviations, unresolved inputs and remaining uncertainty requirement by
   requirement; a passing subset does not complete the objective.
7. Establish expected behavior through independent ISA/peripheral models,
   primary specifications and constraints from the unchanged known-working
   firmware. Test hypotheses against connected reset-to-instrument execution
   and every documented function on both profiles. Physical comparisons
   are additional evidence if available, not an acceptance prerequisite. Do not
   claim measured hardware equivalence or physical flash safety from simulation.
   Physical programming would still require a separately established recovery
   procedure; no experimental flashing or malformed hardware traffic is allowed.
8. Expose the emulator as an actual host-visible USB-MIDI device, with original
   firmware descriptors/control behavior and bidirectional endpoint traffic.
   A standalone virtual ALSA port is not a substitute. Qualify host USB/MIDI
   enumeration and original Spectre under Wine, including SysEx/sample transfer,
   cancellation and errors through the emulator. Permit ordinary host MIDI
   routing, including PipeWire, from a connected keyboard to the virtual device.
   Qualify complete control/playback with standard Debian MIDI tools, including
   notes, controllers, program changes, SysEx and observable replies; no custom
   emulator CLI may substitute for external MIDI and original audio execution.
   Keep host transport separate
   from the board model and original firmware; never fabricate enumeration or
   transfer success to bypass unsupported peripheral/firmware behavior.

9. Provide a standalone normal runtime for one fixed Blofeld Desktop with active
   SL sample capability. There are no model-selection modes or profile switches. A separate end-user CLI
   is
   not part of the product; do not add a blofeld-cli companion. The running
   emulator communicates through MIDI and/or its actual host-visible USB-MIDI
   device, with the behavior of that fixed instrument. Send original DSP audio to
   the session's normal audio host
   and default output, as an ordinary audio application (for example PipeWire).
   Host audio transport is separate from emulated hardware; exclusive direct
   ALSA hardware access is not a requirement. Do not add a USB Audio function:
   the original Blofeld USB connection transports MIDI, not live audio.
   Existing reference/inspection/sample commands remain optional developer
   tools; their availability does not establish a working normal runtime.
   See runtime.txt for this interface contract and its current missing evidence.
10. Make the actual modeled LCD contents available for screenshots and live
    display in an independent GUI. Use a packed 1-bit live PBM image and text
    raster, with PNG for saved screenshots. Publish complete captures under
    $XDG_RUNTIME_DIR/blofeld-emu/ using atomic replacement and directory change
    notifications, so ordinary tools can read them without a blofeld-cli.
    Provide a small native GTK viewer that waits for available frames and
    follows updates and runtime-directory recreation automatically.
    Captures derive from the original firmware's LCD state, including changes
    caused by MIDI; unknown pixels, display modes and unqualified panel mapping
    remain explicit. Live GUI refresh does not establish
    hardware display timing or realtime audio/board equivalence.

Functional acceptance includes every documented Blofeld function on both
profiles, exercised through unmodified firmware with observable MIDI, audio,
display and persistent-state results. Track feature coverage against the manual
in test/function-acceptance.tsv;
isolated helper execution does not establish a complete instrument function.

Completion requires evidence for the whole Desktop and Keyboard emulator, original
MIDI/audio, full update/recovery behavior, reproducible failure/performance tests
and every documented function on both profiles. Physical access is not
a prerequisite for completion. The current component/reference fixtures remain
partial; the requirement catalogue records open scope without treating it as
successful boot, real-time operation or device equivalence.

Current execution budget

Until real-time instrument operation is established, suspend component,
functional, regression and diagnostic execution tests. Only short optimized
startup/throughput probes may establish that milestone. Stop at an absolute
10 host seconds and fail unless playable; never extend or automatically repeat
a failed probe. Strict compilation, formatting and static analysis remain active.
