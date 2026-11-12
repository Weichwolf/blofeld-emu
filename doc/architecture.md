# Target architecture

Execute unchanged Blofeld Desktop SL firmware as native host code. Firmware owns
instrument behavior; cpu12/dsp56300 supply ISA translation. The application owns
executable-image analysis, concrete device bindings and lifecycle.

Core repositories are organizational boundaries. Choose APIs, state ownership,
layouts and LLVM composition for complete-instrument throughput and efficiency.

```text
known MCU/DSP code + variants -> semantic LLVM IR --+
                                                   +-> joint LLVM 19 O3 -> native instrument
concrete host bindings --------> compatible IR -----+

                         VmContext
                    /                 \
             MCU execution         DSP execution
             local state           local state
                    \-- timed bounded channels --/
                                           |
                                  original PCM -> live audio / WAV
```

`src/vi/` owns instrument contracts, `src/host/` reusable transports and OS
bindings, `src/app/` composition and UI. Host code knows no instrument registers;
VI code owns no host API handles. Each core owns its state and has one execution
owner. VmContext binds cores, stable memory and channels without state mirrors,
a universal core hierarchy, electrical simulation or a global instruction clock.
Linux x86-64 and macOS ARM64 share firmware, bootstrap, images and settings.
Platform differences belong to host bindings and native target lowering.

## Compilation and executable coverage

Prepare the complete known executable firmware and variants before playback.
Include loader outputs, overlays, instruction templates, indirect/callback
entries, IRQ vectors and retained fetch/cache forms. Declare instruction streams
and boundaries separately from readable tables or padding. A saved return
address can point to data. Keep overlapping starts independent and validate
complete instruction extents, including operands replaced by ordered writes.
Never combine incompatible source versions into one active code view.

Preparation consumes immutable source and binding descriptions. It executes no
guest instructions, device transactions or musical time. Unknown input data
remains runtime data; it does not prevent translation. Resolve indirect transfers
to prepared native targets. Incomplete executable coverage fails with code
identity, address and reason; optimizing only a hot subset does not qualify.

Prefer startup firmware loading followed by full AOT. Build-time translation and
linking use the same semantic pipeline and native code-quality contract. Both
cores export detached LLVM bitcode or relocatable objects; consumers never copy
private ISA lowering or use public LLVM handles. Preparation has no fixed time
budget. Measure compilation, original boot/readiness and warm execution separately.

Preserve genuine loader/program writes and observable program reads. Known
writes select coherent prepared variants. Actual executable changes revoke
dependent bindings before changed code runs; loading known code into RAM alone
does not require first-use translation. No decoding, code emission, initialization
or symbol lookup belongs on an admitted playback path.

Build natural CFG/SSA regions and connected functions, independent of execution
quotas, transport fragments and catalogue entry density. Preserve every declared
entry without making each instruction a function boundary. Link known successors
directly; indirect transfers use prepared targets. Keep the host stack bounded
over native cycles and guest values live across instructions and successors.

Optimize constants/ranges, redundant computations and loads/stores, register
values, loop invariants and induction variables. Apply strength reduction,
unrolling or vectorization only with valid dependency/range proofs. DSP 56-bit
arithmetic, rounding, saturation, raw accumulator fields and required flags lower
inline to native integer operations, without arithmetic or per-opcode helper
calls. Preserve fixed-width wrap, sticky flags, parallel old operands and aliases.
Materialize state at actual fault, I/O, IRQ, code-change and partial-grant exits.

Optional consumer translation rules are pinned to firmware SHA256, code variant,
instruction region, entry/exit contract and binding ABI. A hash identifies a
candidate; it does not prove equivalence. Preserve complete state, ordered effects,
faults and intermediate event boundaries. Use the prepared original path when
a rule's proved conditions do not hold; never replay an already performed effect.
Candidates and references belong in firmware-fusion-candidates.json.

## Validation, Debug and Release

Validate code coverage, source identity, ABI, target/features, layouts, rights
and provable storage/knowledge aliases during AOT and stopped installation.
Revalidate when a relevant code, memory or device revision changes. Expose these
facts to LLVM; Release must not repeat established invariants per instruction or
short block. Fully initialized, stably mapped RAM uses direct loads/stores without
per-word initialization checks. Removing a check requires a proof covering every
reachable use and every operation that can invalidate it.

Dynamic addresses, MMIO, conditional initialization and observable faults retain
their exact semantics unless analysis proves their outcome. Keep unavoidable
selection and cold failure paths small. OS page protection does not establish
DSP address-space validity, initialization or device behavior. Range proofs must
also respect real storage aliases; non-overlap does not establish distinct
outermost allocations for LLVM noalias guarantees.

Generated code follows the host build's Debug/Release policy. Debug supplies
assertions and diagnostic instrumentation; Release uses joint LLVM O3 without
optional debug checks, sanitizer work or telemetry. Required instrument semantics
and fault boundaries are identical. Sanitizers insert runtime checks during
compilation; apply selected instrumentation to our host/generated code, never
packaged foreign libraries or runtime-generated platform helpers. Verify IR and
binding contracts during preparation. Include mode/instrumentation in artifact
identity and inspect emitted code to qualify the Release path.

## Linking, bindings and artifacts

Core::lower returns owned pre-O3 IR, guest/event exports and component identity
without a runtime Core, borrowed execution RAM or native engine. Compile the real
MCU/DSP adapter and device definitions into binding IR; no parallel implementation
or private core-layout copy. Public state/progress/yield methods join that IR when
needed. Compose both cores and eligible host definitions in host/aot/Code.

Before linking, validate scalar/aggregate types, packing, calling conventions,
ABI attributes, noexcept, parameter guarantees and memory effects. Structural
shape matters, not module-local type names. Linker attribute merging must not
hide incompatibility. Omit an argument noalias guarantee only with proof that
the local allocation has not escaped and cannot alias other arguments. Preserve
call-site return contracts and every declared guest/event root through O3.

Native channel, wake and OS boundaries use explicit owner-scoped imports with
symbol, LLVM19 ABI and function address. Reject missing, duplicate, null, reserved
or incompatible imports and replacement of IR definitions. Use typed C adapters
for nonportable C++ method addresses. Keep the provider and channel state alive
through the final native operation and module destruction. Share the real native
wait/notify provider; do not duplicate its globals in generated modules.

Resolve relocations and every export before publication. LLVM ORC's
GenericLLVMIRPlatform owns initialization/finalization; retain platform-before-
process symbol resolution, including __dso_handle/__cxa_atexit. Initialize/finalize
once; destroy resources before code release. Core::bind copies typed maps and
retains the shared native owner. Count shared native text once, not per variant
or component. Borrowed compiler inputs may expire after binding.

Development always runs complete AOT. Operation may reuse validated artifacts
keyed by source/variant hashes, core/LLVM pipeline, target/features, generation
mode and binding ABI. Store relocations, never instance pointers or device-state
snapshots. Reject corrupt/incompatible artifacts and rebuild before playback.
Keep derived source, IR, objects, catalogues and executables untracked in build/.
Preserve original boot/device effects; caching changes startup, not warm speed.

### Local DSP catalogue, JSON format 1

`inputs` pins six decoded blocks, retained NAND program records and separate
boot-main/boot-eeprom regions by lowercase SHA256. Hash ROM as original big-endian
24-bit bytes, without filling address gaps. Each variant declares id, ordered
writes, code contexts [P address, word count], entries, stopped resumes and guards
{address, value}; null denotes an actually unreadable word. Optional mode/mode_mask
constrains OMR admission.

Writes construct immutable source views, never guest state or loading effects:

| Kind | Source fields |
| --- | --- |
| boot-main / boot-eeprom | word_offset, words, destination address |
| block | block, byte_offset, words, address |
| blob | source name, byte_offset, words, address |
| records | Original count prefix; logical P payloads only |
| copy | Earlier source at from, words, destination address |
| field | from, address, 24-bit mask, shift 0..23; ((word & mask) << shift) & $ffffff |

Optional `inputs.blobs` entries bind {name, sha256} to complete additional source
bytes, including bytes outside selected code slices. The file loader also requires
`path`, resolved relative to the catalogue. Pure preparation receives named byte
spans instead. Names must be unique and supplied/declared sets equal. Blob writes
construct source views only; actual loading still executes the firmware protocol.

Optional memory regions declare {space, first, words, writable, tracked,
aliases_program} for X/Y independently per variant. An omitted contract retains
dynamic grants; an explicit empty list requires zero grants. Fixed layouts enable
direct RAM/loop optimization while cold loader variants retain dynamic mappings.
Defaults are writable, untracked, not program-aliased.
Interrupts {vector, level} cover levels 0..3, including unmaskable software
exceptions. Transitions {from, to, entry} declare prepared handoffs. A shared entry
may name several target variants; reject duplicate triples. Select a unique
admitted target only after actual source words/layout match, retaining the
completed real jump and pending transfer on refusal.
failed_fetches describe actual unreadability, never translation gaps.
Installation validates prepared words against actual guest-produced P contents.
Resume PCs may be explicit entries or starts of nonempty declared code streams;
operand words are not resume points. Entry hints seed native region optimization,
not code coverage: every instruction in every declared stream is translated.
Entry admission must be disjoint: compatible mode/word/memory predicates cannot
share an installation PC across variants. Reject overlaps with both IDs and the
PC before lowering.
The source owner establishes complete code/entry/variant closure; matching hashes
or a translated subset alone does not establish it.

## Independent execution and causality

Run each owner in long native sessions. Couple through real command/data
transactions, audioframes, bounded buffers, reset and faults. Local MCU RAM,
register or LCD work does not synchronize or advance the DSP. Use supported host
scheduling to favor performance resources for execution and efficiency resources
for bounded preparation/background work. Query topology; QoS is not proof of
P/E affinity. Host placement never changes guest time or ISA semantics.

ExecutionTime relates completed native progress to nominal 60 MHz instrument
units. Distinguish completed time from an exclusive channel bound B: no unconsumed
producer event may precede B. Publish effects before the bound covering them.
Receivers must not pass a possible earlier input; late timestamps cannot repair
causality. Derive channel-specific lookahead from prepared CFG/bindings, real
device latency and waiting states. Unknown control flow or addresses end a proof.

Include IRQ paths, actual masks/priorities/shadows and REP/return state in each
bound. Derive incoming-to-outgoing delays as well as autonomous effects. Specify
same-time ordering and atomic instruction crossings. Prove progress through
cyclic dependencies, zero-delay paths, reset epochs and unequal clocks; mirrored
completed-frontier waits can deadlock. Never advance a clock to advertise channel
knowledge or revoke a bound after a legal input arrives.

Batch bounded queue publication with release/acquire committed prefixes and
individual transaction timestamps. An earlier-visible write cannot wait for a
later block's completion. Wake only for requested horizons, data or space; recheck
predicates after registering waiters. Stop/reset cancels waits without advancing
time. Short adaptive spins may precede OS waits. Genuine request/reply dependencies
remain serial; overlap independent work rather than repeatedly waking both owners.

Summarize polling only over proved stable intervals, retaining elapsed nominal
time, timeout, registers/flags, observable reads and IRQ boundaries. FIFO and
clear-on-read accesses require their actual effects. Audio block size establishes
neither a communication horizon nor an IRQ-latency guarantee.

Reset stops the old execution owner before retiring its channel prefix and
acknowledging the epoch. Retain MCU-owned inputs during asserted reset, then
publish them at release without discarding newer data. DSP reset recreates its
Core/code bindings, resets SHI and retains SRAM. Old work cannot publish into the
new epoch. VmContext joins owners and channel consumers before destroying state.

## Instrument device contracts

Use direct memory and functional register/FIFO/DMA/IRQ transactions. Decode
firmware bit-banging locally; no electrical wiring or per-bit host rendezvous.
Unknown accesses fail with address, operation, PC and context. Preserve original
status, ordering, partial transfers and cancellation; never invent readiness,
payloads, acknowledgements or success replies.

- MCU Port Q and DSP Port C retain raw low-byte values, direction and unknown
  bits. J0 supplies configured IRQA; E9 supplies level-sensitive XIRQ. Preserve
  eligible notification generations, mask transitions and source timestamps.
  Interrupt admission does not consume a still-driven command or clear a level.
  Commands 13/14 retain their actual 16-byte response transaction until consumed,
  withdrawn or reset. Repeated bytes remain distinct writes; no future byte is
  readable early. Raw reads outside the decoded transfer retain raw semantics.
- MCU-owned data/direction changes use a bounded SPSC timeline. Apply only
  entries at or before attained DSP time. Local panel sampling can retain an
  admitted peer frontier only until a relevant GPIO/I/O/owner boundary changes it.
  Keep released-input bias distinct from active unknown drivers. Port C's
  documented pull-downs, inactive B3/B5..B7 samples and the E3/E2 package route
  are explicit VM input contracts, not recovered response bytes or PCB evidence.
- PM3 supplies the raw pedal change; original firmware performs normalization and
  polarity. Do not duplicate it as MIDI. The MCU owner samples host controls.
- SHI uses its bounded receive FIFO and distinct HTX/IOSR stages. Preserve CPHA
  loading, overwrite/underflow/error behavior. Stage one completed MCU SPI word;
  outside native callbacks wait for actual receive-mode publication and FIFO
  capacity. HRX consumption releases space; reset/configuration/stop cancel waits.
- LCD transactions retain selection, read-holder/dummy-read and address-advance
  semantics. E4 delimits transactions even after DDRE release; B7/B6 select, B5
  reads, B0 data. Lifecycle supplies reset. Unknown RAM stays unknown. Map columns
  0..127 and page7/bit7 logical top row to the panel.
- MCU nominal clocking uses the 12 MHz reference and 60 MHz PHY profile. Preserve
  detector availability, oscillator-driven RTI, timer capture/compare and pending
  vector order. OC7M/OC7D scratch values remain readable; unsupported compare-output
  activation fails regardless of register-write order. Initialized executable RAM
  overlays receive immutable grants; changed bytes revoke dependent bindings.
- DSP P/X/Y grants share actual physical banks. MS/MSW and OMR changes rebind
  windows without payload copies; preserve the documented three-cycle exclusion.
  Reserved writes are discarded under the explicit VM map. Reserved Y:$16000 and
  Y:$FFFFFF read as zero under the selected manufacturer-model contract; other
  unknown reads fail. These choices create no RAM windows or physical-board claim.
- DSP PCTL supplies its raw nominal clock ratio, with reset PINIT=1. EXTAL,
  HCKT_1 and ACI share the nominal 11.2896 MHz VM reference. Retain fractional phase
  through clock changes and older pending events; do not infer physical PLL lock.
- Boot ROM maps 192 words at P:$FF0000 and optionally 81 EEPROM words at P:$FFFF00.
  Borrow those spans separately; do not fill gaps. The SPI prefix has 55 words;
  the main block's 48 readable data/fill words are not instruction contexts.
  Document-derived inputs establish no physical mask-ROM identity.
- DMA consumes real requests or DE triggers in priority order. Commit counters,
  addresses, reloads and completion only after destination acceptance. Program
  writes publish the changed revision before native work resumes.
- ESAI_1 consumes actual word/slot events and requests DMA refill. Preserve port
  assignment, personal reset, formats, underruns and pending empty requests.
  Publish original TX0 stereo once. DAX retains buffer/refill/IRQ effects without
  a second host stream or biphase encoding.
- DSP timers advance analytically from CLK/2 progress. Preserve load/reload,
  compare/overflow, W1C and TE continuity. Media readiness supplies Timer1 capture.
  Unspecified TLR/TCPR values use explicit VM zeroes retained across RESET.
- Arbitrate source priority, SR masks and VBA through real native vector entry.
  Acknowledge accepted requests only. Masked IRQs permit long native runs; mask
  changes, REP, shadows and returns retain precise admission. WAIT may advance
  device time analytically without manufacturing retired instructions.

## MIDI, audio and musical time

DIN uses one bounded bidirectional port with ordered 31,250-baud 8N1 characters
in instrument time. Two independent cursors feed MCU SCI activity and the DSP's
original Timer0 decoder. Supply scheduled input-level boundaries to Timer0, not
decoded ring writes. Preserve back-to-back timing, SCI clear sequences, overrun,
wakeup and genuine transmit-completion timestamps. USB remains a separate route.

Streaming input publishes an exclusive known-until frontier. Silence emits no
character; source delay adds no device time. Stop at the last authorized native
edge rather than rounding a frontier past a possible input. Publish attained
progress before waiting. Bounded source/output owners drain replies and honor
failure/cancellation without either receiver cursor blocking the other forever.

USB endpoints borrow selected RAM and knowledge arrays. Bank-selection effects
publish grants after the selecting instruction; unrelated register writes do not
remap buffers. Preserve word-commit order and firmware configuration quirks.
Enumeration, SOF and completions come from real host transactions. Expose original
USB-MIDI descriptors/endpoints, never USB Audio or synthesized success.

Audio originates solely in the final original DSP output. Preserve complete
ordered signed 24-bit stereo frames, sample times and device status. Frame
completion edges use the common instrument clock and actual PLL segments.
Split native work at relevant MIDI/device events. Offline drains at maximum
throughput; live host consumption supplies pacing/backpressure outside callbacks.
Query source/native rates and actual conversion; never assume macOS fixes 48 kHz.

Establish readiness through the original application WAI, valid PCM and genuine
DIN identity exchange before choosing the musical origin. Early inquiries must
not overwrite startup mailbox state. Discard boot audio. Live composition primes
a fresh native output epoch, obtains audio correlation, then accepts new host
events; input predating readiness is discarded. Map only retained original audio
intervals. Missing clocks, late input, native faults and underruns fail the session.

SMF reading merges tracks by pulse and source-track order with bounded cursors
and payloads. Format2 requires explicit pattern selection. Retain rational time
across tempo changes and time-code rates; round up only at final clock admission.
Keep meta events off the wire, preserve SysEx/escape bytes and reject cross-track
SysEx collisions. Truncated/changed files cannot finish cleanly. EOF retains the
input barrier until the serialized end plus release/effect tail is published.

Offline composition opens no host audio/MIDI input or viewer. Validate continuous
frame indices, rate, epochs and completion times before writing WAV; reject
underruns or drift beyond half a frame plus integer quantization. Clip the selected
half-open musical interval, stop/join producers and publish only a complete RIFF
file. Never replace an existing output or publish cancelled/failed output.

## Host transport and live correlation

Callbacks never allocate, lock, wait, throw or access files. Fixed SPSC queues
transfer host input and complete PCM. Keep borrowed spans valid until synchronous
consumption completes. Report overflow, disconnect, underrun and actual grants;
bounded queues never silently lose events or frames.

Darwin uses CoreAudio, monotonic sample/host timestamps and an execution-owner
audio workgroup/deadline based on the granted hardware quantum. Close that context
on its owner before thread exit, including unwinding. Reject invalid, repeated or
discontinuous render intervals before consuming queued PCM. Reset establishes a
new timeline; a failed stream must reopen. Join callbacks before state teardown.

Linux uses a dedicated PulseAudio owner, actual server format/buffer grants,
priming before uncork and cork/flush on reset. Backpressure stays outside native
callbacks. Explicit asynchronous timing requests supply raw consumption indices
within measured send/receipt bounds; reject corrupt/backwards/unsubmitted indices,
stopped consumption and excessive uncertainty. No interpolated server clock is
substituted for an actual observation.

Publish coherent bounded-read correlation snapshots: source frame range/index,
native epoch, monotonic time and uncertainty. Retain enough accepted original frame
completion times for the granted maximum lookback; never interpolate guest time
from nominal frame rate. Bind VI and native epoch IDs explicitly. Reset/failure
revokes old observations. Grant maximum uncertainty before priming and include
both margins in MIDI lead/silence admission; queued events are never retimed.
The total live window must fit the physical latency limit in OBJECTIVE.md.
Render/submission/server-consumption points are not analog presentation timestamps.

CoreMIDI converts selected-source MIDI1 UMP/SysEx7 into exact DIN bytes with
message boundaries and native time. Retain open SysEx across callbacks; only
realtime bytes interleave. Reject malformed/backwards/missing timestamps and
unsupported protocols. Keep the first owned CFRunLoop/client alive, dispose
callbacks before storage, and detect selected-source disappearance independently
of unrelated endpoint changes.

Linux binds ALSA sequencer UAPI directly: prefer MIDI1 UMP, otherwise adapt legacy
events with group0. Bound kernel and decoded queues. A dedicated owner records
CLOCK_MONOTONIC receipt points and handles poll cancellation/disconnect. Do not
reinterpret foreign queue-relative timestamps as monotonic hardware arrival.

## Display and persistent state

The MCU owner supplies complete LCD snapshots through a bounded latest-state
mailbox; the control owner renders/publishes them. Live composition alone binds
the viewer and atomically writes display.pbm in the standard user runtime
directory. Publish only changed known pixels; unknown snapshots revoke the bitmap.
No text dump, screenshot/download control or display-path override is required.

A live-only display.live FIFO retains an open publisher descriptor without data
writes. The viewer checks presence once per second, clears stale images after
exit/crash and follows directory recreation. A known live image hides the status
line. Publication does no firmware work or audio-callback work.

Versioned device/NAND images retain identity, programs, samples and explicit
knowledge/history. Storage commands decode into bounded operations and direct
page-index access, not cell physics or payload mirrors. Hold read bytes until
release; an early strobe cannot create valid data. Complete actual image work
before publishing ready/success. Reset replaces the operation epoch and prevents
stale completion. Nominal VM read/reset durations are 64/32 DSP reference periods;
program/erase use about 200 us/2 ms, without a silicon-timing claim.

Preserve protection, interrupted-write known zeros and untouched contents; mark
possibly changed bits/history unknown. Serialize checksums outside ordinary
transfers and qualify host durability separately. Reset/update coordinates owner
stop, channel retirement, image publication and fresh code bindings. Physical
flashing requires an independently established recovery path.

## Qualification and primary references

OBJECTIVE.md defines complete acceptance; ACCEPTANCE.tsv records platform results.
Use the normal instrument for integration, with identical firmware, bootstrap,
programs, samples, settings and timestamped workloads on both platforms. Verify
original behavior, complete PCM and host-scheduling-independent musical time.
Measure startup separately and count valid stereo frames once. Compare authored
core algorithms with equivalent Clang19 O3 C++ under the same numeric/target
contract; microbenchmarks do not establish instrument throughput.
RTF = valid stereo frames / (source frames per second * active host seconds).
Exclude the union of overlapping compilation intervals once; do not double-count
parallel preparation. Readiness and physical latency retain separate measurements.

Primary identities and source hashes remain in runtime-references.json,
hardware/sources.json, function-references.json, downloads.json,
midi-references.json and the remaining indexes in README.txt. Apply relevant
designs without importing a foreign core or changing reference checkouts:

- Gearmulator: native synthesizer firmware and audio/device coupling.
- N64Recomp/Zelda64Recomp and XenonRecomp/UnleashedRecomp: static translation,
  known overlays and consumer runtime integration.
- Remill: exact ISA semantics lifted to LLVM with consumer-owned memory bindings.
- Wasmtime/WAVM: stable contexts/memory, native boundaries and artifact ownership.
- LLVM19 LLJIT: native initialization, relocation and resource lifetime.
- Misra, Distributed Discrete-Event Simulation, sections3-4: channel bounds,
  deadlock and demand-driven progress.
- Apple CoreAudio/CoreMIDI/audio-workgroup, ALSA UAPI and PulseAudio contracts:
  host scheduling, timestamps, buffer grants and lifecycle.

Host API references:
[CoreAudio timestamps](https://developer.apple.com/documentation/coreaudiotypes/audiotimestamp),
[render callback](https://developer.apple.com/documentation/audiotoolbox/aurendercallback),
[audio workgroups](https://developer.apple.com/documentation/audiotoolbox/understanding-audio-workgroups),
[CoreMIDI input](https://developer.apple.com/documentation/coremidi/midireceiveblock),
[ALSA sequencer](https://www.alsa-project.org/alsa-doc/alsa-lib/seq.html),
[ALSA UAPI](https://github.com/torvalds/linux/blob/v6.12/include/uapi/sound/asequencer.h),
[PulseAudio timing](https://www.freedesktop.org/software/pulseaudio/doxygen/structpa__timing__info.html).
