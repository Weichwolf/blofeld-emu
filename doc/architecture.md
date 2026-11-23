# Target architecture

Execute unchanged Blofeld Desktop SL firmware as native host code. Firmware owns
instrument behavior; cpu12/dsp56300 supply ISA translation. The application owns
executable-image analysis, concrete device bindings and lifecycle.

The boundary to the cores is architectural. cpu12 and dsp56300 are generic
instruction-set translators usable by any device on those processors; they hold
no peripheral model and no Blofeld knowledge. This repository owns the device:
chip peripherals, board wiring, firmware knowledge (code inventory, entries,
variants, memory layouts), clocks, scheduling and the coupling of the two cores.
It reaches the cores only through their public interfaces and the IR it links
before optimization. If the instrument needs something from a core, the core
gains a general mechanism that a similar device could use as well.

```text
known MCU/DSP code + variants -> semantic LLVM IR --+
                                                   +-> joint LLVM 19 optimization -> native instrument
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

Complete AOT before starting execution owners. Build-time translation and linking
use the same semantic pipeline and native code-quality contract. Both
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

The product reserves a 64 MiB view for each P/X/Y read and write role. Each view
addresses the full 24-bit space in 32-bit host words. Real zero-initialized RAM
has shared physical backing across both roles and all five OMR layouts. Typed
volatile guest accesses preserve ordering across virtual
aliases; registers and flags remain ordinary SSA. Peripherals retain their
architectural decode and callbacks. P stores outside ordinary grants retain the
consumer's real loader/code-inventory effects. Native access faults terminate the
owner; only fault diagnostics and last completed-return progress are defined.
The consumer owns signal handlers; the escape stays below C++ execution scopes.

`BLOFELD_DSP_MEMORY=exact` selects the checked per-word reference; `direct` is the
product default. The direct views map ordinary P, X and Y RAM at their OMR
addresses, including shared storage for P aliases. Checked execution tracks
startup initialization per word where declared. The affected pages cover low
P `000000..000fff`, X `000000..002fff` and Y `000000..001fff`, high X from
`00c000` to `011fff` or `00ffff`, high Y from `00c000` to the word before the
OMR-selected end `00e000`, `010000`, `014000` or `016000`, and their P aliases.
The desktop profile lends no separate X/Y data ROM. Outside ordinary RAM, these
declared contents grant page-rounded reads:

| Contents | 4 KiB host page (DSP words) | 16 KiB host page (DSP words) |
|---|---|---|
| P:`ff0000..ff00bf`, main ROM | `ff0000..ff03ff` | `ff0000..ff0fff` |
| P:`ffff00..ffff50`, EEPROM ROM when present | `fffc00..ffffff` | `fff000..ffffff` |
| Y:`016000`, zero | `016000..0163ff` | `016000..016fff` |
| Y:`ffffff`, zero | `fffc00..ffffff` | `fff000..ffffff` |

In Checked mode the named Y words return zero through the device memory
contract. Adjacent undeclared words fault. Direct mode additionally permits
reads of ordinary zero padding in these Y pages. For example, Y:`016001` and
Y:`ffff7f` read zero from the rounded views on both listed host page sizes.
Those values describe the host mappings. Only Y:`016000` and Y:`ffffff` are
established guest zero reads.

X has no corresponding last-page read mapping. X:`ffff7f` therefore remains an
unmapped ordinary read. From `ffff80` onward, the X/Y peripheral decoder takes
priority over the view in both modes. The last X and Y write pages remain
protected. Valid registers keep their device behavior through the decoder.
The named Y:`ffffff` zero is resolved by the device memory contract;
padding grants no new register behavior. Discarded X/Y writes below
`ff0000` and discarded P writes at or above `ff0000` use separate scratch.
Their backing leaves the read views and real device state intact.

The checked reference must report no undeclared read in the qualifying
workloads. Code inventory and load completion stay separate from RAM's initial
contents. The original firmware performs loading.

Generated code follows the host build's Debug/Release policy. Debug supplies
assertions and diagnostic instrumentation. Release omits optional debug checks,
sanitizer work and telemetry. The current final pipeline uses O3. Select the
level per image by measuring whole-instrument throughput with exact PCM. Preserve
instrument behavior, including the specified page granularity for direct faults.
Apply selected sanitizer checks during compilation of our host and generated
code, and leave packaged libraries and runtime-generated platform helpers
uninstrumented. Verify IR and binding contracts during preparation. Include
mode and instrumentation in artifact identity and inspect emitted code to
qualify the Release path.

## Linking, bindings and artifacts

Core::lower returns owned unoptimized IR, guest/event exports and component identity
without a runtime Core, borrowed execution RAM or native engine. Compile the real
MCU/DSP adapter and device definitions into binding IR; no parallel implementation
or private core-layout copy. Public state/progress/yield methods join that IR when
needed. Compose both cores and eligible host definitions in host/aot/Code.

Before linking, validate scalar/aggregate types, packing, calling conventions,
ABI attributes, noexcept, parameter guarantees and memory effects. Structural
shape matters, not module-local type names. Linker attribute merging must not
hide incompatibility. Omit an argument noalias guarantee only with proof that
the local allocation has not escaped and cannot alias other arguments. Preserve
call-site return contracts and every declared guest/event root through optimization.

Native channel, wake and OS boundaries use explicit owner-scoped imports with
symbol, LLVM19 ABI and function address. Reject missing, duplicate, null, reserved
or incompatible imports and replacement of IR definitions. Use typed C adapters
for nonportable C++ method addresses. Keep the provider and channel state alive
through the final native operation and module destruction. Share the real native
wait/notify provider; do not duplicate its globals in generated modules.

Resolve native dependency cycles at symbol granularity without expanding every
local block's transitive closure. Give promoted local definitions collision-safe
hidden names and explicit materialization ownership; preserve original weak
definitions, relocations and strict missing/unexpected-symbol checks.

Resolve relocations and every export before publication. LLVM ORC's
GenericLLVMIRPlatform owns initialization/finalization; retain platform-before-
process symbol resolution, including __dso_handle/__cxa_atexit. Initialize/finalize
once; destroy resources before code release. Core::bind copies typed maps and
retains the shared native owner. Count shared native text once, not per variant
or component. Borrowed compiler inputs may expire after binding.

Development and operation may reuse fully validated code artifacts keyed by
source/variant hashes, core/LLVM pipeline, target/features, generation
mode and binding ABI. Store relocations, never instance pointers or device-state
snapshots. Reject corrupt/incompatible artifacts and rebuild before playback.
Keep derived source, IR, objects, catalogues and executables untracked in build/.
The semantic cache keys the inputs passed to Core::lower. Concrete device IR
that only joins during native composition belongs to the final cache key.
Use the build's aot-cache/ for pre-platform optimized bitcode. Validate current
inputs before lookup; key the composed module, ABI sets, target/features and
compiler/build pipeline. Verify artifact identity and payload before import.
Relocate, resolve exports and initialize fresh native instances through ORC.
Preserve original boot/device effects; caching changes startup, not warm speed.

### Local MCU catalogue, JSON format 1

Require `format: 1`, `inputs.mcu` (SHA256 of the original decoded 32 KiB MCU
image), `inputs.blobs` and explicit `cross_domain_entries: true`. Each optional
blob pins a unique name and full source SHA256; file loading additionally needs a
path relative to the catalogue. Reserve the source name `mcu` for decoded flash.

Each variant declares `id`, `source`, logical `address`, `byte_offset`, `bytes`
and exact `instructions: [[PC, length], ...]`. Translate every declared instruction
and prepare all cross-domain entry contracts before starting either owner. Reject
invalid extents, repeated IDs/starts, conflicting source and unresolved code.
Metadata describes executable source; it never initializes guest storage.

Admit only code matching initialized actual memory. Jointly mount compatible,
disjoint domains so normal calls and returns remain native. Protect exactly the
executable RAM bytes; data holes stay writable. Changed code, mapping and USB
buffer grants rebind at completed instruction boundaries. Unknown code fails;
execution never invokes preparation or discovers new variants. Immutable native
programs may be shared; RAM, device state and code lifetime belong to each VM.

### Local DSP catalogue, JSON format 1

`inputs` pins six decoded blocks, retained NAND program records and separate
boot-main/boot-eeprom regions by lowercase SHA256. Hash ROM as original big-endian
24-bit bytes, without filling address gaps. Each variant declares id, ordered
writes, code contexts [P address, word count], entries, stopped resumes and guards
{address, value}; null denotes an actually unreadable word. Optional mode/mode_mask
constrains OMR admission.

A variant can declare `guard_sets` in place of `guards` when several exact
program states admit the same code view. Each alternative is a nonempty array
of word guards. All guards in one alternative must match. Every alternative
must be disjoint from the predicates of other variants at a shared resume PC.

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
exceptions. Transitions {from, to, entry} declare prepared handoffs. The entry
field accepts an address or a nonempty list of addresses for the same pair.
A shared entry may name several target variants; reject duplicate triples. Select a unique
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

Relate nominal core progress to 60 MHz instrument units, retaining fractional
clock phase and exact instruction crossings. Distinguish speculative MCU progress,
attained DSP progress and committed instrument effects. Qualify final MCU instructions, base cycles, RAM and device state at the same
confirmed guest boundary. Report discarded and replayed execution separately; a
host-triggered stop alone is not a deterministic guest boundary.
Host scheduling and window
length change processing time only. Neither advances a guest clock or changes
event ordering. Define ordering at equal timestamps, including real IRQ masks,
priorities, shadows, WAI/STOP/RTI and REP state.

Run the MCU ahead in bounded 19200-tick windows while the DSP consumes its
timestamped output. At each window start, save complete MCU core
state through the common core contract and consumer-owned RAM, device, clock and
pending-event state. Retain ordered host inputs for exact replay; never resample
an external input while replaying. Prepared native code is immutable and shared.
Checkpoints retain pending declared code transfers through the core Snapshot,
including their original image ownership and destination variants. Restore an
unchanged validated binding in place; rebind only after code or mapping revocation.
An incomplete device snapshot is an explicit failure.

Validate DSP-driven MCU inputs against attained DSP history:

- Non-destructive reads of peer data, shared pins, status and interrupt lines use
  the last known DSP output with the actual driver, direction and knownness rules.
  Record each read's time, signal and observed mask/value in a bounded read set.
  Include direction and knownness for the accessed pins. Full-byte and
  read-modify-write accesses retain all read bits. These reads do not wait for
  the DSP and do not create a new DSP session.
- At attained DSP time, confirm and discard the matching read-set prefix. A DSP
  output change at time t invalidates speculation if a read at or after t used a
  different value, or if an interrupt-relevant edge could have interrupted the MCU.
  Stop the DSP before it consumes the invalid MCU suffix, restore the MCU window
  and replay to the exact receiving boundary. Admit the real input and regenerate
  the suffix. Preserve already committed effects without publishing them twice.
  Publish confirmed MCU time as a monotonic frontier after successful epoch
  validation. Keep progress outside the device event log. MIDI input admission
  uses confirmed time; speculative execution and receiver consumption preserve it.
  Refill streamed MIDI knowledge on explicit receiver demand independently of
  confirmed progress, so an unconfirmed epoch can reach its confirmation boundary.
- Destructive reads, including reply-byte consumption and FIFO pops, wait for
  attained DSP time. Publish preceding source transactions before waiting, end
  the speculative window and commit the validated prefix. Perform the real read
  once at its admitted instruction boundary and establish the next checkpoint
  after it. Never predict consumption or replay it against an advanced DSP.

No DSP state rollback is required. With no changed DSP outputs, repeated panel
reads validate without synchronizing the owners. A quiet output trace alone is
not proof that no reads occurred; report the read set by signal.

Stage MCU peer-data, SPI, MIDI output, LCD and storage effects with their actual
timestamps. Release observable effects only through attained DSP time and only
from validated MCU history. Discard invalid speculative effects on rollback.
Retained input history and staged outputs use bounded storage; full buffers stop
the producer at an instruction boundary. A blocked synchronous read must not
prevent the DSP from consuming the preceding MCU history it needs to reach it.

The DSP applies incoming events at their exact receiving boundaries. Changes that
can affect IRQ eligibility or an actual input read bound native execution. Plain
data changes may be applied at the next read if every intervening event and the
value read are preserved. SS assertion/release, including DDR-B changes, follows
the same receiver-boundary rule as other shared outputs. No peer GPIO value or
readiness may become visible early. Local MCU work creates no per-access
rendezvous, CFG-prefix query or joint permission window.

Use release/acquire publication for timestamped history and completion frontiers.
Wake only for actual input, required progress, space, stop or reset; recheck the
predicate after registering a waiter. Stop/reset cancels waits without advancing
time. Genuine request/reply dependencies remain serial. Preserve FIFO and
clear-on-read effects; audio block size establishes no communication horizon.

Qualification compares complete original PCM for W1 (held-note polyphony) and
W2 (SL samples) against the corrected non-speculative controls, with at least two
different window lengths and changed owner scheduling. Report startup and active
throughput separately, rollback count, reads and failed validations by signal,
DSP sessions per frame and bounded storage use. Keep one product coupling path;
the cores execute prepared images without scheduler-specific queries.

Asserted DSP reset immediately disconnects its outputs and pending notification.
Reset stops the old execution owner before retiring its buffered history and
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
  entries at or before attained DSP time and discard invalid speculative suffixes.
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
An optional host output owner translates completed DIN bytes into MIDI 1.0
messages and schedules them against the native audio epoch. A disconnected
destination stops host delivery while the owner continues to retire queued bytes.

Streaming input publishes an exclusive known-until frontier. Silence emits no
character; source delay adds no device time. Stop at the last authorized native
edge rather than rounding a frontier past a possible input. Publish attained
progress before waiting. Bounded source/output owners drain replies and honor
failure/cancellation without either receiver cursor blocking the other forever.

USB endpoints borrow selected RAM and knowledge arrays. Bank-selection effects
publish grants after the selecting instruction; unrelated register writes do not
remap buffers. SETUP acknowledgement clears the interrupt flag while retaining
the received bytes for the original firmware's subsequent read. Endpoint-zero
IN exposes the primed RAM until host acknowledgement, and OUT owns its received
RAM until firmware clears DVALID. Preserve word-commit order and configuration quirks.
Automatic SET_CONFIGURATION raises SETECR and holds status IN at NAK until the
firmware writes DONEECRU. Its status packet belongs to the controller.
Local bulk endpoints resolve logical addresses and direction through the active
configuration. The firmware's NISOF bits remain readable and have no effect on
full-speed bulk routing. Control and bulk share the same buffer ownership rules.
Enumeration, SOF and completions come from real host transactions. Expose original
USB-MIDI descriptors/endpoints, never USB Audio or synthesized success.

Audio originates solely in the final original DSP output. Preserve complete
ordered signed 24-bit stereo frames, sample times and device status. Frame
completion edges use the common instrument clock and actual PLL segments.
Split native work at relevant MIDI/device events. Offline drains at maximum
throughput; live host consumption supplies pacing/backpressure outside callbacks.
Query source/native rates and actual conversion; never assume macOS fixes 48 kHz.

Establish readiness through the original application WAI, valid PCM and genuine
DIN identity exchange before choosing the musical origin. Discard boot audio
and host input predating firmware readiness. The DSP owner stages host MIDI in
DIN order, then advances the shared exclusive input frontier using granted PCM
queue credit. An empty MIDI queue advances the same frontier. Admit late or
uncorrelated packets at the earliest legal tick and count them separately.
Report native faults and underruns.

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

Linux uses one JACK client with two audio outputs and bidirectional MIDI.
The callback drains a preallocated stereo queue and converts signed 24-bit
samples to exact float32. It copies incoming MIDI to a bounded queue with the
frame offset from the current JACK cycle and publishes the completed input
frontier, even for an empty cycle. A fixed frame lead covers the PCM queue and
the guest block. Convert JACK frames to instrument ticks once at admission and
serialize the bytes through the original DIN interface. Output uses completed
firmware messages and JACK frame offsets. The server's MIDI port bounds the
maximum size of a single SysEx event. Device faults and underruns terminate
the live stream. The callback never waits for PCM or MIDI.

On macOS, use Core Audio device observations to time MIDI.
Publish coherent bounded-read correlation snapshots: source frame range/index,
native epoch, monotonic time and uncertainty. Retain enough accepted original frame
completion times for the granted maximum lookback; never interpolate guest time
from nominal frame rate. Bind VI and native epoch IDs explicitly. A new audio
epoch invalidates prior MIDI correlation even before a new observation arrives.
Check the granted maximum uncertainty at priming and leave admitted events at
their original guest ticks.
The total live window must fit the physical latency limit in OBJECTIVE.md.
Render/submission/server-consumption points are not analog presentation timestamps.

CoreMIDI converts selected-source MIDI1 UMP/SysEx7 into exact DIN bytes with
message boundaries and native time. Retain open SysEx across callbacks; only
realtime bytes interleave. Reject malformed/backwards/missing timestamps and
unsupported protocols. Keep the first owned CFRunLoop/client alive, dispose
callbacks before storage, and detect selected-source disappearance independently
of unrelated endpoint changes.

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
transfers. After a clean live shutdown, join the execution owners and atomically
replace each changed image from a synced staged file. Reject changes to the
backing file since startup. Reset/update coordinates owner stop, channel
retirement, image publication and fresh code bindings. Physical flashing requires
an independently established recovery path.

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
- Apple CoreAudio/CoreMIDI/audio-workgroup and JACK contracts:
  host scheduling, timestamps, buffer grants and lifecycle.

Host API references:
[CoreAudio timestamps](https://developer.apple.com/documentation/coreaudiotypes/audiotimestamp),
[render callback](https://developer.apple.com/documentation/audiotoolbox/aurendercallback),
[audio workgroups](https://developer.apple.com/documentation/audiotoolbox/understanding-audio-workgroups),
[CoreMIDI input](https://developer.apple.com/documentation/coremidi/midireceiveblock),
[JACK audio callback](https://jackaudio.org/api/group__ClientCallbacks.html),
[JACK MIDI](https://jackaudio.org/api/group__MIDIAPI.html).
