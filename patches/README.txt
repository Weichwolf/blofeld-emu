Ghidra native SLEIGH integration patch

The byte-backed hardware bus uses MemoryBank wordsize=1 to avoid touching
neighboring registers. Upstream MemoryBank::setValue/getValue assume transfers
span at most two natural words. A 16-bit store merges stale low-byte bits;
wider reads can calculate negative shift counts. Partial stores can also perform
reads of IO registers, which is not a permitted substitute for a CPU store.

ghidra-byte-memory.patch handles wordsize=1 byte by byte, preserves endianness,
and avoids synthetic read-before-write accesses. Other banks remain unchanged.
It applies to a copy under build/, never to the installed Ghidra sources.
Original native source notices and its Apache-2.0 license remain in force.

Original memstate.cc SHA-256:
ef68ee18511cac1bec55a602c64e43f57bc4755ca00e8204f411434d51d27f92

Regressions: native_core stack/call overwrite and byte-transfer side-effect
checks. This patch does not establish completeness of the SLEIGH instruction
semantics or correctness of other native memory-bank configurations.

HC12 branch corrections
----------------------
ghidra-hc12-branches.patch corrects the BRSET predicate in all three operand
constructors, makes BRN decode its displacement byte, and fixes inverted BGE/
LBGE predicates (N equals V). It is guarded by the
original HCS_HC12.sinc SHA-256 recorded in doc/dependencies.json. It is applied
only to a build-directory copy, then compiled with Ghidra's native SLEIGH tool.
No manufacturer firmware bytes or processor-language source copies are shipped
in this repository. The specification retains Ghidra's Apache-2.0 license.
S12CPUV2 instruction definitions provide the independent behavior reference;
native_core covers 12288 bit-branch cases and BRN's fetch length;
hc12_cycle_extended independently checks 1024 short/long branch/CCR/alignment
cases against literal truth masks.

HC12 indexed word correction
---------------------------
ghidra-hc12-indexed-word.patch fixes the HC12 indexed2_5 operand export. The
unpatched description exports an effective-address value or index register,
instead of a two-byte memory operand. LDD then loads the address, and STD can
overwrite the base register rather than memory. The patch restores the missing
dereference and groups the signed nine-bit PC displacement before adding it to
the next instruction address, after the branch patch in the same private copy.
The original source hash remains the version gate; Ghidra's Apache-2.0 license still applies.

Independent regressions exercise 96 signed-offset LDD/STD pairs across X, Y and
SP, exact byte order/flags, unchanged base registers, IO access counts, and a
word postincrement. Additional LEA/LDD/STD vectors check positive/negative
nine-bit PC-relative addresses, operand widths and manual cycle counts. They
fail against the uncorrected description. No vendor program bytes are embedded in these vectors.

HC12 arithmetic corrections
---------------------------
ghidra-hc12-arithmetic.patch follows the exception patch under the same pinned
source hash and Apache-2.0 terms. EDIV/EDIVS preserve D/Y on zero and overflow;
signed division compares against the correct negative lower bound. Divide-zero
guards precede the native divide operation. IDIV and FDIV produce the documented
FFFF quotient on zero/overflow as applicable; IDIVS preserves D/X on zero.
Undefined flags/remainders remain outside assertions and hardware claims.
DAA computes accumulator and Carry from the same original A/C/H operands. The
native adapter implements its two declared intrinsics using S12CPUV2's nine-row
valid-input table; invalid combinations fail visibly with PC/operands.

hc12_cycle_extended executes literal normal/boundary/overflow/zero vectors with
manual cycle counts and 20000 actual ADCA/DAA pairs against independent decimal
addition and defined-flag expectations. These are project-owned deterministic
instruction inputs; manufacturer program bytes and disassembly remain local.

The same patch restores three-byte immediate/extended CPS constructors. Upstream
encoded them as one-byte skips, reading instruction operands as program-memory
data and concealing those bytes from execution-permission checks. The immediate
form uses its instruction operand directly; extended CPS performs one word data
read. The unused skip helper is removed. S12CPUV2 page 158 specifies the lengths,
flags and cycles. hc12_compare_stack checks 4608 independent literal cases across
all eight address modes, preserved registers/memory, overlapping alternative
entries and unknown operand-byte fetches. No installed Ghidra source is modified.

HC12 min/max corrections
-----------------------
ghidra-hc12-minmax.patch follows the CCR patch in the private build copy, under
the same pinned upstream hash and Apache-2.0 terms. All eight unsigned min/max
constructors read their operand once, calculate width-correct NZVC from the
original subtraction and then select the destination. Memory forms always write
the selected value, including unchanged data. The old wider-result V/N tests,
repeated word reads and conditional write omissions are removed.

hc12_minmax supplies exhaustive byte pairs, word boundaries, literal HCS12 bus
slots, changing-input/failure controls and genuine GPIO writes/snapshot replay.
The native timing adapter now supports all five index families. Prior-decoder
controls fail without a diagnostic baseline; no hardware behavior, firmware
result or installed upstream source is substituted. See doc/hc12-minmax.txt.

HC12 exception corrections
-------------------------
ghidra-hc12-exceptions.patch corrects SWI's indirect vector fetch, reads the
vector before stacking, and writes the B:A stack word in hardware byte order.
RTI restores B:A with a word transfer, preserving its documented operand width.
Its native read order is corrected to CCR, B:A, X, return PC, then Y, with the
specified SP adjustments. S12CPUV2 page 270 and S12CPU15UG page 289 agree on this
order. hc12_cycles checks literal read addresses, widths, final registers and
normal/chained cycle counts; the final nine-byte frame layout is unchanged.
WAI also saves B:A with one word operand; the original two byte stores violate
its NXP operand-width definition. Native mcu_wai tests independently check four
word stores and one CCR byte, literal frame/order, idle and wake/RTI behavior;
they fail with the original WAI description.
It also fixes the reversed STOP-disable condition. Enabled STOP remains an
unsupported clock-suspension operation; disabled STOP must be a two-byte NOP.
The patch follows the branch and indexed-word patches under the same source
hash gate and Ghidra Apache-2.0 terms. No installed distribution is modified.

Independent mcu_interrupt tests fail against the uncorrected SWI constructor
and verify the corrected handler address, stack access order and STOP gate.
memory_usb independently checks the IRQ save/RTI restore operand widths and fails
against the original two-byte-load RTI description.

DSP56300 reference adapter
-------------------------
dsp56300-reference-bus.patch applies only to the revision pinned in
cmake/DspCore.cmake and doc/dependencies.json. GNU patch applies this zero-context
diff against that exact revision. It adds unconditional strict bus hooks, custom
peripheral dispatch, exception propagation, exact extension checks
and a bounded reference entry for DO/positive REP. Program stores do not synthesize
old-target reads. REP retains its latched opcode/extension but invalidates the
body cache for later normal execution. Undefined decoded instructions fail in
reference mode. The normal upstream interpreter remains separately callable;
this integration forces the reference interpreter, not the JIT.

The same build-local patch converts asmjit's ARM vector element mask explicitly
to uint32_t before combining it with BaseReg's enum mask. Numeric register
signatures are preserved under C++23, without suppressing mixed-enum diagnostics.
This compatibility correction does not enable or validate a JIT backend.

The build-local AsmJit CPU detector uses std::call_once and a shared once flag.
Its build disables thread-safe local statics, so an explicit once operation is
required. Upstream's shared initialized flag and global
copy race when independent DSP instances are constructed concurrently, even
with the interpreter forced. Detection operations and returned values are
unchanged; no test prewarm or sanitizer suppression is used. dsp_omr_thread
runs two cold owners through actual OMR checkpoint suffixes under TSan.
AsmJit's original Zlib notices remain in the copied source. The pinned original
cpuinfo.cpp SHA-256 is recorded in doc/dependencies.json.

UBSan caught reset using out-of-range CCR enum values for full SR masks. Reset
and interrupt entry now use the correct SR-mask overloads. No sanitizer is
suppressed. Independent native_dsp checks cover flags, unknown fetch/extension,
P writes/mutation, retained map aliases, bounded DO/SPI waits and REP latching.
The original_dsp_boot fixture executes documented partial NXP SPI ROM plus the
unaltered MCU-embedded Blofeld first stage; all reference bytes remain private.

Reference bit instructions optionally delegate GPIO accesses to a symbolic bit
hook. Only the selected bit must be known; untouched output latches keep their
knowledge state. Ordinary peripheral fallback performs one read and at most one
write, preserving FIFO side effects. EA decoding/postincrement occurs once.
BTST and bit branches share the checked read path. Native tests cover Carry,
mixed GPIO states, pp/qq/EA and RAM aa forms, FIFO consumption and bit boundaries.
These hooks do not establish bus-phase, interrupt or pad timing.
The board adapter also handles timer TCSR bits, retaining full writable-word
writeback and its W1C effects while ignoring the read-only DI field on writes.
No additional SDK change is needed for that timer-specific board behavior.

The SDK checkout stays unchanged. CMake stages and patches private build copies,
then updates only changed output files so a small patch edit does not rewrite
all active native headers. This is a limited, uncalibrated DSP56371 reference
path; full ISA, complete interrupt sources/timing and real-device equivalence
remain open.

The bounded native interrupt entry uses source-selected priority rather than the
SDK pending FIFO and its fixed level. Actual vector fetches use the strict bus;
native JSR in either vector slot saves PC/SR, clears long-entry state and raises
IPL, allowing only higher-level nesting. Native RTI restores the frame. Fast
vectors retain foreground PC and may change SR. Native MOVEC masks read-only VBA
bits; active/consecutive REP defers entry. Unsupported vector flows/lengths and
stack overflow fail. Independent dsp_interrupts checks and reference limits are
described in doc/dsp-interrupts.txt. No JIT or physical pipeline claim is made.

NORM's no-shift path now refreshes E/U/N/Z/V as specified by DSP56300FM 13-146.
The SDK previously preserved stale flags indefinitely after a data MOVE. A
three-NORM regression fails against that path; 4608 independently computed
vectors check scaling, A/B, Rn, compatibility and preserved state. Pinned external
MCPX hardware corpus banks b0/b1/b2 provide a separate instruction-state comparison
(doc/dsp-validation.txt). This does not qualify the JIT or a Blofeld board.

Whole-accumulator transfer helpers now update sticky S from the pre-move source
before scaling/limiting. Raw partial transfers bypass that update. Register bit
operations and accumulator REP sources use their standard S/L transfer rules.
Independent dsp_transfers checks include parallel read-before-replace behavior,
raw parts, MOVEC/MOVEM/MOVEP, SR replacement and unselected growth. Native VSL
now stores raw MSP/shifted LSP with one EA update and no CCR changes; its opcode
metadata permits an EA extension and rejects immediate data. dsp_vsl covers
addressing/scaling/flag vectors, modulo REP, strict faults and preservation.
In reference mode, unimplemented instructions throw their name before coreDump
can obscure the cause with unrelated bus reads. These changes do not implement
cache instructions or establish DSP56371 silicon/pipeline equivalence. See
doc/dsp-validation.txt for scope and remaining derivative-specific loop questions.

Normal zero REP now annuls valid one-word targets in reference mode, preserving
count-source effects, LC and flags, with strict target/compatibility/placement
failures. The interpretation uses SR.SC on FM5-13 and annulled REP timing A.2.6.6,
corroborated by MCPX b2. Zero DO now performs native frame setup/unwind so popped
stack slots retain LA/LC/body-PC/SR writes even though its body does not execute.
The old early return omitted those writes. Own failure regressions, nested state,
IRQ chains, boundaries and all71 pinned b2 cases check the corrections. Normal
interpreter/JIT paths are unchanged; full chip/pipeline/compatibility qualification
remains open. See doc/dsp-zero-loops.txt. The original nonzero clear loop is not
shortened, skipped or allowed to write unqualified reserved memory.

Reference TRAP/ILLEGAL, DEBUG, STOP and WAIT now reject their unqualified exception,
debug or clock state before the SDK can issue a host debug break or silently
continue. Conditional TRAP/DEBUG check their condition first: all sixteen literal
encodings have true/false flag vectors with PC/CCR/SP and selected RAM checks in
dsp_instruction_fixture. These guards do not implement software exception entry,
OnCE or suspended clocks; see doc/dsp-instructions.txt.

An earlier own instruction input exposed an ASR carry-bit shift exponent of 70
under UBSan. The reference helper now rejects counts above FM 13-16's defined
55/40 limits (normal/sixteen-bit arithmetic), before any C++ shift. ASL counts
above 55 are also unqualified and reject. The saved raw input and deterministic
boundary tests remain project-owned regression data; no sanitizer is suppressed
and no physical result for an undefined shift is invented.

The native reference checkpoint API captures registers/AGU state, yielded REP
latches, interrupt continuation, diagnostic counters and peripheral pacing.
Deferred CCR updates are canonicalized before capture. Restore rebuilds opcode
dispatch while retaining cached diagnostic cycle costs. REP opcode/extension
latches remain independent of later backing-program changes. JIT/debugger/external-bus and SDK interrupt
queue/callback contexts reject. DspMachine pairs this trusted native state with
an immutable strict-bus copy; no portable SDK object dump is exposed. See
doc/dsp-checkpoints.txt for independent instruction and original-coupling checks.

PulseAudio17 client callback types
The pinned build copy receives pulseaudio-callback-types.patch with zero patch
fuzz. Property destruction now accepts void*, matching pa_free_cb_t, instead
of calling a typed-pointer function through an incompatible function pointer.
JSON frees and dispatch-owned operation references use void* adapter functions
that call the original typed functions. The first sanitized live stream close
exposed the property callback UB; host_audio preserves lifecycle and cancelled
operation regressions. All sanitizer checks remain active. Extended unused
client APIs and JSON parsing are not covered merely by this live adapter test.

HCS08 arithmetic and signed branch corrections
--------------------------------------------
ghidra-hcs08-arithmetic.patch applies only to the hash-pinned build copy under
the Ghidra Apache-2.0 terms. BGE now branches on N XOR V == 0; upstream used
BLT's condition (Family Rev2 p212). Ordinary step() checks640 independent BGE/BLT
cases; the separate captured-queue gate checks native ppp branch/refill behavior.
Installed Ghidra source and firmware bytes stay unchanged.

Selected-source accumulator growth
---------------------------------
The reference data-bus S-growth offset is S0-S1, as defined by FM5-13:
high bit 46 normally, 47 when scaling down and 45 when scaling up. The old
offset reversed the scaled cases. Sticky S, the selected source, numeric
scaling/clipping and original notices remain intact. Own full-state streams
and unchanged vendor-model windows distinguish these cases independently;
the older own transfer oracle is corrected from the same primary equations.
The patch uses one zero-context diff per affected upstream file and produces
the qualified native source bytes. No JIT/physical timing claim is implied.
See doc/dsp-accumulator-transfers.txt and its reference manifest.

HCS08 software interrupt
------------------------
ghidra-hcs08-swi.patch dereferences FFFC/FFFD instead of branching to FFFC.
It changes only the hash-pinned generated language copy; upstream Apache-2.0
notices and installed SDK/firmware bytes remain intact. Native execution owns
stack/branch/SP effects; the memory owner resolves actual vector reads.
See doc/hcs08-swi.txt for independent tests and timing limits.

Native virtual-memory information initialization
----------------------------------------------
The retained cold-owner TSan regression exposed concurrent writers to AsmJit's
shared VirtMem::info cache. An atomic ready flag did not give initialization
exclusive ownership. Explicit std::call_once now publishes the complete native
OS-derived information once, retaining -fno-threadsafe-statics and all sanitizer
checks. Existing CPUInfo synchronization and original notices remain intact.
The independent dsp_vm_info gate starts sixteen cold callers together and checks
OS page size/alignment and 32768 immutable reads; dsp_repeat_clock_thread also
runs actual concurrent DSP execution and trusted checkpoint continuations.

ghidra-hcs08-decimal.patch replaces unresolved DAA intrinsics with native
p-code for Family Rev2 Table A-2. Both corrections use original A/H/C;the
owner rejects unspecified inputs and tracks undefined V. The pinned build
copy retains Ghidra Apache-2.0 terms;installed SDK/firmware stay unchanged.
Independent decimal/table/snapshot tests:doc/hcs08-decimal.txt.
