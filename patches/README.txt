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
original HCS_HC12.sinc SHA-256 recorded in cmake/dependencies.json. It is applied
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
result or installed upstream source is substituted. See the corresponding independent tests under test/.

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

PulseAudio17 client callback types
The pinned build copy receives pulseaudio-callback-types.patch with exact context
matching. Property destruction now accepts void*, matching pa_free_cb_t, instead
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

HCS08 software interrupt
------------------------
ghidra-hcs08-swi.patch dereferences FFFC/FFFD instead of branching to FFFC.
It changes only the hash-pinned generated language copy; upstream Apache-2.0
notices and installed SDK/firmware bytes remain intact. Native execution owns
stack/branch/SP effects; the memory owner resolves actual vector reads.
See the corresponding independent tests under test/ for independent tests and timing limits.

ghidra-hcs08-decimal.patch replaces unresolved DAA intrinsics with native
p-code for Family Rev2 Table A-2. Both corrections use original A/H/C;the
owner rejects unspecified inputs and tracks undefined V. The pinned build
copy retains Ghidra Apache-2.0 terms;installed SDK/firmware stay unchanged.
Independent decimal/table/snapshot tests:the corresponding independent tests under test/.

ghidra-hc12-masked-read.patch routes only BRCLR/BRSET byte observations through
an explicit selected-bit intrinsic. Effective-address subtables retain native
address updates and avoid an extra data read. Native predicates/CCR/PC and the
ordinary whole-byte paths remain; see the corresponding independent tests under test/.
