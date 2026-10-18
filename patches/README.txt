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
constructors and makes BRN decode its displacement byte. It is guarded by the
original HCS_HC12.sinc SHA-256 recorded in doc/dependencies.json. It is applied
only to a build-directory copy, then compiled with Ghidra's native SLEIGH tool.
No manufacturer firmware bytes or processor-language source copies are shipped
in this repository. The specification retains Ghidra's Apache-2.0 license.
S12CPUV2 instruction definitions provide the independent behavior reference;
native_core covers 12288 bit-branch cases and BRN's fetch length.

HC12 indexed word correction
---------------------------
ghidra-hc12-indexed-word.patch fixes the HC12 indexed2_5 operand export. The
unpatched description exports an effective-address value or index register,
instead of a two-byte memory operand. LDD then loads the address, and STD can
overwrite the base register rather than memory. The patch restores the missing
dereference, after the branch patch in the same private build copy. The original
source hash remains the version gate; Ghidra's Apache-2.0 license still applies.

Independent regressions exercise 96 signed-offset LDD/STD pairs across X, Y and
SP, exact byte order/flags, unchanged base registers, IO access counts, and a
word postincrement. They fail against the uncorrected description. No vendor
program bytes are embedded in these vectors.

HC12 exception corrections
-------------------------
ghidra-hc12-exceptions.patch corrects SWI's indirect vector fetch, reads the
vector before stacking, and writes the B:A stack word in hardware byte order.
RTI restores B:A with a word transfer, preserving its documented operand width.
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
path; full ISA, interrupts/timing and real-device equivalence remain open.
