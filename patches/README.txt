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
