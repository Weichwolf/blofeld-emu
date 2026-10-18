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
