# SPDX-License-Identifier: GPL-3.0-only
"""Check original SPI loading and the current explicit unresolved-memory frontier.

This supporting reference test does not establish authentic ROM identity, physical
clock calibration, complete original boot or normal instrument operation.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROM_SHA256 = "4d72a5bd75ad086ec7190c6c25b4438aa1cfaa2ddc0ac510055c2540e886de56"
PERIODS = 9777
HEADER = ("Clocked document ROM + original94word frame;explicit between-period SPI stimulus;"
          "no MCU scheduling/physical phase claim")
LOADED = "ORIGINAL_STAGE_LOADED all94Pwords_exact=1"
ENTERED = "ORIGINAL_STAGE_ENTERED all94Pwords_exact=1"
CONTEXT = "sent=96 entered=1 loaded=1 cycles=9777 pc=0x000072"
FAULT = ("DSP fetch completed-cycles=9777 DMA-index=9777 ticks=153035 "
         "pc=0x000072 omr=0x600389: DSP bus-cycle preflight pc=0x000072 omr=0x600389: "
         "Native memory E1 write X:0x002000 and write Y:0x002000 pc=0x000072 "
         "omr=0x600389: Unmapped DSP memory write at Y:0x002000")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(returncode, diagnostics, text):
    require(returncode == 1, "Original runtime must still expose the unresolved memory failure")
    require(diagnostics.splitlines() == [
        "Original clocked first-stage frontier:" + CONTEXT + " " + FAULT, FAULT],
        "Unexpected native/sanitizer diagnostic; inspect the complete retained output")
    rows = text.splitlines()
    require(rows and rows[0] == HEADER and rows[-1] == "FRONTIER " + CONTEXT + " error=" + FAULT,
            "Missing original reference scope or strict final memory diagnostic")
    require(rows.count(LOADED) == 1 and rows.count(ENTERED) == 1 and
            rows.index(LOADED) < rows.index(ENTERED), "Original SPI load must precede actual entry")
    require(rows[rows.index(ENTERED) - 1] == "1572 pc=0x00006a retired=0xff0035",
            "The genuine native loader JMP must retire into the unchanged received stage")
    cycles = [row for row in rows[1:-1] if row not in (LOADED, ENTERED)]
    require(len(cycles) == PERIODS, "Original reference pipeline period count changed")
    pattern = r"([1-9][0-9]*) pc=0x[0-9a-f]{6}(?: decode=0x[0-9a-f]{6})?(?: retired=0x[0-9a-f]{6})?"
    for expected, row in enumerate(cycles, 1):
        parsed = re.fullmatch(pattern, row)
        require(parsed is not None and int(parsed[1]) == expected,
                "Native period rows must be complete, ordered and structurally valid")


def check_inputs(firmware, rom, root):
    if not firmware.is_file() or not rom.is_file():
        print("SKIP: local original1.25 firmware or document-derived192word ROM absent")
        return False
    manifest = json.loads((root / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    require(hashlib.sha256(firmware.read_bytes()).hexdigest() == expected,
            "Original firmware ZIP identity mismatch")
    data = rom.read_bytes()
    require(len(data) == 576 and hashlib.sha256(data).hexdigest() == ROM_SHA256,
            "Document-derived main ROM identity mismatch; this is not an authentic chip dump")
    return True


def main():
    binary, firmware, rom, trace = map(Path, sys.argv[1:])
    if not check_inputs(firmware, rom, Path(__file__).resolve().parents[1]):
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=trace.stem + "-", suffix=".trace", dir=trace.parent)
    os.close(descriptor)
    actual = Path(temporary)
    # A failed run retains its unique trace for inspection; no older trace can satisfy this run.
    result = subprocess.run([str(binary.resolve()), str(firmware), str(rom), str(actual)],
                            capture_output=True, text=True, check=False)
    diagnostics = result.stdout + result.stderr
    print(diagnostics, end="")
    validate(result.returncode, diagnostics, actual.read_text())
    actual.replace(trace)
    print("Original94-word SPI frame and strict Y:002000 frontier verified;full instrument boot remains open")
    return 0


if __name__ == "__main__":
    sys.exit(main())
