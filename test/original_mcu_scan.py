# SPDX-License-Identifier: GPL-3.0-only
"""Pin original MCU helper spans and check independently derived CPU/cycle results."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def original_mcu(firmware, reference):
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("Changed original 1.25 archive")
    midi, _ = load_input(firmware)
    messages, _ = read_smf(midi)
    main = decode_block(split_blocks(messages)[4])[1]
    _, offset = hc12_object(main)
    host = main[offset:offset + 32768]
    if hashlib.sha256(host).hexdigest() != reference["mcu_sha256"]:
        raise ValueError("Changed original MCU image")
    for row in reference["routines"]:
        start = row["entry"] - 0x8000
        data = host[start:start + row["bytes"]]
        if hashlib.sha256(data).hexdigest() != row["sha256"]:
            raise ValueError("Changed original delay/scan helper span")


def validate(text):
    boundaries = {0, 1, 2, 127, 128, 254, 255}
    patterns = {0x50, 0x75, 0xda, 0xff}
    entries = {0xb9c2, 0xb9ba, 0x8a42}
    expected = {(entry, value, flags) for entry in entries for value in range(256)
                for flags in range(256) if value in boundaries or flags in patterns}
    seen = set()
    instructions = cycles = writes = 0
    for line in text.splitlines():
        fields = line.split()
        if len(fields) != 13 or fields[0] != "CASE":
            raise ValueError("Invalid original MCU scan trace")
        entry, value, flags, pc, d, x, y, sp, ccr = map(lambda v: int(v, 16), fields[1:10])
        count, elapsed, stores = map(int, fields[10:])
        key = entry, value, flags
        if key not in expected or key in seen:
            raise ValueError("Missing, duplicate or out-of-matrix helper call")
        seen.add(key)
        n = value if value else 256
        if entry == 0xb9c2:
            endpoint, retired, clocks, ports = flags, n + 2, 3 * n + 9, 0
        elif entry == 0xb9ba:
            # DBNE leaves CCR intact after LDAB #255; Carry is preserved.
            endpoint = (flags & 0xf0) | 8 | (flags & 1)
            retired, clocks, ports = n + 261, 3 * n + 787, 0
        else:
            endpoint, retired, clocks, ports = flags & 0xf0, 6, 19, 3
        actual = pc, d, x, y, sp, ccr, count, elapsed, stores
        wanted = 0x1703, 0xa500 | (entry == 0x8a42), 0x369c, 0xc963, 0x1ff0, endpoint, retired, clocks, ports
        if actual != wanted:
            raise ValueError(f"Independent state/cycle mismatch: {key}: {actual} != {wanted}")
        instructions += count
        cycles += elapsed
        writes += stores
    if seen != expected or len(seen) != 8364:
        raise ValueError("Incomplete original MCU helper matrix")
    return {"cases": len(seen), "native_instructions": instructions,
            "core_cycles": cycles, "original_port_writes": writes}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original MCU scan helpers: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/mcu-scan-reference.json").read_text())
    original_mcu(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=160)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "runtime error:")):
        raise ValueError(f"Original MCU helper execution failed: {diagnostics}")
    totals = validate(trace.read_text())
    report.write_text(json.dumps({"passed": True, **totals,
                                  "scope": reference["qualification"],
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print(f"Verified {totals['cases']} original delay/scan calls and independent CPU/core-cycle "
          "expectations. No physical timing, full scan, interrupts or boot qualification.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
