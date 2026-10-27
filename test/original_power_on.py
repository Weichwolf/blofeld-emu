# SPDX-License-Identifier: GPL-3.0-only
"""Pin the original image and collect its conditional bootstrap from held POR."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(text, reference):
    expected = [
        f"original-POR suffix={suffix} retired={reference['retired']} "
        f"bus-cycles={reference['bus_cycles']} MCU-ticks={reference['mcu_ticks']} "
        f"shared-ticks={reference['shared_ticks']}"
        for suffix in range(2)
    ]
    expected.append(
        "Conditional original MCU bootstrap from POR; full instrument/DSP ROM remains open")
    if text.splitlines() != expected:
        raise ValueError("Missing or changed original POR bootstrap/native clock/replay evidence")
    return {"bootstrap_suffixes": 2, "retired_per_suffix": reference["retired"],
            "bus_cycles_per_suffix": reference["bus_cycles"],
            "mcu_ticks_per_suffix": reference["mcu_ticks"],
            "shared_ticks_per_suffix": reference["shared_ticks"]}


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original POR: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/mcu-power-on-reference.json").read_text())
    original_mcu(firmware, reference)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())], check=False,
                            capture_output=True, text=True, timeout=110)
    report.parent.mkdir(parents=True, exist_ok=True)
    diagnostics = result.stdout + result.stderr
    report.with_suffix(".log").write_text(diagnostics)
    if result.returncode or result.stderr or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer",
            "ThreadSanitizer", "runtime error:")):
        raise ValueError(f"Original POR execution failed: {diagnostics}")
    totals = validate(result.stdout, reference)
    original_mcu(firmware, reference)
    if hashlib.sha256(binary.read_bytes()).hexdigest() != digest:
        raise ValueError("Original POR binary changed during execution")
    report.write_text(json.dumps({"passed": True, **totals, "binary_sha256": digest,
                                 "scope": reference["qualification"],
                                 "complete_instrument_boot": False,
                                 "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified two original MCU bootstrap suffixes from held POR with a held DSP; "
          "laboratory inputs, full instrument boot remains open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
