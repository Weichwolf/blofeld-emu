# SPDX-License-Identifier: GPL-3.0-only
"""Pin the original firmware and its conditional native Board bootstrap ledger."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(text, reference):
    expected = (
        f"Original Board POR pc=0x8200 retired={reference['retired']} "
        f"bus-cycles={reference['bus_cycles']} common-ticks={reference['shared_ticks']} "
        f"operations={reference['retired'] + 195} DSP-held=1 DSP-cycles=0\n"
        "Conditional original MCU bootstrap through actual Board POR; no full instrument claim\n")
    if text != expected:
        raise ValueError("Missing or changed original Board POR/native bootstrap ledger")


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original Board POR: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/mcu-power-on-reference.json").read_text())
    original_mcu(firmware, reference)
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())],
                            check=False, capture_output=True, text=True, timeout=110)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.with_suffix(".log").write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr:
        raise ValueError(f"Original Board POR failed: {result.stdout}{result.stderr}")
    validate(result.stdout, reference)
    original_mcu(firmware, reference)
    if hashlib.sha256(binary.read_bytes()).hexdigest() != binary_hash:
        raise ValueError("Original Board POR binary changed during execution")
    report.write_text(json.dumps({"passed": True, "binary_sha256": binary_hash,
                                 "reference_qualification": reference["qualification"],
                                 "complete_instrument_boot": False}, indent=2) + "\n")
    print("Verified original MCU bootstrap through Board POR; DSP held, laboratory inputs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
