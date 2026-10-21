# SPDX-License-Identifier: GPL-3.0-only
"""Pin unmodified MCU/IRQ/submitter bytes before conditional native MIDI tests."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, log = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original USB-MIDI producer: local firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/usb-midi-producer-reference.json").read_text())
    original_mcu(firmware, reference)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())],
                            check=False, capture_output=True, text=True, timeout=50)
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stdout + result.stderr)
        return result.returncode
    rows = [tuple(map(int, row)) for row in re.findall(
        r"^CASE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", result.stdout, re.M)]
    expected = [(events, cursor, ccr, replay, (events + 15) // 16 + 1)
                for events in (0, 1, 15, 16, 17, 31) for cursor in (0, 124)
                for ccr in (64, 197, 0) for replay in range(3)]
    if [row[:-1] for row in rows] != expected or any(row[-1] <= 0 for row in rows):
        raise ValueError("Incomplete original IRQ/batch/wrap/mask/restore matrix")
    summary = "SUMMARY cases=36 replays=72 own-peripheral-configuration=1 original-boot=0"
    if result.stdout.splitlines()[-1:] != [summary]:
        raise ValueError("Missing exact conditional producer scope")
    log.with_suffix(".json").write_text(json.dumps({
        "passed": True, "cases": 36, "restores": 72,
        "native_instructions": sum(row[-1] for row in rows),
        "scope": "Original MIDI IN IRQ and submitter under own declared USB/RAM inputs; "
                 "not original initialization, enumeration, timing or physical equivalence."
    }, indent=2) + "\n")
    print("Verified conditional original USB-MIDI batching, native IRQ/RTI/X clear and restores.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
