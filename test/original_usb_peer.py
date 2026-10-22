# SPDX-License-Identifier: GPL-3.0-only
"""Pin original USB/peer paths and check native IRQ, GPIO and failure accounting."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def expectations():
    # Full native IRQ/strobe/failure counts plus captured IRQ/strobe prefix.
    lengths = (0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1)
    result = []
    for length in lengths:
        result += [(1, length, 0, int(length != 0), 0)] * 4
    for length in (6, 3, 3, 5, 48, 0, 0):
        result += [(1, length, 0, int(length != 0), 0)] * 4
    result += [(2, 6, 0, 1, 0)] * 2
    for length in (0, 3, 3, 3, 6, 6, 96, 0, 0):
        result += [(0, length, 0, 0, 0)] * 4
    for length in (1, 1, 2, 3, 8):
        result += [(0, length, 0, 0, 0)] * 3
    for length in (3, 3, 3, 3, 2, 2, 3):
        result += [(0, length, 0, 0, 0)] * 2
    result += [(1, 0, 1, 1, 0), (0, 0, 1, 0, 0), (0, 0, 1, 0, 0)]
    result += [(1, length, 1, 1, length) for length in range(3)]
    result += [(1, 0, 0, 0, 0)] * 4  # Actual zero-byte OUT, no forwarding call.
    assert len(result) == 169
    return result


def validate(text):
    rows = [tuple(map(int, row)) for row in re.findall(
        r"^CASE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if [row[:2] for row in rows] != [(case, attempt) for case in range(169) for attempt in range(3)]:
        raise ValueError("Incomplete original OUT/header/gate/raw-count/peer/failure matrix")
    expected = expectations()
    for case, _, instructions, irq, strobes, failure in rows:
        if instructions <= 0 or (irq, strobes, failure) != expected[case][:3]:
            raise ValueError(f"Incorrect independent original IRQ/strobe/failure counts: {case}")
    prefixes = [tuple(map(int, row)) for row in re.findall(
        r"^PREFIX (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if [row[0] for row in prefixes] != list(range(169)):
        raise ValueError("Missing native checkpoint prefix")
    for case, instructions, irq, strobes in prefixes:
        needed = expected[case][1] != 0 or expected[case][2] != 0
        if (irq, strobes) != expected[case][3:] or (instructions > 0) != needed:
            raise ValueError("Incorrect captured native prefix or restored event recounting")
    summary = "SUMMARY cases=169 restores=338 own-gpio-endpoints=1 original-boot=0 physical-peer=0"
    if text.splitlines()[-1:] != [summary]:
        raise ValueError("Missing exact conditional GPIO/USB scope")
    return {"cases": 169, "restores": 338,
            "original_instructions_excluding_replayed_prefixes": sum(row[2] for row in rows) -
                                                                 2 * sum(row[1] for row in prefixes),
            "original_usb_irq_entries": sum(row[3] for row in rows) - 2 * sum(row[2] for row in prefixes),
            "original_peer_strobes": sum(row[4] for row in rows) - 2 * sum(row[3] for row in prefixes),
            "expected_fault_cases": 6, "expected_fault_replays": 18, "stopped_retry_attempts": 18}


def main():
    binary, firmware, log = map(Path, sys.argv[1:])
    report = log.with_suffix(".json")
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original USB/peer paths: local firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/usb-peer-reference.json").read_text())
    original_mcu(firmware, reference)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())], capture_output=True,
                            text=True, check=False, timeout=160)
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stdout + result.stderr)
        return result.returncode
    report.write_text(json.dumps({"passed": True, **validate(result.stdout), "scope": reference["qualification"],
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified original OUT/peer GPIO execution, actual strobes, native faults and joint restores.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
