# SPDX-License-Identifier: GPL-3.0-only
"""Verify pinned original dispatcher execution and independent native route counts."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def expectations():
    # USB IRQ, SCI IRQ, USB calls, UART calls, own caller instructions.
    ordinary = [(0, 0, 0, 0, 0), (0, 1, 0, 1, 0), (1, 0, 1, 0, 0),
                (0, 1, 0, 1, 0), (0, 0, 0, 0, 0), (0, 0, 1, 0, 0), (0, 1, 0, 1, 0)]
    result = [row for _ in range(7) for row in ordinary for _ in range(2)]
    result += [ordinary[index] for index in range(4)]
    result += [(1, 1, 1, 1, 2)] * 42
    routes = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 1), (1, 1), (1, 1)]
    for chunks, deferred in [(1, 1), (1, 0), (1, 0), (2, 1), (2, 1), (1, 0)]:
        for index, (uart, usb) in enumerate(routes):
            interrupts = usb * (deferred if index == 6 else chunks)
            result += [(interrupts, uart, chunks * usb, chunks * uart, 0)] * 2
    result += [(1, 1, 1, 1, 2)] * 14
    assert len(result) == 242
    return result


def validate(text):
    rows = [tuple(map(int, row)) for row in re.findall(
        r"^CASE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if [row[:2] for row in rows] != [(case, attempt) for case in range(242) for attempt in range(3)]:
        raise ValueError("Incomplete ordinary/chunk/native-caller/restore matrix")
    expected = expectations()
    for case, _, instructions, *counts in rows:
        if instructions <= 0 or tuple(counts) != expected[case]:
            raise ValueError(f"Incorrect independent original dispatcher/IRQ/caller counts: {case}")
    prefixes = [tuple(map(int, row)) for row in re.findall(
        r"^PREFIX (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if [row[0] for row in prefixes] != list(range(242)):
        raise ValueError("Missing captured native-prefix accounting")
    for case, instructions, usb, sci in prefixes:
        if usb or sci != int(case >= 228) or (instructions > 0) != bool(sum(expected[case][2:4])):
            raise ValueError("Incorrect retained prefix or restored interrupt recounting")
    if text.splitlines()[-1:] != ["SUMMARY cases=242 restores=484 own-caller=1 original-boot=0"]:
        raise ValueError("Missing exact own-caller/conditional native scope")
    return {"cases": 242, "restores": 484,
            "original_instructions_excluding_replayed_prefixes": sum(row[2] for row in rows) -
                                                                 2 * sum(row[1] for row in prefixes),
            "original_usb_irq_entries": sum(row[3] for row in rows) - 2 * sum(row[2] for row in prefixes),
            "original_sci_irq_entries": sum(row[4] for row in rows) - 2 * sum(row[3] for row in prefixes),
            "own_caller_instructions": sum(row[7] for row in rows)}


def main():
    binary, firmware, log = map(Path, sys.argv[1:])
    report = log.with_suffix(".json")
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original MIDI dispatcher: local firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/midi-dispatch-reference.json").read_text())
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
    print("Verified original MIDI dispatcher, actual queues/IRQs and explicit native caller replays.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
