# SPDX-License-Identifier: GPL-3.0-only
"""Pin original forwarding/callees and check the fixed native execution matrix."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def expectations():
    # Independent channel-message lengths; C/D retain the declared stack byte.
    lengths = (3, 3, 3, 3, 2, 2, 3)
    result = []
    for length in lengths:
        for _ in (0, 62, 63):
            for available in range(length + 1):
                complete = int(available == length)
                result.append((complete, 0, complete, 0, complete))
    result += [(0, 0, 0, 0, 1)] * 18  # Discard one data/system byte.
    result += [(1, 0, 1, 0, 1)] * 3   # At most one of two complete messages.
    result += [(2, 0, 1, 0, 1)] * 21  # Actual earlier IN ACK and queued packet.
    result += [(0, 0, 1, 0, 1)] * 14  # Full-ring failure still consumes mirror.
    result += [(2, 0, 1, 0, 1)] * 14  # Full ring invokes original batch sender.
    result += [(3, 0, 1, 0, 1)] * 2   # Busy wait progresses only after actual ACK.
    result += [(1, 0, 1, 0, 1)]       # Raw cursor reads the aliased SCI consumer.
    for length in lengths:
        result += [(1, 1, 1, length, 1)] * 3
    result += [(1, 1, 1, 4, 1)] * 2  # FF stops its real one-byte UART queue call.
    result += [(0, 1, 0, 4, 1)] * 3  # Realtime first; channel message remains.
    result += [(0, 0, 0, 0, 0)]      # Empty peer/TX rings.
    assert len(result) == 178
    return result


def validate(text):
    rows = [tuple(map(int, row)) for row in re.findall(
        r"^CASE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    expected = expectations()
    if [row[:2] for row in rows] != [(case, attempt) for case in range(178) for attempt in range(3)]:
        raise ValueError("Incomplete original forwarding/peer/backpressure/restore matrix")
    for case, _, instructions, *counts in rows:
        if instructions <= 0 or tuple(counts) != expected[case]:
            raise ValueError(f"Incorrect actual original IRQ/callee/publication counts: {case}")
    prefixes = [tuple(map(int, row)) for row in re.findall(
        r"^PREFIX (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if [row[0] for row in prefixes] != list(range(178)):
        raise ValueError("Missing captured native-prefix accounting")
    if text.splitlines()[-1:] != ["SUMMARY cases=178 restores=356 own-peripheral-inputs=1 original-boot=0"]:
        raise ValueError("Missing exact conditional forwarding scope")
    return {"cases": 178, "restores": 356,
            "original_instructions_excluding_replayed_prefixes": sum(row[2] for row in rows) -
                                                                 2 * sum(row[1] for row in prefixes),
            "original_usb_irq_entries": sum(row[3] for row in rows) - 2 * sum(row[2] for row in prefixes),
            "original_sci_irq_entries": sum(row[4] for row in rows) - 2 * sum(row[3] for row in prefixes)}


def main():
    binary, firmware, log = map(Path, sys.argv[1:])
    report = log.with_suffix(".json")
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original MIDI forwarding: local firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/midi-forwarding-reference.json").read_text())
    original_mcu(firmware, reference)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())], capture_output=True,
                            text=True, check=False, timeout=160)
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stdout + result.stderr)
        return result.returncode
    totals = validate(result.stdout)
    report.write_text(json.dumps({"passed": True, **totals, "scope": reference["qualification"],
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified original MIDI forwarding through real UART/USB queues, IRQs and joint restores.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
