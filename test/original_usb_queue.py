# SPDX-License-Identifier: GPL-3.0-only
"""Pin original queue/callees and validate the complete fixed native matrix."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(text):
    def rows(kind, fields):
        pattern = rf"^{kind}" + r" (\d+)" * fields + "$"
        return [tuple(map(int, row)) for row in re.findall(pattern, text, re.M)]
    headers = rows("HEADER", 8)
    expected = [(message, flags, cursor, busy, replay)
                for message in range(16) for flags in (0, 4, 255)
                for cursor in (0, 124) for busy in (0, 1) for replay in range(3)]
    if [row[:5] for row in headers] != expected:
        raise ValueError("Incomplete original header/flags/cursor/busy/replay matrix")
    for message, flags, _, busy, _, instructions, usb, timer in headers:
        expected_usb = 2 if busy else int(flags == 0 or message in range(1, 6))
        if instructions <= 0 or usb != expected_usb or timer:
            raise ValueError("Incorrect native header delivery/deferral or actual IRQ count")
    full = rows("FULL", 6)
    if [row[:4] for row in full] != [
            (cursor, busy, masked, replay) for cursor in (0, 124)
            for busy, masked in ((0, 1), (1, 1), (0, 0)) for replay in range(3)]:
        raise ValueError("Incomplete actual full-ring/CCR/backpressure controls")
    if any(row[-2] <= 0 or row[-1] != (0 if row[2] else 2) for row in full):
        raise ValueError("Incorrect native full-ring failure or original flush")
    alias, pulc, busy, deadline = rows("ALIAS", 2), rows("PULC", 6), rows("BUSY", 3), rows("DEADLINE", 3)
    if ([row[0] for row in alias] != list(range(3)) or
            [row[:4] for row in pulc] != [(cursor, full, after_irq, replay) for cursor in (0, 124)
                                        for full in (0, 1) for after_irq in (0, 1) for replay in range(3)] or
            [row[0] for row in busy] != list(range(3)) or
            [row[0] for row in deadline] != list(range(3))):
        raise ValueError("Missing raw alias, actual interrupted PULC, busy wake or deadline restore")
    if (any(row[-2] <= 0 or row[-1] != (3 if row[1] else 1) for row in pulc) or
            any(row[-2] <= 0 or row[-1] != 3 for row in busy) or
            any(row[-2] <= 0 or row[-1] != 16 for row in deadline) or any(row[-1] <= 0 for row in alias)):
        raise ValueError("Incorrect actual nested IRQ/clock count or native instruction result")
    prefixes = re.findall(r"^PREFIX (PULC|BUSY|DEADLINE) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)
    expected_prefixes = [("PULC", str(cursor), str(full), str(after_irq)) for cursor in (0, 124)
                         for full in (0, 1) for after_irq in (0, 1)]
    expected_prefixes += [("BUSY", "0", "0", "0"), ("DEADLINE", "0", "0", "0")]
    if [row[:4] for row in prefixes] != expected_prefixes or any(int(row[4]) <= 0 for row in prefixes):
        raise ValueError("Missing independently retained interrupted-prefix accounting")
    summary = ("SUMMARY headers=192 full=6 alias=1 pulc=8 busy=1 deadline=1 restores-per-case=2 "
               "own-peripheral-inputs=1 original-boot=0")
    if text.splitlines()[-1:] != [summary]:
        raise ValueError("Missing exact conditional native scope")
    instructions = sum(row[5] for row in headers) + sum(row[4] for row in full)
    instructions += sum(row[1] for row in alias) + sum(row[4] for row in pulc)
    instructions += sum(row[1] for row in busy + deadline) - 2 * sum(int(row[4]) for row in prefixes)
    return {"cases": 209, "restores": 418, "original_instructions_excluding_replayed_prefixes": instructions,
            "original_usb_irq_entries": sum(row[6] for row in headers) + sum(row[5] for row in full) +
                                        sum(row[5] for row in pulc) + sum(row[2] for row in busy) -
                                        2 * sum(int(row[5]) for row in prefixes),
            "original_timer_irq_entries": sum(row[2] for row in deadline) - 2 * sum(int(row[6]) for row in prefixes)}


def main():
    binary, firmware, log = map(Path, sys.argv[1:])
    report = log.with_suffix(".json")
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original MIDI queue: local firmware absent")
        return 77
    original_mcu(firmware, json.loads((ROOT / "doc/usb-midi-queue-reference.json").read_text()))
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())],
                            capture_output=True, text=True, check=False, timeout=160)
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stdout + result.stderr)
        return result.returncode
    totals = validate(result.stdout)
    report.write_text(json.dumps({"passed": True, **totals,
                                  "scope": "Own valid USB/RAM/caller and scan-inhibited timer inputs; "
                                           "not original initialization, boot or physical USB timing."}, indent=2) + "\n")
    print("Verified original MIDI event queue, actual USB/RTI interrupts, native deadline and replays.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
