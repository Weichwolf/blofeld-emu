# SPDX-License-Identifier: GPL-3.0-only
"""Pin original USB routines before cold-start/IRQ/reset and replay checks."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(text):
    def rows(kind):
        return [tuple(map(int, match)) for match in re.findall(
            rf"^{kind} (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    resets, replays, configurations = rows("RESET"), rows("REPLAY"), rows("CONFIGURATION")
    if len(resets) != 3 or [row[:5] for row in resets] != [
            (0, 1, 0, 1, 1), (1, 3, 2, 1, 1), (2, 256, 255, 1, 1)]:
        raise ValueError("Incomplete original USB reset/counter/IRQ/RTI matrix")
    if replays != [resets[1], resets[1]]:
        raise ValueError("Missing complete mid-ISR replay comparisons")
    if len(configurations) != 1 or configurations[0][:5] != (1, 0, 0, 1, 1):
        raise ValueError("Missing original configuration IRQ acknowledgement and return")
    frontiers = re.findall(r"^MIDI_FRONTIER (IN|OUT) (.+)$", text, re.M)
    if len(frontiers) != 2 or {kind for kind, _ in frontiers} != {"IN", "OUT"}:
        raise ValueError("Missing original MIDI descriptor failure evidence")
    for kind, error in frontiers:
        expected = "physical-endpoint=2 description=0x820008d1" if kind == "IN" else (
            "physical-endpoint=3 description=0x820008c1")
        if "Reserved USB local endpoint NISOF field " + expected != error:
            raise ValueError("A different failure replaced the original NISOF boundary")
    if any(row[-1] <= 0 for row in resets + replays + configurations):
        raise ValueError("A genuine original instruction path is required")
    return {"reset_profiles": len(resets), "replays": len(replays),
            "configuration_irqs": len(configurations), "reserved_midi_failures": len(frontiers),
            "original_delay_calls": sum(row[2] for row in resets),
            "native_instructions": sum(row[-1] for row in resets + replays + configurations)}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original USB reset: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/usb-reset-reference.json").read_text())
    original_mcu(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=280)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "runtime error:")):
        raise ValueError(f"Original USB reset execution failed: {diagnostics}")
    totals = validate(trace.read_text())
    report.write_text(json.dumps({"passed": True, **totals, "scope": reference["qualification"],
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified original cold-start USB reset/configuration IRQs and two joint mid-ISR "
          "replays. Original MIDI endpoints retain their reserved NISOF failure; "
          "no successful MIDI, physical USB timing, full board boot or fuzzing claim.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
