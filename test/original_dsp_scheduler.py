# SPDX-License-Identifier: GPL-3.0-only
"""Run unchanged original scheduler words against independent signed equations."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_scheduler_model import directed_cases, validate
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def check_trace(result, cases, reference):
    returns = re.findall(r"^CALLER_RETURN ([0-9a-f]+)$", result, re.M)
    if returns != ["ff004c"]:
        raise ValueError("Changed own explicit-register harness return address")
    metrics, steps = validate(result, cases, int(returns[0], 16))
    rows = re.findall(r"^VISITED([^\n]+)$", result, re.M)
    if len(rows) != 1:
        raise ValueError("Missing original scheduler instruction coverage")
    visited = [int(value, 16) for value in rows[0].split()]
    encoded = "".join(f"{pc:06x}\n" for pc in visited).encode("ascii")
    if (visited != reference["instruction_starts"]
            or hashlib.sha256(encoded).hexdigest() != reference["coverage_sha256"]):
        raise ValueError("Changed original scheduler instruction coverage")
    total = re.findall(r"^TOTAL (\d+) (\d+)$", result, re.M)
    if total != [(str(reference["cases"]), str(steps))]:
        raise ValueError("Incomplete original scheduler totals")
    return dict(cases=len(cases), native_instructions=steps,
                instruction_starts=len(visited), **metrics)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original scheduler regression: private firmware absent")
        return 77
    records = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, records)
    reference = json.loads((ROOT / "doc/dsp-scheduler-reference.json").read_text())
    for expected in reference["complete_original_program_records"]:
        if expected != records["records"][expected["record"]]:
            raise ValueError("Scheduler source component differs from original record identity")
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    cases = directed_cases()
    inputs = trace.with_suffix(".cases")
    inputs.write_text("".join(" ".join(f"{value:x}" for value in [i, *case]) + "\n"
                              for i, case in enumerate(cases)))
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(inputs.resolve()), str(trace.resolve())],
                            check=False, timeout=240)
    if result.returncode:
        return result.returncode
    totals = check_trace(trace.read_text(), cases, reference)
    report.write_text(json.dumps({"passed": True, "scope": reference["qualification"],
                                  **totals}, indent=2) + "\n")
    print(f"Verified {totals['cases']} original scheduler cases and "
          f"{totals['instruction_starts']} instruction starts; "
          "queue completion and complete voice/audio scheduling remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
