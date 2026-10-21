# SPDX-License-Identifier: GPL-3.0-only
"""Pin the original archive before complete native workspace call checks."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from dsp_workspace_model import directed_cases
from dsp_workspace_validation import validate
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original workspace regression: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-workspace-reference.json").read_text())
    runtime_path = ROOT / reference["runtime_reference"]["path"]
    if hashlib.sha256(runtime_path.read_bytes()).hexdigest() != reference["runtime_reference"]["sha256"]:
        raise ValueError("Changed ordered original runtime reference")
    runtime = json.loads(runtime_path.read_text())
    if runtime["records"][reference["program"]["record"]] != reference["program"]:
        raise ValueError("Changed original workspace program span")
    original_records(firmware, runtime)
    cases = directed_cases()
    if len(cases) != reference["cases"]:
        raise ValueError("Incomplete directed workspace cases")
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    corpus = trace.with_suffix(".cases")
    corpus.write_text("".join(" ".join(f"{value:x}" for value in row) + "\n" for row in cases))
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(corpus.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=280)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    sanitizer_markers = ("AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer",
                         "runtime error:", "Sanitizer:DEADLYSIGNAL")
    if result.returncode or any(marker in diagnostics for marker in sanitizer_markers):
        raise ValueError(f"Native workspace execution failed ({result.returncode}): {diagnostics}")
    totals = validate(trace.read_text(), cases, reference)
    report.write_text(json.dumps({"passed": True, "scope": reference["qualification"],
                                  "cases": len(cases), "guarded_ram_words": 77824,
                                  "fuzz_execution": False, "physical_traffic": False,
                                  **totals}, indent=2) + "\n")
    print(f"Verified {len(cases)} original workspace calls; all 93 module instruction starts, "
          "mapped RAM guards and two repeated joint snapshots. "
          "Full boot, upstream note allocation, MIDI/audio and hardware timing remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
