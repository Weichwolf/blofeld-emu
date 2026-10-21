# SPDX-License-Identifier: GPL-3.0-only
"""Pin original firmware and verify directed CPU-clocked serial evidence."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "bytes": {"bytes": 256, "interrupts": 2304, "returns": 2304, "callbacks": 0, "ring-wraps": 4},
    "pitch": {"bytes": 12, "interrupts": 108, "returns": 108, "callbacks": 5},
    "checkpoint": {"snapshots": 1, "identical-suffixes": 2, "changed-suffixes": 1,
                   "unknown-DI-failures": 1, "recovered-suffixes": 1, "callbacks": 4},
}


def verify_trace(text, scope):
    rows = re.findall(r"^RESULT ([^\n]+)$", text, re.M)
    if len(rows) != 1:
        raise ValueError("Missing or duplicated original clocked-wire result")
    fields = [token.split("=", maxsplit=1) for token in rows[0].split()]
    if any(len(field) != 2 for field in fields) or len({field[0] for field in fields}) != len(fields):
        raise ValueError("Malformed or duplicated clocked-wire result field")
    values = dict(fields)
    if values.pop("scope", None) != scope:
        raise ValueError("Changed directed original clocked-wire scope")
    if {key: int(value) for key, value in values.items()} != EXPECTED[scope]:
        raise ValueError("Incomplete original clocked-wire evidence")
    faults = re.findall(r"^EXPECTED_FAILURE ([^\n]+)$", text, re.M)
    if scope == "checkpoint":
        if (len(faults) != 1
                or "DSP reference step pc=0x002ce4 omr=0x600085" not in faults[0]
                or "Unknown DSP timer DI read at X:0xffff8f" not in faults[0]):
            raise ValueError("Missing exact original unknown-DI failure context")
    elif faults:
        raise ValueError("Unexpected failure in a normal original wire scope")
    return EXPECTED[scope]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:4])
    scope = sys.argv[4]
    if scope not in EXPECTED:
        raise ValueError("Unknown original clocked-wire scope")
    if not firmware.is_file():
        print("SKIP original clocked-wire regression: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(trace.resolve()), scope], check=False,
                            timeout=240 if scope == "bytes" else 110)
    if result.returncode:
        return result.returncode
    totals = verify_trace(trace.read_text(), scope)
    report.write_text(json.dumps({"passed": True, "scope": scope, **totals,
        "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
        "qualification": "Declared nominal CPU/timer reference with artificial bit period; "
                         "no physical baud, complete boot or original audio qualification."},
        indent=2) + "\n")
    print("Verified original CPU-clocked wire scope: " + scope)
    return 0


if __name__ == "__main__":
    sys.exit(main())
