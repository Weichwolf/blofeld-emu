# SPDX-License-Identifier: GPL-3.0-only
"""Pin original records before native Reverb clear-to-ready callback checks."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def validate(trace):
    rows = [tuple(map(int, row)) for row in re.findall(
        r"^SCENE (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", trace, re.M)]
    expected = {(mode, diffusion, word) for mode in (0, 1) for diffusion in (0, 64, 127)
                for word in (0, 0xa55aa5, 0xffffff)}
    if len(rows) != len(expected) or {row[:3] for row in rows} != expected:
        raise ValueError("Missing unique reset/live, Diffusion and vacant-word scene")
    minimum = 48 * (512 + 32)
    if any(row[3] < minimum or row[4:] != (24, 48, 50, 2067) for row in rows):
        raise ValueError("Missing native clears/RTS/read count or independent filter result")
    for mode in (0, 1):
        for diffusion in (0, 64, 127):
            if len({row[3] for row in rows if row[:2] == (mode, diffusion)}) != 1:
                raise ValueError("A vacant read value changes original instruction retirement")
    return {"scenes": len(rows), "native_instructions": sum(row[3] for row in rows),
            "clear_calls": sum(row[5] for row in rows),
            "callback_RTS_returns": sum(row[6] for row in rows),
            "vacant_X_reads": sum(row[4] for row in rows),
            "first_filtered_word": 2067}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original Reverb clear: private firmware absent")
        return 77
    reference = json.loads((ROOT / "test/reference/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=160)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "ThreadSanitizer", "runtime error:")):
        raise ValueError(f"Original Reverb lifecycle execution failed: {diagnostics}")
    totals = validate(trace.read_text())
    report.write_text(json.dumps({"passed": True, **totals,
                                  "firmware_zip_sha256": hashlib.sha256(firmware.read_bytes()).hexdigest(),
                                  "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
                                  "scope": "Conditional native DSP setup/clear/ready callbacks with authored caller and declared no-interference inputs; not connected instrument, hardware invalid-domain AGU or audio timing qualification",
                                  "fuzz": False, "physical_traffic": False}, indent=2) + "\n")
    print("Original Reverb: 48 firmware-pointer clear calls then two ready calls per scene; "
          "18 scenes, 864 clears, 900 actual callback RTS returns and 432 real X reads. "
          "Cursor16000 and the first actual discarded store are preserved.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
