# SPDX-License-Identifier: GPL-3.0-only
"""Pin unmodified firmware before the independent native master-output regression."""

import itertools
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original DSP master output: private firmware absent")
        return 77
    shared = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    reference = json.loads((ROOT / "doc/dsp-master-output-reference.json").read_text())
    original_records(firmware, shared)
    if (reference["program_record"] != shared["records"][44] or
            reference["midi_sha256"] != shared["midi_sha256"]):
        raise ValueError("Changed original master-output record selection")
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=220, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "runtime error:")):
        raise ValueError(f"Native original master output failed: {diagnostics}")
    cases, streams, totals = [], [], []
    for row in trace.read_text().splitlines():
        kind, *values = row.split()
        decoded = tuple(map(int, values))
        {"CASE": cases, "STREAM": streams, "RESULT": totals}[kind].append(decoded)
    expected_cases = list(itertools.product(range(128), (0, 0x400, 0x800), (0, 32), (0, 1), range(4)))
    if [row[:5] for row in cases] != expected_cases:
        raise ValueError("Incomplete volume/scaling/phase/rounding/input matrix")
    if len(cases) != 6144 or not any(row[5] for row in cases):
        raise ValueError("Missing clipping paths or independently checked full calls")
    if streams != list(itertools.product((0, 0x400, 0x800), (0, 1), range(4))):
        raise ValueError("Missing all six dependent four-block streams")
    if totals != [(6169, 18, 5)]:
        raise ValueError("Missing mid-DO restores, strict read failures or trusted recovery")
    report.write_text(json.dumps({"passed": True, "matrix_calls": len(cases),
        "dependent_blocks": len(streams), "dependent_streams": 6,
        "total_calls": totals[0][0], "restores": totals[0][1], "strict_failures": totals[0][2],
        "independent_output_words": totals[0][0] * 32,
        "scope": reference["qualification"], "fuzz_execution": False,
        "physical_audio": False}, indent=2) + "\n")
    print("Verified 6169 unmodified master-output calls, 197408 independent words, six dependent "
          "streams, 18 restores and five strict read failures; no boot or physical audio claim.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
