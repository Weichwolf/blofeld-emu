# SPDX-License-Identifier: GPL-3.0-only
"""Check actual status dependencies against primary expectations and NXP windows."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from dsp_interlock_clock import execute

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from prepare_dsp_status_reference import reference_program


def main():
    binary, destination = sys.argv[1:]
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    _, prepared = reference_program()
    reference = json.loads((ROOT / "doc/dsp-status-timing-reference.json").read_text())
    instructions = prepared["instructions"]
    path = destination / "complete-own-status-program.asm"
    path.write_text("\n".join(instructions) + "\n")
    result = subprocess.run([binary, "dump", str(path)], capture_output=True, text=True, check=True)
    encoded = bytes.fromhex(result.stdout.strip())
    assert hashlib.sha256(encoded).hexdigest() == reference["own_program"]["sha256_24_bit_words"]
    assert len(encoded) == reference["own_program"]["words"] * 3
    records = []
    for index, (case, expected) in enumerate(zip(prepared["cases"], reference["cases"], strict=True)):
        assert all(case[key] == expected[key] for key in ("kind", "consumer", "gap"))
        start = case["start_index"]
        source = instructions[:9] + instructions[start:case["body_index"] + 9]
        path = destination / f"case-{index}.asm"
        path.write_text("\n".join(source) + "\n")
        window = [9 + case[key] - start for key in ("window_start", "window_end")]
        result, kind, actual = execute(binary, path, window, "interlocks")
        path.with_suffix(".stdout").write_text(result.stdout)
        path.with_suffix(".stderr").write_text(result.stderr)
        if expected["qualified"]:
            assert result.returncode == 0 and kind == "done" and not result.stderr, result.stderr
            primary = 12 + case["gap"] + (case["consumer"] == "bit") + case["expected_stall_cycles"]
            assert actual["window"] == primary == expected["observed_window"], (case, actual)
            assert actual[case["expected_register"]] == case["expected_value"], (case, actual)
            assert actual["x0"] == case["expected_bus_value"], (case, actual)
            assert actual["stalls"] == case["expected_stall_cycles"]
            assert actual["instructions"] == len(source)
        else:
            assert result.returncode == 2 and kind == "fault", result.stderr
            assert actual["index"] == actual["retired"] == 9 + case["body_index"] - start
            assert "status timing after partial accumulator read" in result.stderr
            assert "opcode read P:" in result.stderr
        records.append(dict(case_index=index, qualified=expected["qualified"], **actual))
    assert len(records) == 64 and sum(case["qualified"] for case in records) == 48
    report = dict(passed=True, matched_windows=48, unqualified_partial_read_windows=16,
                  complete_word_stream_matches_independent_NXP_assembly=True, cases=records,
                  scope="Selected status completion clocks and actual flags/data; no physical pipeline phases")
    (destination / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Matched48 primary/NXP status windows;16 unresolved partial-read windows reject.")


if __name__ == "__main__":
    main()
