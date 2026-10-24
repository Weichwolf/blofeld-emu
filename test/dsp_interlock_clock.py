# SPDX-License-Identifier: GPL-3.0-only
"""Compare actual native dependency clocks with independently measured quiet windows."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from prepare_dsp_pipeline_reference import reference_program
from prepare_dsp_interlock_forms import reference_program as accumulator_program


def fields(output):
    records = []
    for line in output.splitlines():
        if line.startswith(("done ", "fault ")):
            records.append(line)
        else:
            assert re.fullmatch(r"Jit@\d+: No profiler detected", line), line
    assert len(records) == 1, output[-2000:]
    kind, *values = records[0].split()
    return kind, {k: int(v) for k, v in (field.split("=") for field in values)}


def execute(binary, path, window, profile):
    result = subprocess.run([binary, profile, str(path), *(str(n) for n in window)],
                            capture_output=True, text=True, check=False)
    kind, observed = fields(result.stdout)
    return result, kind, observed


def accumulator_forms(binary, directory):
    prepared = accumulator_program()
    reference = json.loads((ROOT / "doc/dsp-interlock-form-reference.json").read_text())
    instructions = prepared["instructions"]
    path = directory / "complete-own-accumulator-program.asm"
    path.write_text("\n".join(instructions) + "\n")
    words = subprocess.run([binary, "dump", str(path)], capture_output=True, text=True, check=True)
    encoded = bytes.fromhex(words.stdout.strip())
    assert hashlib.sha256(encoded).hexdigest() == reference["own_program"]["sha256_24_bit_words"]
    assert len(encoded) == 3 * reference["own_program"]["words"]
    records = []
    start = 9
    for case, expected in zip(prepared["cases"], reference["cases"], strict=True):
        if case["kind"] == "payload":
            continue
        assert (case["kind"], case["gap"]) == (expected["kind"], expected["gap"])
        end = case["body_index"] + (18 if "store" in case["kind"] else 9)
        source = instructions[:9] + instructions[start:end]
        window = [9 + case[name] - start for name in ("window_start", "window_end")]
        path = directory / f"accumulator-{case['kind']}-{case['gap']}.asm"
        path.write_text("\n".join(source) + "\n")
        result, kind, actual = execute(binary, path, window, "interlocks")
        path.with_suffix(".stdout").write_text(result.stdout)
        path.with_suffix(".stderr").write_text(result.stderr)
        if case["kind"] == "short_store":
            assert result.returncode == 2 and kind == "fault", result.stderr
            assert actual["index"] == 9 + case["body_index"] - start
            assert actual["retired"] == actual["index"]
            assert "short-store nominal cost" in result.stderr
        else:
            assert result.returncode == 0 and kind == "done" and not result.stderr, result.stderr
            assert actual["window"] == expected["observed_window"], (case, actual)
            value = 0x200000 if case["kind"] == "long_store" else case["expected_value"]
            assert actual["x1"] == value, (case, actual)
            assert actual["instructions"] == len(source)
        records.append(dict(kind=case["kind"], gap=case["gap"], **actual))
        start = end
    return records


def main():
    binary, directory = sys.argv[1:]
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    _, prepared = reference_program()
    reference = json.loads((ROOT / "doc/dsp-pipeline-timing-reference.json").read_text())
    instructions = prepared["instructions"]
    program = directory / "complete-own-program.asm"
    program.write_text("\n".join(instructions) + "\n")
    words = subprocess.run([binary, "dump", str(program)], capture_output=True, text=True, check=True)
    encoded = bytes.fromhex(words.stdout.strip())
    assert hashlib.sha256(encoded).hexdigest() == reference["own_program"]["sha256_24_bit_words"]
    assert len(encoded) == 3 * reference["own_program"]["words"]
    cases = prepared["cases"]
    assert len(cases) == len(reference["cases"]) == 38
    start = 10  # The own OMR/SR and eight-NOP entry prefix.
    observed_cases = []
    for index, (case, expected) in enumerate(zip(cases, reference["cases"], strict=True)):
        assert (case["kind"], case["gap"]) == (expected["kind"], expected["nop_gap"])
        end = case["body_index"] + 9
        source = instructions[:10] + instructions[start:end]
        window = [10 + case[name] - start for name in ("window_start", "window_end")]
        path = directory / f"case-{index}.asm"
        path.write_text("\n".join(source) + "\n")
        result, kind, actual = execute(binary, path, window, "interlocks")
        (directory / f"case-{index}.stdout").write_text(result.stdout)
        (directory / f"case-{index}.stderr").write_text(result.stderr)
        if expected["independent_expected_stall_cycles"] is None:
            assert result.returncode == 2 and kind == "fault", result.stderr
            assert actual["index"] == 10 + case["body_index"] - start
            assert actual["retired"] == actual["index"]
            assert "Unqualified DSP interlock" in result.stderr and "opcode read P:" in result.stderr
        else:
            assert result.returncode == 0 and kind == "done" and not result.stderr, result.stderr
            assert actual["window"] == expected["observed_clock_cycles"], (case, expected, actual)
            assert actual[case["expected_register"]] == case["expected_value"], (case, actual)
            assert actual["instructions"] == len(source)
            if index == 0:
                old, old_kind, nominal = execute(binary, path, window, "nominal")
                assert old.returncode == 0 and old_kind == "done" and nominal["window"] == 12
                assert nominal["stalls"] == 0 and actual["window"] - nominal["window"] == 3
        observed_cases.append(dict(case_index=index, kind=case["kind"], gap=case["gap"], **actual))
        start = end
    report = dict(passed=True, matched_windows=33, unqualified_rejections=5,
                  word_stream_matches_independent_NXP_assembly=True, cases=observed_cases,
                  scope="Selected dependency completion accounting, not physical bus-stage timing, automatic DMA or full boot")
    report["accumulator_forms"] = accumulator_forms(binary, directory)
    (directory / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Native dependency clocks match33 independent windows; five unqualified status-bit cases reject.")


if __name__ == "__main__":
    main()
