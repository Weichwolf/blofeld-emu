# SPDX-License-Identifier: GPL-3.0-only
"""Controlled TIO1 input paths through original queue polling and first GPIO access."""

import copy
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_request_queue_model import (CALLBACK, FIELDS, TAIL, directed_cases,
                                     queue_model, validate_windows)
from original_dsp_request_queue import CALLER_RETURN, service_state
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def readiness_cases():
    original = directed_cases()
    cases = []
    for index in (136, 140, 144, 148):
        for profile in range(1, 5):
            case = copy.deepcopy(original[index])
            case["ready_profile"] = profile
            cases.append(case)
    if any(c["head"] != c["tail"] or c["scan"] for c in cases):
        raise ValueError("Readiness cases require genuine nonempty queues after publication")
    return cases


def gpio_state(block, case):
    scheduler = case["target"] == TAIL
    command, pointer, payload = (0x1c00, CALLBACK, 528) if scheduler else (0x200, 0x720, 72)
    model = queue_model(0, command, 0x11e0, pointer, payload,
                        case["head"], case["tail"], case["bases"])
    registers = list(model["registers"])
    registers[0] = (registers[0] & (255 << 48)) | (model["header"] << 24)
    pc, sp, ret = (0x1894, 2, 0x244) if scheduler else (0x18a5, 1, CALLER_RETURN)
    rows = re.findall(r"^READY_STATE ([^\n]+)$", block, re.M)
    if len(rows) != 1:
        raise ValueError("Missing actual GPIO-failure pre-step state")
    values = [int(v, 16) for v in rows[0].split()]
    wanted_addresses = [pointer, model["r1"], case["tail"] + 1, 0x700, 0x800, 0xc500, 0, 0]
    modifiers = [case["m0"], 0xffffff, 0x80ff, *([0xffffff] * 5)]
    if (len(values) != 24 or values[:7] != [pc, sp, ret, *registers]
            or values[8:] != wanted_addresses + modifiers):
        raise ValueError("Original ready service header load/registers/frame differ")
    if values[7] & ((1 << 21) | 0xc00) != case["mode"] or values[7] & 4:
        raise ValueError("Readiness changed rounding/scaling or new-head XOR flags")


def validate(result, cases):
    metrics, instructions = validate_windows(result, cases, CALLER_RETURN)
    if metrics["complete_descriptors"] != 16 or metrics["scan_frontiers"] != 0:
        raise ValueError("Readiness must follow sixteen complete original descriptors")
    blocks = re.findall(r"^CASE [0-9a-f]+\n([\s\S]*?)(?=^CASE |^VISITED)", result, re.M)
    expected_counts = {1: [1, 0, 0, 0, 0, 0], 2: [4, 4, 0, 0, 0, 0],
                       3: [1, 0, 1, 1, 1, 1], 4: [3, 2, 1, 1, 1, 1]}
    for block, case in zip(blocks, cases):
        profile = case["ready_profile"]
        rows = re.findall(r"^READY_COUNTS ([^\n]+)$", block, re.M)
        if len(rows) != 1 or list(map(int, rows[0].split())) != expected_counts[profile]:
            raise ValueError("Actual TIO1 polls, GPIO attempts or native writes differ")
        if re.findall(r"^READY_PROFILE (\d+)$", block, re.M) != [str(profile)]:
            raise ValueError("Controlled input profile differs")
        error = re.findall(r"^READY_ERROR ([^\n]+)$", block, re.M)
        expected_pc = "001894" if case["target"] == TAIL else "0018a5"
        if profile == 2:
            if error != ["none"]:
                raise ValueError("Low TIO1 must hold the actual polling loop")
        elif (len(error) != 1 or "omr=0x600389" not in error[0]
              or (profile == 1 and ("pc=0x00188a" not in error[0]
                    or "Unknown DSP timer DI read at X:0xffff8b" not in error[0]))
              or (profile > 2 and (f"pc=0x{expected_pc}" not in error[0]
                    or "Unknown DSP GPIO data read at X:0xffffbd mask=0x000100" not in error[0]))):
            raise ValueError("Missing original unknown-input/latch failure context")
        if profile < 3:
            service_state(block.replace("READY_STATE ", "SERVICE_STATE "), case, False)
        else:
            gpio_state(block, case)
        expected_flags = case["flags"] | (0x2000 if profile > 2 else 0)
        if re.findall(r"^READY_FLAGS ([0-9a-f]+)$", block, re.M) != [f"{expected_flags:x}"]:
            raise ValueError("Busy flag must be written only after the original ready decision")
    return dict(cases=len(cases), prefix_instructions=instructions,
                unknown_input_failures=4, bounded_low_polls=4, gpio_latch_failures=8,
                scheduler_producers=8, continuation_producers=8)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original request readiness: private firmware absent")
        return 77
    original_records(firmware, json.loads((ROOT / "doc/dsp-voice-reference.json").read_text()))
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    cases = readiness_cases()
    inputs = trace.with_suffix(".cases")
    inputs.write_text("".join(" ".join(f"{v:x}" for v in
                           [i, *[c[n] for n in FIELDS], *c["bases"], c["ready_profile"]]) + "\n"
                           for i, c in enumerate(cases)))
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(inputs.resolve()), str(trace.resolve()), "readiness"],
                            check=False, timeout=120)
    if result.returncode:
        return result.returncode
    totals = validate(trace.read_text(), cases)
    report.write_text(json.dumps({"passed": True, **totals,
                                  "scope": "Controlled untimed TIO1 inputs, original polling and "
                                           "strict first-GPIO failure; no completed transfer."},
                                 indent=2) + "\n")
    print("Verified original controlled-input polling and GPIO boundaries; "
          "no device transaction or hardware readiness is established.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
