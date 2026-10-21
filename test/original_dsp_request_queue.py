# SPDX-License-Identifier: GPL-3.0-only
"""Check original descriptors and actual service paths to the timer-input boundary."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_request_queue_model import (CALLBACK, DEFAULT, FIELDS, TAIL, directed_cases,
                                     queue_model, validate)
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
CALLER_RETURN = 0xff0051


def service_state(block, case, empty):
    target = case["target"]
    payload = 528 if target in (TAIL, DEFAULT) else 72 if target == CALLBACK else case["x0"]
    fmt = 0 if target in (TAIL, DEFAULT, CALLBACK) else case["x1"]
    command = 0x1c00 if target == TAIL else 0x200 if target == CALLBACK else case["y0"]
    address = 0x11e0 if target in (TAIL, CALLBACK) else case["y1"]
    pointer = CALLBACK if target == TAIL else 0x720 if target == CALLBACK else case["r0"]
    model = queue_model(fmt, command, address, pointer, payload,
                        case["head"], case["tail"], case["bases"])
    registers = list(model["registers"])
    registers[0] = (registers[0] & (255 << 48)) | ((model["published"] ^ case["tail"]) << 24)
    pc = (0x244 if target == TAIL else CALLER_RETURN) if empty else 0x188a
    sp = (1 if target == TAIL else 0) if empty else (2 if target == TAIL else 1)
    ret = CALLER_RETURN if empty else 0x244 if target == TAIL else CALLER_RETURN
    rows = re.findall(r"^SERVICE_STATE ([^\n]+)$", block, re.M)
    if len(rows) != 1:
        raise ValueError("Missing actual post-descriptor CPU state")
    values = [int(value, 16) for value in rows[0].split()]
    address_registers = [pointer, model["r1"], case["tail"], 0x700, 0x800, 0xc500, 0, 0]
    modifiers = [case["m0"], 0xffffff, case["m0"], *([0xffffff] * 5)]
    if (len(values) != 24 or values[:2] != [pc, sp] or (sp and values[2] != ret)
            or values[3:7] != registers or values[8:] != address_registers + modifiers):
        raise ValueError("Original service full registers/frame differ from independent equations")
    if values[7] & (1 << 21 | 0xc00) != case["mode"] or bool(values[7] & 4) != empty:
        raise ValueError("Original service new-head XOR flags or rounding/scaling differs")


def check_service(result, cases):
    blocks = re.findall(r"^CASE [0-9a-f]+\n([\s\S]*?)(?=^CASE |^VISITED)", result, re.M)
    returned = faulted = 0
    for block, case in zip(blocks, cases):
        rows = re.findall(r"^SERVICE_(RETURN|FAULT) ([^\n]+)$", block, re.M)
        if case["scan"]:
            if rows:
                raise ValueError("Incomplete descriptor must not enter the queue service")
            continue
        empty = 0x2700 + ((case["head"] - 0x2700 + 4) % 256) == case["tail"]
        if empty:
            pc, sp = (0x244, 1) if case["target"] == TAIL else (CALLER_RETURN, 0)
            if rows != [("RETURN", f"{pc:x} {sp:x}")]:
                raise ValueError("Original queue-empty RTS frame differs")
            returned += 1
        else:
            if (len(rows) != 1 or rows[0][0] != "FAULT"
                    or "pc=0x00188a" not in rows[0][1] or "omr=0x600389" not in rows[0][1]
                    or "Unknown DSP timer DI read at X:0xffff8b" not in rows[0][1]):
                raise ValueError("Missing actual original TIO1 failure")
            faulted += 1
        service_state(block, case, empty)
    total = re.findall(r"^PROBE_TOTAL (\d+) (\d+) (\d+)$", result, re.M)
    if (len(total) != 1 or tuple(map(int, total[0][:2])) != (returned, faulted)
            or returned + faulted != 152 or int(total[0][2]) <= 0):
        raise ValueError("Incomplete post-descriptor service probes")
    return dict(service_returns=returned, unknown_TIO1_failures=faulted,
                service_instructions=int(total[0][2]))


def check_trace(result, cases, reference):
    if re.findall(r"^CALLER_RETURN ([0-9a-f]+)$", result, re.M) != [f"{CALLER_RETURN:x}"]:
        raise ValueError("Changed explicit own caller return")
    metrics, steps = validate(result, cases, CALLER_RETURN)
    rows = re.findall(r"^VISITED([^\n]+)$", result, re.M)
    if len(rows) != 1:
        raise ValueError("Missing original queue instruction coverage")
    visited = [int(value, 16) for value in rows[0].split()]
    expected = sorted(reference["queue_instruction_starts"] + reference["integrated_instruction_starts"])
    encoded = "".join(f"{pc:06x}\n" for pc in visited).encode("ascii")
    if visited != expected or hashlib.sha256(encoded).hexdigest() != reference["coverage_sha256"]:
        raise ValueError("Changed original queue/scheduler instruction coverage")
    if re.findall(r"^TOTAL (\d+) (\d+)$", result, re.M) != [(str(len(cases)), str(steps))]:
        raise ValueError("Incomplete original descriptor totals")
    return dict(cases=len(cases), prefix_instructions=steps,
                instruction_starts=len(visited), **metrics, **check_service(result, cases))


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original request queue regression: private firmware absent")
        return 77
    records = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, records)
    reference = json.loads((ROOT / "doc/dsp-request-queue-reference.json").read_text())
    expected = reference["complete_original_queue_record"]
    if records["records"][expected["record"]] != expected:
        raise ValueError("Queue source component differs from original record identity")
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    cases = directed_cases()
    inputs = trace.with_suffix(".cases")
    inputs.write_text("".join(" ".join(f"{value:x}" for value in
                                      [i, *[case[name] for name in FIELDS], *case["bases"]]) + "\n"
                              for i, case in enumerate(cases)))
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(inputs.resolve()), str(trace.resolve())],
                            check=False, timeout=240)
    if result.returncode:
        return result.returncode
    totals = check_trace(trace.read_text(), cases, reference)
    report.write_text(json.dumps({"passed": True, "scope": reference["qualification"],
                                  **totals}, indent=2) + "\n")
    print(f"Verified {totals['complete_descriptors']} original descriptors and actual "
          "queue service boundaries; transfers and complete audio scheduling remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
