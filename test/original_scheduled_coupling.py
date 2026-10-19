# SPDX-License-Identifier: GPL-3.0-only
"""Pin private inputs and independently audit both nominal processor timelines."""

from collections import defaultdict
import json
from pathlib import Path
import re
import sys

import original_dsp_boot
from original_dsp_clock import identity, nominal_units
from original_owned_clock import census, phases


COMMON_HZ = 540000000
READY_DELAY = 5400


def fields(line):
    return {key: int(value, 0) for key, value in
            (item.split("=", 1) for item in line.split()[1:])}


def clock_ledger(text):
    writes = [(int(c), int(a, 16), int(v, 16)) for c, a, v in
              re.findall(r"^cycle=(\d+) write=(0x[0-9a-f]+):(0x[0-9a-f]+)$", text, re.M)]
    switches = [(c, v) for c, a, v in writes if a == 0x39]
    assert switches == [(75, 0x80), (233898, 0), (233906, 0x80)], switches
    assert all(value == 0 for _, address, value in writes if address == 0x35)

    def ticks_at(boundary):
        ticks = previous = 0
        period = 10  # 60 MHz reference: 12 MHz OSC with divide-by-two CPU bus.
        for cycle, value in switches:
            if boundary <= cycle:
                break
            ticks += (cycle - previous) * period
            previous = cycle
            period = 2 if value == 0x80 else 10
        return ticks + (boundary - previous) * period

    return ticks_at, writes


def verify_events(text, rows, case, ticks_at):
    # Source-cycle records and receiver-wire observations have separate timestamps.
    case_writes = [(int(c), int(value, 16)) for c, value in
                   re.findall(r"^cycle=(\d+) write=0x0001:(0x[0-9a-f]+)$", text, re.M)]
    assert len(case_writes) == 7275
    local = 0
    events = defaultdict(list)
    dsp_steps = wires = 0
    previous_time = -1
    for line in text.splitlines():
        if line.startswith("DSP "):
            item = fields(line)
            row = rows[item["pc"]]
            cost = nominal_units(row)
            period = 45 if row["pctl"] == 0x04401d else 10
            assert row["pctl"] in (0x04401d, 0x02e112)
            assert item["start"] == local and item["units"] == cost
            assert item["ticks"] == cost * period
            local += cost * period
            assert item["end"] == case["epoch"] + local
            events[item["end"]].append("dsp")
            dsp_steps += 1
        elif line.startswith("MCU-WIRE "):
            item = fields(line)
            cycle, value = case_writes[wires]
            assert item["end"] == ticks_at(cycle) * 9 and item["value"] == value
            events[item["end"]].append("mcu")
            wires += 1
        else:
            continue
        assert item["end"] >= previous_time
        previous_time = item["end"]
    ties = 0
    expected = ["dsp", "mcu"] if case["name"].endswith("dsp-first") else ["mcu", "dsp"]
    for order in events.values():
        if "dsp" in order and "mcu" in order:
            assert order == expected, order
            ties += 1
    assert ties == case["ties"]
    assert dsp_steps == case["dsp-steps"] and wires == 7275
    assert local == case["dsp-ticks"]
    assert case["epoch"] + local <= case["common"]
    return {"completed_dsp_steps": dsp_steps, "wire_writes": wires,
            "literal_tie_order_checks": ties, "dsp_reference_ticks": local}


def verify_case(trace, common, line):
    tokens = dict(item.split("=", 1) for item in line.split()[1:])
    case = {key: value if key == "name" else int(value, 0) for key, value in tokens.items()}
    prefix = Path(str(trace) + "." + case["name"])
    event_text = Path(str(prefix) + ".trace").read_text()
    ticks_at, _ = clock_ledger(common + event_text)
    costs = census(Path(str(prefix) + ".mcu.jsonl"))
    expanded = phases(Path(str(prefix) + ".phases.json"))
    expected_cycles = 5 + costs["instruction_cycles"] + case["entries"] * 9 + expanded["low"]
    assert case["retired"] == costs["retired"]
    assert case["cycles"] == expected_cycles
    assert case["extra"] == expanded["low"]
    assert case["ticks"] == ticks_at(expected_cycles)
    assert case["OSC"] == case["ticks"] // 5
    assert case["common"] == case["ticks"] * 9
    assert case["ready"] >= case["epoch"] + READY_DELAY
    assert 0 < case["pairs"] < 8192
    rows = [json.loads(row) for row in Path(str(prefix) + ".dsp.jsonl").read_text().splitlines()]
    units = ticks = steps = 0
    for row in rows:
        cost = nominal_units(row)
        assert row["units"] == cost
        assert row["pctl"] in (0x04401d, 0x02e112)
        period = 45 if row["pctl"] == 0x04401d else 10
        units += row["count"] * cost
        ticks += row["count"] * cost * period
        steps += row["count"]
    assert units == case["dsp-units"] and ticks == case["dsp-ticks"]
    assert steps == case["dsp-steps"]
    events = verify_events(event_text, {row["pc"]: row for row in rows}, case, ticks_at)
    return {"observed": case, "mcu_manual_costs": costs, "event_checks": events}


def verify(trace):
    text = trace.read_text()
    lines = [line for line in text.splitlines() if line.startswith("case ")]
    assert len(lines) == 4
    common = "\n".join(line for line in text.splitlines() if line.startswith("cycle=")) + "\n"
    cases = [verify_case(trace, common, line) for line in lines]
    assert {row["observed"]["name"] for row in cases} == {
        "phase0-dsp-first", "phase0-mcu-first", "phase9-dsp-first", "phase9-mcu-first"}
    assert sum(row["observed"]["ties"] for row in cases) > 0
    rejected = [line for line in text.splitlines() if line.startswith("rejected ")]
    assert len(rejected) == 4
    for reason in ("Ambiguous simultaneous", "configured idle SCK",
                   "Unqualified simultaneous DSP completion", "Unknown DSP SHI MOSI"):
        assert sum(reason in line for line in rejected) == 1
    return {"scope": "Declared common nominal completion experiment; not physical board timing",
            "common_hz": COMMON_HZ, "cases": cases, "rejection_cases": 4}


def main():
    inputs = [Path(name) for name in sys.argv[1:4]]
    frozen = {str(path): identity(path) for path in inputs if path.is_file()}
    code = original_dsp_boot.main()
    if code:
        return code
    trace = Path(sys.argv[4])
    result = verify(trace)
    assert all(identity(Path(name)) == expected for name, expected in frozen.items())
    result["inputs"] = frozen
    Path(str(trace) + ".verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Independent MCU/DSP manual costs, wire timestamps, tie policies and replay agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
