# SPDX-License-Identifier: GPL-3.0-only
"""Audit a declared reset-net experiment executing unchanged original MCU/DSP code."""

import json
from pathlib import Path
import re
import sys

import original_dsp_boot
from original_dsp_clock import identity, nominal_units
from original_owned_clock import census, phases, record
from original_scheduled_coupling import clock_ledger


ASSERT_TICK = 4213296
RELEASE_TICK = 87317154
RESET_TICKS = 83103948
HELD = {"retired": 1672473, "cycles": 4850619, "ticks": 9701902,
        "common": 87317118, "assert": ASSERT_TICK, "tail": 18,
        "held": 83103822, "phase": 27}
POSITIVE = {"retired": 1856956, "cycles": 5365452, "ticks": 10731568,
            "common": 96584112, "mcu": 0xb9ad, "dsp": 0x73, "r4": 8193,
            "lc": 122880, "omr": 0x600389, "units": 212306,
            "dsp_ticks": 92370823, "reset_ticks": RESET_TICKS,
            "frames": 97, "writes": 7275, "pairs": 1}


def single(text, prefix):
    line, = [line for line in text.splitlines() if line.startswith(prefix + " ")]
    return record(line)


def verify_mcu(prefix, observed, ticks_at, maximum_partial):
    costs = census(Path(str(prefix) + ".mcu.jsonl"))
    expanded = phases(Path(str(prefix) + ".phases.json"))
    assert expanded["low"] == 0 and costs["retired"] == observed["retired"]
    complete = 5 + costs["instruction_cycles"] + observed.get("entries", 0) * 9
    partial = observed["cycles"] - complete
    assert 0 <= partial < maximum_partial
    assert observed["ticks"] == ticks_at(observed["cycles"])
    assert observed["common"] == observed["ticks"] * 9
    if "OSC" in observed:
        assert observed["OSC"] == observed["ticks"] // 5
    return {"manual_costs": costs, "unretired_periods": partial}


def verify_dsp(prefix, observed, event_text):
    rows = [json.loads(line) for line in Path(str(prefix) + ".dsp.jsonl").read_text().splitlines()]
    lookup = {row["pc"]: row for row in rows}
    units = ticks = attempts = 0
    for row in rows:
        cost = nominal_units(row)
        assert cost == row["units"] and row["pctl"] in (0x04401d, 0x02e112)
        period = 45 if row["pctl"] == 0x04401d else 10
        units += row["count"] * cost
        ticks += row["count"] * cost * period
        attempts += row["count"]
    # The last observed candidate is the genuine, unretired one-unit Y:002000 store.
    assert units - 1 == observed["units"] and attempts == observed["dsp_attempts"]
    assert ticks - 10 == observed["dsp_ticks"] - observed["reset_ticks"]
    assert observed["reset_ticks"] == RELEASE_TICK - ASSERT_TICK + 2 * 45
    local = observed["reset_ticks"]
    events = [record(line) for line in event_text.splitlines() if line.startswith("DSP ")]
    assert len(events) == attempts
    for item in events:
        row = lookup[item["pc"]]
        cost = nominal_units(row)
        period = 45 if row["pctl"] == 0x04401d else 10
        assert item["units"] == cost and item["ticks"] == cost * period
        assert item["start"] == local and item["end"] == ASSERT_TICK + local + item["ticks"]
        local += item["ticks"]
    assert events[-1]["pc"] == 0x72
    assert local - 10 == observed["dsp_ticks"]
    return {"manual_units": units - 1, "manual_ticks": ticks - 10,
            "native_attempts": attempts, "failed_completion": events[-1]["end"]}


def verify_wire(event_text, ticks_at):
    stores = [(int(cycle), int(value, 16)) for cycle, value in re.findall(
        r"^cycle=(\d+) write=0x0001:(0x[0-9a-f]+)$", event_text, re.M)]
    wires = [(int(tick), int(value, 16), int(frame)) for tick, value, frame in re.findall(
        r"^B (\d+) 0x[0-9a-f]+ (0x[0-9a-f]+) frames=(\d+)$", event_text, re.M)]
    assert len(stores) == len(wires) == 7275 and wires[-1][2] == 97
    for (cycle, value), (tick, received, _) in zip(stores, wires):
        assert value == received and tick == ticks_at(cycle) * 9
    return {"actual_native_stores": len(stores), "frames": wires[-1][2]}


def verify(trace):
    text = trace.read_text()
    held = single(text, "HELD_CUT")
    assert held == HELD
    prefix = Path(str(trace) + ".prefix.trace").read_text()
    assertion = single(prefix, "ASSERT")
    assert assertion == {"tick": ASSERT_TICK, "completed": ASSERT_TICK + 18,
                          "pc": 0xb947, "retired": 90322}
    cases = []
    stops = [line for line in text.splitlines() if line.startswith("STOP ")]
    assert len(stops) == 3
    for index, line in enumerate(stops):
        observed = record(line)
        assert observed["replay"] == index and observed["unused_pulldowns"] == int(index != 0)
        assert observed["release"] == RELEASE_TICK and observed["nvm"] == 1
        output = Path(str(trace) + "." + str(index))
        events = Path(str(output) + ".trace").read_text()
        ticks_at, _ = clock_ledger(prefix + events)
        result = {"observed": observed,
                  "mcu": verify_mcu(output, observed, ticks_at, 3)}
        assert all(observed[name] == value for name, value in POSITIVE.items())
        assert "Unmapped DSP memory write at Y:0x002000" in line
        result["dsp"] = verify_dsp(output, observed, events)
        result["wire"] = verify_wire(events, ticks_at)
        assert result["dsp"]["failed_completion"] == observed["common"] + 17
        cases.append(result)
    for suffix in (".trace", ".mcu.jsonl", ".dsp.jsonl", ".phases.json"):
        expected = Path(str(trace) + ".0" + suffix).read_bytes()
        assert expected == Path(str(trace) + ".1" + suffix).read_bytes()
        assert expected == Path(str(trace) + ".2" + suffix).read_bytes()
    assert "REPLAY native=1 ram=1 events=1" in text
    ticks_at, _ = clock_ledger(prefix)
    held_costs = verify_mcu(Path(str(trace) + ".held"), held, ticks_at, 1)
    return {"scope": "Declared PU0/reset-net source experiment; prior DSP power-on, physical routing/timing and full board boot unqualified",
            "held": held, "held_manual_costs": held_costs, "cases": cases,
            "known_unused_U_inputs_are_laboratory_only": True,
            "fuzz_execution": False, "physical_traffic": False}


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
    print("Original reset assertion/release, manual CPU costs, wire timestamps and held replay agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
