# SPDX-License-Identifier: GPL-3.0-only
"""Audit the actual reserved-write attempt on both original native timelines."""

from collections import Counter
import json
from pathlib import Path
import re

from original_dsp_clock import nominal_units
from original_owned_clock import census, phases


def fields(line):
    return {key: int(value, 0) for key, value in
            (item.split("=", 1) for item in line.split()[1:])}


def single(text, name):
    line, = [row for row in text.splitlines() if row.startswith(name + " ")]
    return fields(line)


def verify_dsp(prefix, text, root, final, epoch):
    rows = [json.loads(line) for line in
            Path(str(prefix) + ".frontier.dsp.jsonl").read_text().splitlines()]
    units = ticks = attempts = 0
    for row in rows:
        cost = nominal_units(row)
        assert row["units"] == cost
        assert row["pctl"] in (0x04401d, 0x02e112)
        period = 45 if row["pctl"] == 0x04401d else 10
        units += cost * row["count"]
        ticks += cost * period * row["count"]
        attempts += row["count"]
    # The before-step observer includes the last, failed candidate. A native
    # operand exception does not retire that word or charge its nominal duration.
    assert units - 1 == final["dsp-units"]
    assert ticks - 10 == final["dsp-ticks"]
    assert attempts - 1 == final["dsp-steps"]
    steps = [fields(line) for line in text.splitlines() if line.startswith("DSP ")]
    assert len(steps) == 8192 - root["r4"] + 1
    local = root["dsp-ticks"]
    for index, step in enumerate(steps):
        assert step["pc"] == 0x72 and step["r4"] == root["r4"] + index
        assert step["start"] == local and step["units"] == 1 and step["ticks"] == 10
        local += 10
        assert step["end"] == epoch + local
    assert steps[-1]["r4"] == 8192
    assert local - 10 == final["dsp-ticks"]
    assert final["dsp-steps"] - root["dsp-steps"] == len(steps) - 1
    assert final["dsp-units"] - root["dsp-units"] == len(steps) - 1
    return steps[-1]["end"]


def verify_mcu(prefix, text, root, final, ticks_at):
    costs = census(Path(str(prefix) + ".frontier.mcu.jsonl"))
    expanded = phases(Path(str(prefix) + ".frontier.phases.json"))
    assert expanded["low"] == 0
    assert costs["retired"] == final["retired"]
    complete_cycles = 5 + costs["instruction_cycles"]
    partial = final["cycles"] - complete_cycles
    # HCS12: LDX immediate takes two; each taken DBNE IX takes three. The last
    # interrupted DBNE has only the clocks already consumed before the peer fault.
    assert 0 <= partial < 3
    starts = [fields(line) for line in text.splitlines() if line.startswith("MCU-BEGIN ")]
    assert len(starts) == final["retired"] - root["retired"] + 1
    assert starts[0]["pc"] == 0xb9aa and starts[0]["cycles"] == root["cycles"]
    assert final["cycles"] == root["cycles"] + 2 + (len(starts) - 2) * 3 + partial
    for index, step in enumerate(starts[1:]):
        assert step["pc"] == 0xb9ad and step["x"] == 65535 - index
        assert step["cycles"] == root["cycles"] + 2 + index * 3
    # Match retired instructions to a separately recorded native census, not
    # just a total that could hide a replaced or skipped delay-loop iteration.
    base = {row["pc"]: sum(row["flags"]) for row in
            map(json.loads, Path(str(prefix) + ".mcu.jsonl").read_text().splitlines())}
    current = {row["pc"]: sum(row["flags"]) for row in
               map(json.loads, Path(str(prefix) + ".frontier.mcu.jsonl").read_text().splitlines())}
    delta = {pc: count - base.get(pc, 0) for pc, count in current.items()
             if count != base.get(pc, 0)}
    assert delta == dict(Counter(step["pc"] for step in starts[:-1]))
    assert final["ticks"] == ticks_at(final["cycles"])
    assert final["OSC"] == final["ticks"] // 5 and final["common"] == final["ticks"] * 9
    return {"manual_costs": costs, "completed_instruction_cycles": complete_cycles,
            "partial_failed_instruction_cycles": partial, "native_attempts": len(starts)}


def verify(prefix, case, ticks_at):
    text = Path(str(prefix) + ".frontier.trace").read_text()
    epoch = single(text, "epoch")["ticks"]
    root, near, final = (single(text, name) for name in ("root", "near", "fault"))
    assert epoch == case["epoch"]
    for key in ("cycles", "ticks", "OSC", "common", "dsp-steps", "dsp-units", "dsp-ticks"):
        assert root[key] == case[key]
    assert root["r4"] == case["pairs"] and root["mcu-pc"] == 0xb9aa
    assert 8160 <= near["r4"] < 8192 and near["lc"] == 131072 - near["r4"]
    assert near["mcu-pc"] == 0xb9ad and near["dsp-pc"] == 0x72
    assert "MCU-WIRE " not in text and "cycle=" not in text
    attempted = verify_dsp(prefix, text, root, final, epoch)
    mcu = verify_mcu(prefix, text, root, final, ticks_at)
    error, = [line for line in text.splitlines() if line.startswith("rejected ")]
    assert "Unmapped DSP memory write at Y:0x002000" in error
    assert "mcu-pc=0xb9ad" in error and "DSP reference step pc=0x000072 omr=0x600389" in error
    declared_attempt, = re.findall(r"attempted-completion=(\d+)", error)
    horizon, = re.findall(r"horizon=(\d+)", error)
    mcu_end, = re.findall(r"mcu-end=(\d+)", error)
    last_ticks, = re.findall(r"completed-reference-ticks=(\d+)", error)
    last_units, = re.findall(r"completed-native-cycle-units=(\d+)", error)
    assert int(declared_attempt) == attempted
    assert int(last_ticks) == final["dsp-ticks"] and int(last_units) == final["dsp-units"]
    assert int(horizon) == int(mcu_end) == ticks_at(final["cycles"] + 1) * 9
    assert epoch + final["dsp-ticks"] < attempted
    assert final["common"] < attempted <= int(horizon)
    near_line, = [line for line in text.splitlines(keepends=True) if line.startswith("near ")]
    suffix = text.split(near_line, 1)[1]
    assert Path(str(prefix) + ".frontier.replay.trace").read_text() == suffix
    return {"scope": "Actual reserved write during the original MCU's final delay; "
                     "no reserved-memory policy or physical timing inferred",
            "root": root, "checkpoint": near, "failed_native_state": final,
            "attempted_completion": attempted, "mcu": mcu,
            "joint_fault_replay_identical": True}
