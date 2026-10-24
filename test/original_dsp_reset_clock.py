# SPDX-License-Identifier: GPL-3.0-only
"""Check original bootstrap replay across a nominal continuous-time reset."""

import json
from pathlib import Path
import re
import sys

import original_dsp_boot
from original_dsp_clock import identity, nominal_units, verify


def records(text):
    return {name: {key: int(value) for key, value in
                   (field.split("=") for field in fields)}
            for name, *fields in (line.split() for line in text.splitlines())}


def verify_reset(text, rows):
    data = records(text)
    units = ticks = 0
    for row in rows:
        cost = nominal_units(row)
        if row["units"] != cost or row["pctl"] not in (0x04401d, 0x02e112):
            raise ValueError("Unqualified second-epoch nominal instruction")
        units += cost * row["count"]
        ticks += cost * row["count"] * (45 if row["pctl"] == 0x04401d else 10)
    first = data["frontier"]
    if (first["cycles"], first["ticks"], first["clk2"]) != (units, ticks, units // 2):
        raise ValueError("Reset bootstrap changed the complete original nominal prefix")
    expected = {"epoch": 1, "native": units, "local": units, "units": units,
                "ticks": ticks, "edges": units // 2, "faulted": 0}
    if data["before_reset"] != expected:
        raise ValueError("Unexpected pre-reset epoch state")
    expected.update(epoch=2, native=0, local=0)
    if data["reset_epoch"] != expected:
        raise ValueError("Reset must retain lifetime time and restart only native counters")
    reset = data["reset"]
    if (reset["cycles"], reset["ticks"], reset["clk2"], reset["pc"], reset["omr"]) != (
            units, ticks, units // 2, 0xff0000, 0x305):
        raise ValueError("Reset entry, mode or lifetime counters differ")
    repeated = dict(first, cycles=units * 2, ticks=ticks * 2, clk2=(units // 2) * 2)
    if data["reset_frontier"] != repeated:
        raise ValueError("Repeated original stage lost its independent time/loop frontier")
    expected.update(native=units, local=units, units=units * 2, ticks=ticks * 2,
                    edges=(units // 2) * 2)
    if data["reset_complete"] != expected:
        raise ValueError("Native and lifetime counters diverged after repeated bootstrap")
    rejected = data["reset_rejected"]
    expected.update(faulted=1)
    if {key: value for key, value in rejected.items() if key != "native"} != {
            key: value for key, value in expected.items() if key != "native"}:
        raise ValueError("The reserved store must quarantine the same completed time prefix")
    return {"passed": True, "epochs": 2, "nominal_units": units * 2,
            "reference_ticks": ticks * 2, "original_stage_loads": 2,
            "original_stage_words_per_load": 94, "mapped_pairs_per_epoch": 8192,
            "oracle_negative_controls": 5, "fuzz_execution": False, "physical_traffic": False,
            "scope": "Original ROM/stage and two nominal instruction-boundary epochs; "
            "reserved Y:002000 still fails. No held-reset, physical latency or full boot claim."}


def negative_controls(text, rows):
    for name, field in (("reset_epoch", "ticks"), ("reset_epoch", "epoch"),
                        ("reset_epoch", "native"), ("reset_frontier", "cycles"),
                        ("reset_rejected", "faulted")):
        pattern = "^(" + name + r" [^\n]*?\b" + field + r"=)(\d+)"
        changed, count = re.subn(pattern, lambda m: m[1] + str(int(m[2]) ^ 1),
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError("Missing reset oracle-control field")
        try:
            verify_reset(changed, rows)
        except ValueError:
            pass
        else:
            raise ValueError("Reset oracle accepted a corrupted epoch/time/failure field")


def main():
    inputs = [Path(name) for name in sys.argv[1:4]]
    frozen = {str(path): identity(path) for path in inputs if path.is_file()}
    code = original_dsp_boot.main(("reset",))
    if code:
        return code
    trace = Path(sys.argv[4])
    first = verify(trace)
    rows = [json.loads(line) for line in
            Path(str(trace) + ".reset-census.jsonl").read_text().splitlines()]
    text = trace.read_text()
    result = verify_reset(text, rows)
    negative_controls(text, rows)
    if not all(identity(Path(name)) == value for name, value in frozen.items()):
        raise ValueError("Original reset inputs changed during native execution")
    result.update(inputs=frozen, initial_epoch=first)
    Path(str(trace) + ".verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Original bootstrap repeats with monotonic time; the reserved store still fails.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
