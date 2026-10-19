# SPDX-License-Identifier: GPL-3.0-only
"""Pin the original input and check its declared DSP clock estimate ledger."""

import hashlib
import json
from pathlib import Path
import sys

import original_dsp_boot


def identity(path):
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def matches(opcode, pattern):
    # Bit layouts below are independently transcribed from FM chapter 13.
    assert len(pattern) == 24
    return all(char not in "01" or int(char) == ((opcode >> (23 - index)) & 1)
               for index, char in enumerate(pattern))


def ea_penalty(opcode, immediate):
    # FM A-1 and 12-18: post-update by N is baseline; offset/pre-decrement add one.
    mode, register = (opcode >> 11) & 7, (opcode >> 8) & 7
    if mode in (0, 1, 2, 3, 4):
        return 0
    if mode in (5, 7):
        return 1
    if mode == 6 and register == 0:
        return 1  # Long absolute.
    if mode == 6 and register == 4 and immediate is not None:
        return immediate  # Separate long-immediate column.
    raise ValueError("Unreviewed nominal effective-address form")


def nominal_units(row):
    # Independent DSP56300FM Rev5 table A-1 nominal costs. Pipeline interlocks,
    # actual chip timing and analog clock transitions are not inferred here.
    name, opcode = row["name"].upper(), row["opcode"]
    inherent = {"NOP": 1, "CLR": 1, "ORI": 3, "ANDI": 3, "REP": 5, "DO": 5,
                "JCLR": 4, "JSET": 4, "JMP": 3, "BRA": 4}
    if name in inherent:
        return inherent[name]
    if name == "MOVE":
        # FM 13-113/116: immediate-short and register parallel move, with ALU NOP.
        if matches(opcode, "001dddddiiiiiiii00000000"):
            return 1
        # FM 13-131: control register-to-register move (no EA penalty).
        if matches(opcode, "00000100W1eeeeee101DDDDD"):
            return 1
        # FM 13-119: X general EA or long immediate, with ALU NOP.
        if matches(opcode, "01dd0dddW1MMMRRR00000000"):
            return 1 + ea_penalty(opcode, immediate=1)
        # FM 13-127: long X/Y memory pair, with ALU NOP; no immediate form.
        if matches(opcode, "0100L0LLW1MMMRRR00000000"):
            return 1 + ea_penalty(opcode, immediate=None)
    if name == "MOVEP":
        # FM 13-135/136 and A-7: memory-space forms differ from register forms.
        if matches(opcode, "0000100sW1MMMRRR1Spppppp"):
            return 2 + ea_penalty(opcode, immediate=0)
        if matches(opcode, "000000001WMMMRRR0Sqqqqqq"):
            return 6 + ea_penalty(opcode, immediate=None)
        if (matches(opcode, "00000100W1dddddd1q0qqqqq") or
                matches(opcode, "00000100W1dddddd0q1qqqqq")):
            return 1
    raise ValueError(f"DSP nominal instruction form requires primary-manual review: {row}")


def verify(trace):
    rows = [json.loads(line) for line in Path(str(trace) + ".census.jsonl").read_text().splitlines()]
    units = ticks = 0
    for row in rows:
        cost = nominal_units(row)
        assert row["units"] == cost, row
        assert row["pctl"] in (0x04401d, 0x02e112), row
        period = 45 if row["pctl"] == 0x04401d else 10
        units += cost * row["count"]
        ticks += cost * row["count"] * period
    records = {}
    for line in trace.read_text().splitlines():
        name, *fields = line.split()
        records[name] = {key: int(value) for key, value in (field.split("=") for field in fields)}
    assert records["frontier"] == records["replay"]
    final = records["frontier"]
    assert final["cycles"] == units and final["ticks"] == ticks and final["clk2"] == units // 2
    assert final["pc"] == 0x72 and final["r4"] == 8192 and final["lc"] == 0x1e000
    assert final["omr"] == 0x600389
    return {"scope": "Declared native-cycle nominal reference; not calibrated physical timing",
            "nominal_units": units, "reference_ticks": ticks, "records": records}


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
    print("Original DSP nominal cycle/PLL/CLK2 ledger and mid-REP replay agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
