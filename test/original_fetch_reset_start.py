# SPDX-License-Identifier: GPL-3.0-only
"""Audit original MCU reset/SPI execution on dsp56300, not complete device boot."""

import collections
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from original_dsp_clock import identity
from original_dsp_clocked_frontier import check_inputs
from original_owned_clock import record
from original_reset_start import ASSERT_TICK, HELD, RELEASE_TICK
from original_reset_start import single, verify_mcu, verify_wire
from original_scheduled_coupling import clock_ledger

# Reference-profile coordinates, additionally checked against independent MCU
# opcode costs, actual GPIO/SPI stores and the native effect/PLL/REP ledgers below.
SUCCESS = "Original reset-net source, dsp56300 runtime, SPI and held replay passed.\n"
POSITIVE = {"release": RELEASE_TICK, "retired": 1864555, "cycles": 5388249,
            "ticks": 10777162, "OSC": 2155432, "entries": 0, "common": 96994458,
            "mcu": 0xb9ad, "dsp": 0x72, "r4": 0x2001, "lc": 0x20000 - 0x2000,
            "omr": 0x600389, "units": 253284, "dsp_ticks": 96994459,
            "reset_ticks": RELEASE_TICK + 2 * 45, "dsp_retired": 42289,
            "clk2": 253284 // 2, "dsp_attempts": 253284, "frames": 97,
            "writes": 7275, "pairs": 1, "nvm": 1}
NATIVE_HELD = {**HELD, "held": HELD["common"], "phase": HELD["common"] % 45}
FAULT = ("HC12 cycle execution pc=0xb9ad ccr=0xd8 completed-cycles=5388249: "
         "Clock/operand P address=unreconstructed-prefetch/internal: "
         "Own DSP timeline operation=reference-cycle pc=0x000072 omr=0x600389 "
         "attempted-ticks=96994469: Own DSP DMA/F2 cycle P:0x00007b pc=0x000072 "
         "omr=0x600389: Unmapped DSP memory write at Y:0x002000")


def verify_dsp(events, observed):
    release = end = None
    cycles = retirements = 0
    periods = collections.Counter()
    last = 0
    pctl = 0x04401d
    clear = []
    setup_after_pll = []
    pll_changed = False
    for line in events.splitlines():
        if line.startswith("U "):
            assert release is None and line == f"U {RELEASE_TICK} 0xb98a 0x027a 0x00 low=0"
            release = RELEASE_TICK
            end = release + 2 * 45
            tick = release
        elif line.startswith("DSP "):
            assert re.fullmatch(r"DSP cycle=\d+ end=\d+ period=\d+ pc=0x[0-9a-f]{6} "
                                r"pctl=0x[0-9a-f]{6} retired=[01] native=[01] "
                                r"native_pc=0x[0-9a-f]{6} pending=\d+ r4=\d+ lc=\d+", line)
            item = record(line)
            tick = item["end"]
            period = item["period"]
            assert release is not None and period == (45 if pctl == 0x04401d else 10)
            assert item["cycle"] == cycles + 1 and tick == end + period
            assert item["pctl"] in (0x04401d, 0x02e112)
            if pctl != item["pctl"]:
                assert not pll_changed and pctl == 0x04401d and item["pctl"] == 0x02e112
                pll_changed = True
            pctl = item["pctl"]
            cycles += 1
            retirements += item["retired"]
            periods[period] += 1
            end = tick
            if period == 10:
                if item["native"] and item["native_pc"] == 0x72:
                    clear.append(item)
                else:
                    assert not clear
                    setup_after_pll.append(item)
        elif line.startswith("B "):
            tick = int(line.split()[1])
        else:
            assert re.fullmatch(r"cycle=\d+ write=0x[0-9a-f]+:0x[0-9a-f]+", line)
            continue
        assert tick >= last
        last = tick
    # Declared native publication: AGU, X, Y, normal PCU, REP PCU, retirement.
    # The pending types after those accepted effects are X/Y writes, two PCU
    # updates, hidden retirement and no pending instruction, respectively.
    pair_count = 0x2000
    assert len(clear) == pair_count * 6 + 2 and len(setup_after_pll) == 5
    pending = [1, 1, 11, 11, 4, 255]
    for index, item in enumerate(clear):
        pair, phase = divmod(index, 6)
        assert item["r4"] == pair + 1
        assert item["lc"] == 0x20000 - pair - int(phase >= 4)
        assert item["pending"] == pending[phase] and item["retired"] == int(phase == 5)
        assert item["pc"] == (0x73 if phase == 3 else 0x72)
    assert cycles == observed["units"] == observed["dsp_attempts"]
    assert retirements == observed["dsp_retired"] and observed["clk2"] == cycles // 2
    assert periods == {45: 204125, 10: pair_count * 6 + 7}
    elapsed = release + 2 * 45 + sum(period * count for period, count in periods.items())
    assert observed["reset_ticks"] == release + 2 * 45 and elapsed == observed["dsp_ticks"]
    assert end == elapsed == observed["common"] + 1
    return {"native_periods": cycles, "period_counts": dict(periods),
            "reset_ticks": observed["reset_ticks"], "elapsed_ticks": elapsed,
            "expected_CLK2_edges": cycles // 2, "last_native_end": end,
            "independent_complete_XY_pairs": pair_count,
            "accepted_failed_pair_prefix": ["AGU", "X"], "next_rejected_effect": "Y"}


def verify(trace):
    text = trace.read_text()
    rows = text.splitlines()
    assert len(rows) == 5 and rows[-1] == "REPLAY native=1 ram=1 events=1"
    held = single(text, "HELD_CUT")
    assert held == NATIVE_HELD
    prefix = Path(str(trace) + ".prefix.trace").read_text()
    assert single(prefix, "ASSERT") == {"tick": ASSERT_TICK, "completed": ASSERT_TICK + 18,
                                        "pc": 0xb947, "retired": 90322}
    ticks_at, _ = clock_ledger(prefix)
    held_costs = verify_mcu(Path(str(trace) + ".held"), held, ticks_at, 1)
    cases = []
    for index, line in enumerate(rows[1:4]):
        assert line.startswith("STOP ") and line.split(" fault=", 1)[1] == FAULT
        observed = record(line)
        assert observed == {**POSITIVE, "replay": index, "unused_pulldowns": int(index != 0)}
        output = Path(str(trace) + "." + str(index))
        event_bytes = Path(str(output) + ".trace").read_bytes()
        events = event_bytes.decode()
        ticks_at, _ = clock_ledger(prefix + events)
        cases.append({"observed": observed,
                      "mcu": verify_mcu(output, observed, ticks_at, 3),
                      "dsp": verify_dsp(events, observed),
                      "wire": verify_wire(events, ticks_at)})
    for suffix in (".trace", ".mcu.jsonl", ".phases.json"):
        expected = Path(str(trace) + ".0" + suffix).read_bytes()
        assert all(expected == Path(str(trace) + f".{index}" + suffix).read_bytes()
                   for index in (1, 2))
    return {"scope": "Original reset-net/SPI with native DSP from initial held reset at zero; document ROM "
                     "and explicit lab inputs; actual DSP power-on, physical timing and full boot open",
            "held": held, "held_manual_costs": held_costs, "cases": cases,
            "physical_traffic": False,
            "complete_instrument_boot": False, "native_core": "dsp56300"}


def diagnostics(returncode, stdout, stderr):
    assert returncode == 0 and stdout == SUCCESS
    expected = [f"Original native failure replay={index} mcu=0xb9ad dsp=0x000072 "
                f"frames=97 writes=7275 reason={FAULT}" for index in range(3)]
    assert stderr.splitlines() == expected


def main():
    binary, firmware, rom, destination = map(Path, sys.argv[1:])
    if not check_inputs(firmware, rom, Path(__file__).resolve().parents[1]):
        return 77
    frozen = {str(path): identity(path) for path in (binary, firmware, rom)}
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=destination.stem + "-", suffix=".trace",
                                        dir=destination.parent)
    os.close(descriptor)
    trace = Path(name)
    result = subprocess.run([str(binary.resolve()), str(firmware), str(rom), str(trace)],
                            capture_output=True, text=True, check=False)
    Path(str(trace) + ".stdout").write_text(result.stdout)
    Path(str(trace) + ".stderr").write_text(result.stderr)
    diagnostics(result.returncode, result.stdout, result.stderr)
    verified = verify(trace)
    assert all(identity(Path(name)) == expected for name, expected in frozen.items())
    verified["inputs"] = frozen
    Path(str(trace) + ".verification.json").write_text(json.dumps(verified, indent=2) + "\n")
    # Retain each run's complete unique sidecars; this index selects only a verified run.
    destination.write_text(json.dumps({"verified_trace": str(trace),
                                       "verification": str(trace) + ".verification.json"}) + "\n")
    print("Original reset-net/SPI/dsp56300 and held replay verified; full instrument boot remains open")
    return 0


if __name__ == "__main__":
    sys.exit(main())
