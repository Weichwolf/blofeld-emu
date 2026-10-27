# SPDX-License-Identifier: GPL-3.0-only
"""Check conditional original reset/SPI/pipeline execution, not complete device boot."""

import collections
import hashlib
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
from original_reset_start import ASSERT_TICK, HELD, RELEASE_TICK, RESET_TICKS
from original_reset_start import single, verify_mcu, verify_wire
from original_scheduled_coupling import clock_ledger

TRACE_SHA256 = "2c4b6b0eba774bce3fa1c2231a0b1f8dcf99a5424b18a7ae98bdc35053d783e6"
SUCCESS = "Original reset-net source, automatic native pipeline, SPI and held replay passed.\n"
POSITIVE = {"release": RELEASE_TICK, "retired": 1856961, "cycles": 5365468,
            "ticks": 10731600, "OSC": 2146320, "entries": 0, "common": 96584400,
            "mcu": 0xb9ad, "dsp": 0x72, "r4": 8193, "lc": 122881, "omr": 0x600389,
            "units": 212313, "dsp_ticks": 92371103, "reset_ticks": RESET_TICKS,
            "dsp_attempts": 212313, "frames": 97, "writes": 7275, "pairs": 1, "nvm": 1}
FAULT = ("HC12 cycle execution pc=0xb9ad ccr=0xd8 completed-cycles=5365468: "
         "Clock/operand q address=unreconstructed-prefetch/internal: "
         "Scheduled pipeline peer mcu-pc=0xb9ad dsp-pc=0x000072 mcu-kind=q "
         "mcu-address=none mcu-end=96584418: DSP fetch completed-cycles=212313 "
         "DMA-index=212313 ticks=92371103 pc=0x000072 omr=0x600389: "
         "DSP bus-cycle preflight pc=0x000072 omr=0x600389: "
         "Native memory E1 write X:0x002000 and write Y:0x002000 pc=0x000072 "
         "omr=0x600389: Unmapped DSP memory write at Y:0x002000")


def verify_dsp(events, observed):
    release = end = None
    cycles = 0
    periods = collections.Counter()
    last = ASSERT_TICK
    for line in events.splitlines():
        if line.startswith("U "):
            assert release is None and line == f"U {RELEASE_TICK} 0xb98a 0x027a 0x00 low=0"
            release = RELEASE_TICK
            end = release + 2 * 45
            tick = release
        elif line.startswith("DSP "):
            assert re.fullmatch(r"DSP cycle=\d+ end=\d+ period=\d+ pc=0x[0-9a-f]{6} "
                                r"pctl=0x[0-9a-f]{6} retired=[01]", line)
            item = record(line)
            tick = item["end"]
            period = item["period"]
            assert release is not None and period in (10, 45)
            assert item["cycle"] == cycles + 1 and tick == end + period
            assert item["pctl"] in (0x04401d, 0x02e112)
            cycles += 1
            periods[period] += 1
            end = tick
        elif line.startswith("B "):
            tick = int(line.split()[1])
        else:
            assert re.fullmatch(r"cycle=\d+ write=0x[0-9a-f]+:0x[0-9a-f]+", line)
            continue
        assert tick >= last
        last = tick
    assert cycles == observed["units"] == observed["dsp_attempts"]
    assert periods == {45: 204115, 10: 8198}
    reset = release - ASSERT_TICK + 2 * 45
    elapsed = reset + sum(period * count for period, count in periods.items())
    assert reset == observed["reset_ticks"] and elapsed == observed["dsp_ticks"]
    assert end == ASSERT_TICK + elapsed == observed["common"] - 1
    return {"native_periods": cycles, "period_counts": dict(periods),
            "reset_ticks": reset, "elapsed_ticks": elapsed,
            "expected_CLK2_edges": cycles // 2, "last_native_end": end}


def verify(trace):
    text = trace.read_text()
    rows = text.splitlines()
    assert len(rows) == 5 and rows[-1] == "REPLAY native=1 ram=1 events=1"
    held = single(text, "HELD_CUT")
    assert held == HELD
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
        assert hashlib.sha256(event_bytes).hexdigest() == TRACE_SHA256
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
    return {"scope": "Conditional original reset-net/pipeline/SPI experiment; document ROM and "
                     "explicit lab inputs; pre-assert DSP power-on, physical timing and full boot open",
            "held": held, "held_manual_costs": held_costs, "cases": cases,
            "trace_sha256": TRACE_SHA256, "fuzz_execution": False, "physical_traffic": False}


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
    print("Original reset-net/SPI/pipeline and held replay verified; full instrument boot remains open")
    return 0


if __name__ == "__main__":
    sys.exit(main())
