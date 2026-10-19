# SPDX-License-Identifier: GPL-3.0-only
"""Pin a private firmware input and independently check the clocked MCU census."""

from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from hc12_manual_cycle_costs import cycles


MID_SHA256 = "447f7ec928b5b4144fad2193351f0604011f2ab8ed9b9dbe3d1bb7c594e813c0"


def artifact(path: Path) -> dict:
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def verify_reference(path: Path) -> None:
    manifest = json.loads((Path(__file__).resolve().parents[1] / "doc/downloads.json").read_text())
    zip_sha256 = next(row["sha256"] for row in manifest["files"]
                      if row["file"] == "Blofeld_Firmware_V1.25.zip")
    expected = zip_sha256 if path.suffix.lower() == ".zip" else MID_SHA256
    if artifact(path)["sha256"] != expected:
        raise ValueError("The clocked test requires the pinned original ZIP or identical complete MID")


def census(path: Path) -> dict:
    totals = Counter()
    retired = 0
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if len(row["flags"]) != 16 or any(n < 0 for n in row["flags"]):
            raise ValueError("Invalid native PC/CCR census")
        count = sum(row["flags"])
        if not count:
            continue
        retired += count
        totals[row["instruction"].split()[0]] += cycles(row)
    return {"retired": retired, "instruction_cycles": sum(totals.values()),
            "by_mnemonic": dict(sorted(totals.items()))}


def phases(path: Path) -> dict:
    data = json.loads(path.read_text())
    assert data["high"] == data["low"] and not data["pending"], data
    assert sum(word["count"] for word in data["words"]) == data["low"], data
    for word in data["words"]:
        assert 0x4000 <= word["address"] < 0xffff and word["address"] & 1, word
        assert word["count"] > 0, word
    return data


def record(line: str) -> dict:
    return {key: int(value, 0) for key, value in
            (word.split("=", 1) for word in line.split(" fault=", 1)[0].split()[1:])}


def clock_ledger(text: str):
    writes = [(int(c), int(a, 16), int(v, 16)) for c, a, v in
              re.findall(r"^cycle=(\d+) write=(0x[0-9a-f]+):(0x[0-9a-f]+)$", text, re.M)]
    switches = [(c, v) for c, a, v in writes if a == 0x39]
    assert switches == [(75, 0x80), (233898, 0), (233906, 0x80)], switches
    assert all(v == 0 for _, a, v in writes if a == 0x35), "Unreviewed REFDV change"
    assert [(c, v) for c, a, v in writes if a == 0x3b] == [(233874, 0x40)]
    sci_config = [(c, a, v) for c, a, v in writes if 0xc8 <= a <= 0xcb]
    assert sci_config == [(84, 0xc8, 0), (84, 0xc9, 60), (87, 0xca, 8),
                          (90, 0xcb, 12), (233795, 0xcb, 0x2c)], sci_config

    def ticks_at(boundary):
        ticks = 0
        previous = 0
        per_cycle = 10  # 12 MHz OSC, divider two, at the 60 MHz reference timebase.
        for cycle, value in switches:
            if boundary <= cycle:
                break
            ticks += (cycle - previous) * per_cycle
            previous = cycle
            per_cycle = 2 if value == 0x80 else 10
        return ticks + (boundary - previous) * per_cycle

    # Native RTICTL=$40 starts an 8192-OSC-edge period after its write clock.
    configured_edges = ticks_at(233874) // 5
    main_ticks = ticks_at(141809799)
    remaining_edges = 8192 - ((main_ticks // 5 - configured_edges) % 8192)
    expected_idle = (remaining_edges * 5 - main_ticks % 5 + 1) // 2
    assert remaining_edges == 913 and expected_idle == 2282
    return ticks_at, expected_idle


def verify_midi(trace: Path, text: str, level: int, suffix: str, ticks_at) -> dict:
    # The operand clock precedes the first TE/RE enable at boundary90. Native
    # divider residue survives the entire cold start and partial-frame restore.
    epoch, divisor, main = 90, 60, 141809799
    first_fall = main + 960
    first_sample = epoch + ((first_fall + 1 - epoch + divisor - 1) // divisor) * divisor
    decision = first_sample + 153 * divisor
    assert decision - main == 10191
    prefix_cycles, prefix_idle = 141816115, 2282
    strict_line, = [line for line in text.splitlines()
                    if line.startswith(f"midi-strict input={level} ")]
    strict = record(strict_line)
    assert strict["cycles"] == decision
    assert strict["idle"] == prefix_idle + decision - prefix_cycles
    results = []
    for delay in (0, 1, 59, 60, 61, 359, 360):
        lines = [line for line in text.splitlines()
                 if line.startswith(f"midi input={level} delay={delay} ")]
        assert len(lines) == 2
        replays = []
        for attempt, line in enumerate(lines):
            observed = record(line)
            assert observed.pop("replay") == attempt
            name = str(trace) + suffix + f".midi{delay}.r{attempt}"
            costs = census(Path(name + ".census.jsonl"))
            expanded = phases(Path(name + ".phases.json"))
            assert observed["entries"] == observed["returns"]
            assert 0 <= observed["reused"] <= observed["entries"]
            assert observed["serial"] == 8 and observed["first-sci"] == decision + delay
            irq_cycles = observed["reused"] * 6 + (observed["entries"] - observed["reused"]) * 9
            expected = prefix_cycles + observed["idle"] - prefix_idle + irq_cycles + costs["instruction_cycles"] + expanded["low"]
            assert observed["cycles"] == expected >= main + 960 + 8 * 9600
            assert observed["retired"] == costs["retired"]
            assert observed["extra-flash"] == expanded["low"]
            assert observed["ticks"] == ticks_at(expected)
            assert observed["OSC"] == observed["ticks"] // 5
            replays.append({"observed": observed, "manual_costs": costs,
                            "irq_cycles": irq_cycles, "phase_pairs": expanded})
        assert replays[0] == replays[1], "Coordinated native MIDI peer replay changed execution/time"
        results.append({"delay": delay, "replays": replays})
    return {"scope": "Explicit unqualified timing hypotheses; not hardware equivalence",
            "generator_epoch": epoch, "stop_decision": decision,
            "strict_default": strict, "phases": results}


def verify(trace: Path) -> dict:
    text = trace.read_text()
    ticks_at, expected_idle = clock_ledger(text)
    root_line, = [line for line in text.splitlines() if line.startswith("root ")]
    root_record = record(root_line)
    root = census(Path(str(trace) + ".root.census.jsonl"))
    assert phases(Path(str(trace) + ".root.phases.json"))["low"] == 0
    assert root["retired"] == root_record["retired"]
    root_cycles = root["instruction_cycles"] + 5 + root_record["entries"] * 9 + 1
    assert root_cycles == root_record["cycles"]
    assert root_record["ticks"] == ticks_at(root_cycles)
    assert root_record["OSC"] == root_record["ticks"] // 5
    assert "Unknown MCU GPIO input at 0x0248 mask=0x08" in root_line
    result = {"scope": "Manual mnemonic/mode costs and separate Flash phase observations; "
                       "independent of production timing decoder, not independent execution "
                       "path or physical board calibration", "root": root, "cases": []}
    for level, suffix in [(0, ".low"), (8, ".high")]:
        main_line, = [line for line in text.splitlines() if line.startswith(f"main input={level} ")]
        main = record(main_line)
        costs = census(Path(str(trace) + suffix + ".census.jsonl"))
        expanded = phases(Path(str(trace) + suffix + ".phases.json"))
        expected = root_cycles - 1 + costs["instruction_cycles"] + main["entries"] * 9 + expanded["low"]
        assert main["cycles"] == expected and main["retired"] == costs["retired"]
        assert main["extra-flash"] == expanded["low"] == 70 and main["idle"] == 0
        assert main["ticks"] == ticks_at(expected) and main["OSC"] == main["ticks"] // 5
        wake_lines = [line for line in text.splitlines() if line.startswith(f"wake input={level} ")]
        assert len(wake_lines) == 2
        wake_results = []
        for attempt, line in enumerate(wake_lines):
            wake = record(line)
            name = str(trace) + suffix + f".wake{attempt}"
            wake_costs = census(Path(name + ".census.jsonl"))
            wake_phases = phases(Path(name + ".phases.json"))
            # WAI wake and RTI chaining reuse the frame: fVfPPP = 6; ordinary entries cost9.
            assert 1 <= wake["reused"] <= wake["entries"]
            irq_cycles = wake["reused"] * 6 + (wake["entries"] - wake["reused"]) * 9
            expected_wake = expected + expected_idle + irq_cycles + wake_costs["instruction_cycles"] + wake_phases["low"]
            assert wake["cycles"] == expected_wake and wake["retired"] == wake_costs["retired"]
            assert wake["idle"] == expected_idle and wake["extra-flash"] == wake_phases["low"]
            assert wake["ticks"] == ticks_at(expected_wake)
            assert wake["OSC"] == wake["ticks"] // 5
            wake_results.append({"observed": wake, "manual_costs": wake_costs,
                                 "irq_cycles": irq_cycles, "phase_pairs": wake_phases})
        assert wake_results[0] == wake_results[1], "Trusted idle replay changed execution or time"
        result["cases"].append({"input": level, "main": main, "manual_costs": costs,
                                "phase_pairs": expanded, "wake_replays": wake_results,
                                "midi": verify_midi(trace, text, level, suffix, ticks_at)})
    return result


def main() -> int:
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original clocked MCU: private firmware absent")
        return 77
    verify_reference(firmware)
    frozen = {str(path): artifact(path) for path in (binary, firmware)}
    trace.parent.mkdir(parents=True, exist_ok=True)
    code = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                          check=False).returncode
    if code:
        return code
    result = verify(trace)
    assert all(artifact(Path(name)) == expected for name, expected in frozen.items())
    result["inputs"] = frozen
    Path(str(trace) + ".verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Original clocked reset/WAI/MIDI hypotheses: independent cycle/OSC/divider ledger agrees.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
