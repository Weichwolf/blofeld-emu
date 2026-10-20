# SPDX-License-Identifier: GPL-3.0-only
"""Constrain explicit HREQ/PU0 hypotheses with unchanged original native cold boot."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_reference_cli import CONFIG, ROOT, identity


def run_case(binary, firmware, rom, directory, name, release, polarity, instructions):
    config = CONFIG.replace("dsp-release 87317172", f"dsp-release {release}")
    config = config.replace("initial drive U 3f 00 00", "initial drive U 3e 00 00")
    config = config.replace("at 87322572 drive U 3f 01 00\n", f"shi-hreq U 01 {polarity}\n")
    board, trace = directory / f"{name}.bfboard", directory / f"{name}.trace"
    board.write_text(config)
    result = subprocess.run([binary, "--firmware", firmware, "--reference-board", board,
                             "--dsp-rom", rom, "--instructions", str(instructions), "--trace", trace],
                            capture_output=True, text=True, timeout=180, check=False)
    log = result.stdout + result.stderr
    (directory / f"{name}.log").write_text(log)
    assert "spi-writes=0 frames=0" in log
    returns = []
    with trace.open() as stream:
        for line in stream:
            assert not line.startswith("SPI end="), line
            if line.startswith("SHI-RETURN") or line.startswith("DSP-RELEASE"):
                returns.append(line.strip())
    assert any("signal=hreq driven=0 level=unknown" in line for line in returns)
    assert any(line.startswith(f"DSP-RELEASE requested={release}") for line in returns)
    if name == "early-inverted":
        assert result.returncode == 2, log
        assert "SHI hreq return pad observation mcu-pc=0xb9c2 dsp-pc=0xff001b" in log
        assert "MCU GPIO output contention at 0x0278" in log
        assert "instructions=1672362 operations=1672362" in log
    elif name == "late-inverted":
        assert result.returncode == 2, log
        assert "Unknown MCU GPIO input at 0x0278 mask=0x01" in log
        assert "MCU pc=0xb98d" in log
    elif name == "early-direct":
        assert result.returncode == 0, log
        assert f"instructions={instructions}" in log
        assert "boot completion is not established" in log
        assert any("signal=hreq driven=1 level=0" in line for line in returns)
    else:
        raise ValueError(name)
    release_event = next(line for line in returns if line.startswith("DSP-RELEASE"))
    actual = int(re.search(r"actual=(\d+)", release_event)[1])
    return {"name": name, "release_requested": release, "release_actual": actual,
            "polarity": polarity, "instruction_budget": instructions, "exit_code": result.returncode,
            "observations": returns, "trace": identity(trace), "config": identity(board)}


def main():
    binary, firmware, rom, directory = (path.resolve() for path in map(Path, sys.argv[1:]))
    if not firmware.is_file() or not rom.is_file():
        print("SKIP original SHI routes: private firmware/ROM input absent")
        return 77
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"] if row["file"] == firmware.name)
    assert identity(firmware) == {"bytes": 102986, "sha256": expected}
    assert identity(rom) == {"bytes": 174,
                           "sha256": "8d5193c52b22cfdd633bcdd2acf8651a498e361ff06e4716454fddf92be11901"}
    directory.mkdir(parents=True, exist_ok=True)
    cases = [("early-inverted", 87310000, "inverted", 2000000),
             ("late-inverted", 87317172, "inverted", 2000000),
             ("early-direct", 87310000, "direct", 1672500)]
    reports = [run_case(binary, firmware, rom, directory, *case) for case in cases]
    report = {"binary": identity(binary), "firmware": identity(firmware), "rom": identity(rom),
              "cases": reports,
              "scope": "Explicit nominal clock/reset/logical route hypotheses; no physical pin-role, buffer, pull, reset or timing qualification"}
    (directory / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print("Unchanged original native boot rejects early/late inverted and non-ready direct HREQ/PU0 scenarios")
    return 0


if __name__ == "__main__":
    sys.exit(main())
