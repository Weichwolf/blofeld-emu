# SPDX-License-Identifier: GPL-3.0-only
"""Replay actual original boot from a selected partial SPI word to the strict DSP fault."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_reference_cli import CONFIG, ROOT, audit, identity
from test_reference_cli import replay_sections


def main():
    binary, firmware, rom, trace = (p.resolve() for p in map(Path, sys.argv[1:]))
    if not firmware.is_file() or not rom.is_file():
        print("SKIP original joint checkpoint: private firmware/ROM absent")
        return 77
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"] if row["file"] == firmware.name)
    assert identity(firmware) == {"bytes": 102986, "sha256": expected}
    assert identity(rom) == {"bytes": 165,
                           "sha256": "26531751e85d9d796795e443869330c84689b6bdb1eccbafc2cbd1554e377f16"}
    trace.parent.mkdir(parents=True, exist_ok=True)
    config = trace.with_suffix(".bfboard")
    # A completed native MCU operation inside a selected, partly clocked word.
    config.write_text(CONFIG + "checkpoint 1750000 2\n")
    result = subprocess.run([binary, "--firmware", firmware, "--reference-board", config,
                             "--dsp-rom", rom, "--instructions", "2000000", "--trace", trace],
                            capture_output=True, text=True, timeout=240, check=False)
    trace.with_suffix(".log").write_text(result.stdout + result.stderr)
    assert result.returncode == 2, result.stderr
    assert "Unmapped DSP memory write at Y:0x002000" in result.stderr
    assert "replay changed" not in result.stderr
    assert "r4=8193 lc=122880" in result.stderr
    assert "instructions=1857015 operations=1857015" in result.stderr
    assert "cycles=5365748 idle=0 ticks=10732160" in result.stderr
    assert "completed-steps=59286 units=212426 ticks=9272275" in result.stderr
    assert "spi-writes=7275 frames=97 ordered-pairs=1 ties=368" in result.stderr
    prefix, branches = replay_sections(trace.read_text())
    assert len(branches) == 3 and branches == [branches[0]] * 3
    writes = [line for line in prefix.splitlines() if line.startswith("SPI ")]
    last = writes[-1]
    value = int(re.search(r"value=(0x[0-9a-f]+)", last)[1], 16)
    assert value & 2 == 0 and value & 4 != 0, last
    assert "frames=9" in last
    assert prefix.splitlines()[-1].startswith("MCU end=91435878 pc=0xb9c5 instructions=1750000")
    baseline = trace.with_suffix(".baseline.trace")
    baseline.write_text(prefix + branches[0])
    # Every previous baseline row agrees after the documented short-BRA ROM PC relocation.
    assert identity(baseline) == {
        "bytes": 139392017,
        "sha256": "e1c8e454cfb2bfc9744b3cb90ce3e82225044291233e5581e692060f9597084d"}
    report = {"binary": identity(binary), "firmware": identity(firmware), "rom": identity(rom),
              "config": identity(config), "trace": identity(trace), "baseline": audit(baseline, firmware),
              "checkpoint_operation": 1750000, "checkpoint_common_ticks": 91435878,
              "selected": True, "clock_high": True, "completed_frames_before_capture": 9,
              "identical_suffixes": len(branches), "suffix_bytes": len(branches[0].encode()),
              "exit_code": result.returncode}
    trace.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print("Native original mid-SPI checkpoint replays identical suffixes and the actual reserved-Y fault")
    return 0


if __name__ == "__main__":
    sys.exit(main())
