# SPDX-License-Identifier: GPL-3.0-only
"""Pin all original records before native note-dispatch boundary checks."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def validate(result):
    rows = re.findall(r"^FAULT note=(\d+) velocity=(\d+) channel=(\d+) (.+)$", result, re.M)
    expected = [(note, velocity, 0) for note in (0, 64, 127) for velocity in (1, 64, 127)]
    expected += [(64, 64, channel) for channel in range(16)] + [(64, 64, 0)] * 2
    if [tuple(map(int, row[:3])) for row in rows] != expected:
        raise ValueError("Incomplete original note boundary matrix")
    for *_, error in rows:
        if ("pc=0x0004ce" not in error or "omr=0x600389" not in error
                or "Unimplemented DSP peripheral read at Y:0xffffff" not in error):
            raise ValueError("Changed original note boundary context")
    controls = re.findall(r"^CONTROL status=(\d+) note=(\d+) velocity=(\d+) selector=(\d+)$",
                          result, re.M)
    expected_controls = [(status, note, velocity, 0) for note in (0, 64, 127)
                         for status, velocity in ((144, 0), (128, 64))]
    expected_controls += [(144 | channel, 64, 64, (channel + 1) % 16 + 1)
                          for channel in range(16)] + [(144, 64, 0, 1)]
    if [tuple(map(int, row)) for row in controls] != expected_controls:
        raise ValueError("Incomplete returned original note controls")
    summary = "SUMMARY allocation-frontiers=27 completed-controls=23 fragmented-prefixes=2"
    if result.splitlines()[-1:] != [summary]:
        raise ValueError("Missing completed original note controls")
    return dict(allocation_frontiers=27, completed_controls=23, fragmented_prefixes=2)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original note regression: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(trace.resolve())], check=False, timeout=110)
    if result.returncode:
        return result.returncode
    totals = validate(trace.read_text())
    report.write_text(json.dumps({"passed": True,
                                  "scope": "Original note dispatch to a strict reserved-read fault; "
                                           "allocation completion and MIDI-to-audio remain unproven.",
                                  **totals}, indent=2) + "\n")
    print("Verified original note dispatch and reserved-read rejection; "
          "successful allocation, voice lifecycle and audio remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
