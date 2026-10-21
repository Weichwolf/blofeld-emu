# SPDX-License-Identifier: GPL-3.0-only
"""Pin original peripheral initialization and validate the actual native boundaries."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original peripheral initialization: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([binary.resolve(), firmware.resolve(), trace.resolve()],
                            check=False, timeout=110)
    if result.returncode:
        return result.returncode
    text = trace.read_text()
    expected = "cases=4 cold-faults=2 reset-prefixes=2 fault-replays=12 zero-frame-replays=2"
    if re.findall(r"^RESULT (.+)$", text, re.M) != [expected]:
        raise ValueError("Incomplete original initialization scope")
    cases = re.findall(r"^CASE (\d+) reset=([01]) clock=([01])$", text, re.M)
    if cases != [(str(i), str(i // 2), str(i % 2)) for i in range(4)]:
        raise ValueError("Changed reset/HCKT input matrix")
    faults = re.findall(r"^FAULT (.+)$", text, re.M)
    if len(faults) != 12 or any("pc=0x002eaa" not in row for row in faults[:6]) or any(
            "pc=0x002eb1" not in row for row in faults[6:]):
        raise ValueError("Missing actual original fault/recovery evidence")
    if len(re.findall(r"^WRITE ", text, re.M)) != 60:
        raise ValueError("Incomplete native original register-write prefixes")
    report.write_text(json.dumps({"passed": True, "cases": 4, "fault_replays": 12,
        "zero_frame_replays": 2, "original_writes": 60,
        "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
        "qualification": "Isolated unchanged original callee, own native caller and optional "
                         "declared personal reset/HCKT. Default active-configuration fault retained. "
                         "One original zero-primed two-slot frame is independently decoded twice "
                         "with the CPU paused before the reserved write. No complete boot, "
                         "original caller provenance, DMA progress, physical timing or host audio."},
        indent=2) + "\n")
    print("Verified four original initialization cases and exact fault/frame replays")
    return 0


if __name__ == "__main__":
    sys.exit(main())
