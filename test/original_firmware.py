# SPDX-License-Identifier: GPL-3.0-only
"""Check the original firmware identity before a private CPU execution experiment."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main() -> int:
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original CPU reference: private firmware absent")
        return 77
    manifest = json.loads((Path(__file__).resolve().parents[1] / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("The original 1.25 firmware identity does not match")
    trace.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.run([str(binary.resolve()), str(firmware), str(trace)], check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
