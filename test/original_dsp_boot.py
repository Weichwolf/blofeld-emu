# SPDX-License-Identifier: GPL-3.0-only
"""Run the optional private DSP reference with identities checked before execution."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main(extra=()) -> int:
    binary, firmware, rom, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not rom.is_file():
        print("SKIP DSP reference: private firmware/document-derived ROM prefix absent")
        return 77
    manifest = json.loads((Path(__file__).resolve().parents[1] / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("The original 1.25 firmware identity does not match")
    prefix_sha256 = "8d5193c52b22cfdd633bcdd2acf8651a498e361ff06e4716454fddf92be11901"
    if hashlib.sha256(rom.read_bytes()).hexdigest() != prefix_sha256:
        raise ValueError("Document-derived SPI bootstrap prefix identity does not match")
    trace.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.run([str(binary.resolve()), str(firmware), str(rom), str(trace), *extra],
                          check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
