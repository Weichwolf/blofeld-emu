# SPDX-License-Identifier: GPL-3.0-only
"""Run the document-derived reset-prefix pipeline check with its local identity pinned."""
import hashlib
from pathlib import Path
import subprocess
import sys


def main():
    binary, rom, trace = map(Path, sys.argv[1:])
    if not rom.is_file():
        print("SKIP: private document-derived SPI bootstrap prefix absent")
        return 77
    data = rom.read_bytes()
    if len(data) != 165 or hashlib.sha256(data).hexdigest() != (
            "26531751e85d9d796795e443869330c84689b6bdb1eccbafc2cbd1554e377f16"):
        raise ValueError("Document-derived SPI bootstrap prefix identity does not match")
    trace.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.run([str(binary.resolve()), "bootstrap", str(rom), str(trace)],
                          check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
