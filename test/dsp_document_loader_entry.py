# SPDX-License-Identifier: GPL-3.0-only
"""Check local document main-ROM identity before executing the loader-entry integration test."""
import hashlib
from pathlib import Path
import subprocess
import sys


def main():
    binary, rom, trace = map(Path, sys.argv[1:])
    if not rom.is_file():
        print("SKIP: private document-derived main boot ROM absent")
        return 77
    data = rom.read_bytes()
    if len(data) != 576 or hashlib.sha256(data).hexdigest() != (
            "4d72a5bd75ad086ec7190c6c25b4438aa1cfaa2ddc0ac510055c2540e886de56"):
        raise ValueError("Document-derived main boot ROM identity does not match")
    trace.parent.mkdir(parents=True, exist_ok=True)
    return subprocess.run([str(binary.resolve()), str(rom), str(trace)], check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
