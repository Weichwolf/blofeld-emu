# SPDX-License-Identifier: GPL-3.0-only
"""Pin the complete private MIDI fixture without distributing its program data."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    generator, firmware, output = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP MIDI ROM identity: private original archive absent")
        return 77
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / "doc/downloads.json").read_text())
    expected = next(row for row in manifest["files"] if row["file"] == firmware.name)
    source = firmware.read_bytes()
    assert len(source) == expected["bytes"]
    assert hashlib.sha256(source).hexdigest() == expected["sha256"]
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([generator, firmware, output], capture_output=True, text=True,
                            timeout=30, check=False)
    assert result.returncode == 0 and not result.stderr, result.stdout + result.stderr
    encoded = output.read_bytes()
    assert len(encoded) == 28584
    assert hashlib.sha256(encoded).hexdigest() == (
        "c19ef03e5c527718a82c66f229739247a2a07e3a328ab6ea334a0443f22a90b8")
    assert firmware.read_bytes() == source
    print("Complete private MIDI ROM retains its independently qualified authored loader and data identity")
    return 0


if __name__ == "__main__":
    sys.exit(main())
