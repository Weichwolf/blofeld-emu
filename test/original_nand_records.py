# SPDX-License-Identifier: GPL-3.0-only
"""Prepare one hash-pinned original runtime page; never initialize a physical device."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original NAND records: private firmware absent")
        return 77
    downloads = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in downloads["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("Original firmware identity mismatch")
    sys.path.insert(0, str(ROOT / "tools"))
    from inspect_firmware import decode_block, load_input, read_smf, split_blocks
    midi, _ = load_input(firmware)
    messages, _ = read_smf(midi)
    reference = json.loads((ROOT / "doc/nand-first-records-reference.json").read_text())
    payload = decode_block(split_blocks(messages)[reference["update_block"]])[1]
    start, size = reference["runtime_byte_offset"], reference["page_bytes"]
    page = payload[start:start + size]
    if len(page) != size or hashlib.sha256(page).hexdigest() != reference["page_sha256"]:
        raise ValueError("Original runtime page identity mismatch")
    trace.parent.mkdir(parents=True, exist_ok=True)
    page_file = trace.with_suffix(trace.suffix + ".page.bin")
    page_file.write_bytes(page)
    return subprocess.run([str(binary.resolve()), str(firmware), str(page_file), str(trace)],
                          check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
