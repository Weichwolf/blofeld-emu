# SPDX-License-Identifier: GPL-3.0-only
"""Validate identities of private firmware inputs for component tests."""

import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def original_mcu(firmware, reference):
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("Changed original 1.25 archive")
    midi, _ = load_input(firmware)
    messages, _ = read_smf(midi)
    main = decode_block(split_blocks(messages)[4])[1]
    _, offset = hc12_object(main)
    host = main[offset:offset + 32768]
    if hashlib.sha256(host).hexdigest() != reference["mcu_sha256"]:
        raise ValueError("Changed original MCU image")
    for row in reference["routines"]:
        start = row["entry"] - 0x8000
        data = host[start:start + row["bytes"]]
        if hashlib.sha256(data).hexdigest() != row["sha256"]:
            raise ValueError("Changed original delay/scan helper span")
