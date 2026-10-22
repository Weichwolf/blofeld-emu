# SPDX-License-Identifier: GPL-3.0-only
"""Verify original input records and fixed logical-wire execution evidence."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import dsp_records, hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def main() -> int:
    binary, firmware, trace = map(Path, sys.argv[1:4])
    scope = sys.argv[4]
    reference = json.loads((ROOT / "doc/dsp-midi-wire-reference.json").read_text())
    if scope not in reference["scopes"]:
        raise ValueError("Unsupported original MIDI wire scope")
    if not firmware.is_file():
        print("SKIP original MIDI wire reference: private firmware absent")
        return 77
    downloads = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in downloads["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != expected:
        raise ValueError("Changed original firmware archive")
    midi, _ = load_input(firmware)
    if hashlib.sha256(midi).hexdigest() != reference["midi_sha256"]:
        raise ValueError("Changed original firmware MID")
    messages, _ = read_smf(midi)
    main_payload = decode_block(split_blocks(messages)[4])[1]
    _, host_offset = hc12_object(main_payload)
    dsp_offset = host_offset + ((0x8000 + 2) // 3) * 3 + 3
    records, _, _, _ = dsp_records(main_payload[dsp_offset:])
    for block in reference["blocks"]:
        record = records[block["record"]]
        actual = (record["space"], record["address"], record["words"],
                  hashlib.sha256(record["data"]).hexdigest())
        if actual != (block["space"], block["address"], block["words"], block["sha256"]):
            raise ValueError("Changed original wire component: " + block["name"])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(trace.resolve()), scope], check=False, timeout=110)
    if result.returncode:
        return result.returncode
    text = trace.read_text()
    if scope == "download":
        expected = []
        for mode in range(4):
            for command in (0x74, 0x75):
                expected.extend(f"NORMAL {mode} {command} {value}" for value in (0, 127))
                expected.append(f"OMITTED {mode} {command}")
        expected.extend(("REJECTED 4", "FRAGMENTED 1", "RESTARTED 1"))
        rows = text.splitlines()
        if rows[:-1] != expected or not rows[-1].startswith("RESULT "):
            raise ValueError("Incomplete fixed original wire/SysEx/download case matrix")
        observed = dict(token.split("=", maxsplit=1) for token in rows[-1][len("RESULT "):].split())
    else:
        observed = dict(token.split("=", maxsplit=1) for token in text.split())
    if {key: int(value) for key, value in observed.items()} != reference["scopes"][scope]:
        raise ValueError("Incomplete original MIDI wire evidence: " + scope)
    print("Verified original MIDI wire scope: " + scope)
    return 0


if __name__ == "__main__":
    sys.exit(main())
