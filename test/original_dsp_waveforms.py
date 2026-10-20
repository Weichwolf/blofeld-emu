# SPDX-License-Identifier: GPL-3.0-only
"""Check pinned original blocks independently before native waveform execution."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import dsp_records, hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original DSP waveforms: private firmware absent")
        return 77
    downloads = json.loads((ROOT / "doc/downloads.json").read_text())
    pinned = next(row for row in downloads["files"] if row["file"] == firmware.name)
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != pinned["sha256"]:
        raise ValueError("Changed original firmware archive")
    reference = json.loads((ROOT / "doc/dsp-waveform-reference.json").read_text())
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
        if (record["space"], record["address"], record["words"],
                hashlib.sha256(record["data"]).hexdigest()) != (
                    block["space"], block["address"], block["words"], block["sha256"]):
            raise ValueError("Changed original waveform block: " + block["name"])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, timeout=110)
    if result.returncode:
        return result.returncode
    lines = trace.read_text().splitlines()
    rows = dict(part.split("=", maxsplit=1) for part in lines[0].split())
    observations = {key: int(value) for key, value in rows.items()}
    if observations.pop("instructions", 0) <= 0 or observations != reference["counts"]:
        raise ValueError("Incomplete original waveform generation counts")
    rows = {"visited": lines[1].removeprefix("visited ")}
    coverage = reference["coverage"]
    visited = {int(value, 16) for value in rows["visited"].split()}
    encoded = "".join(f"{pc:06x}\n" for pc in sorted(visited)).encode("ascii")
    if (len(visited) != coverage["instruction_starts"]
            or hashlib.sha256(encoded).hexdigest() != coverage["sha256"]):
        raise ValueError("Incomplete original filter instruction coverage")
    print(f"Original waveform coverage: {len(visited)} pinned instruction starts; "
          f"complete original recurrence and harmonic stages.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
