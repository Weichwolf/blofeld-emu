# SPDX-License-Identifier: GPL-3.0-only
"""Pin original records before native voice execution and independent checks."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from dsp_voice_validation import validate

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import dsp_records, hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def original_records(firmware, reference):
    downloads = json.loads((ROOT / "doc/downloads.json").read_text())
    pinned = next(row for row in downloads["files"]
                  if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if (firmware.stat().st_size != pinned["bytes"]
            or hashlib.sha256(firmware.read_bytes()).hexdigest() != pinned["sha256"]):
        raise ValueError("Changed original firmware archive")
    midi, _ = load_input(firmware)
    if hashlib.sha256(midi).hexdigest() != reference["midi_sha256"]:
        raise ValueError("Changed original firmware MID")
    messages, _ = read_smf(midi)
    payload = decode_block(split_blocks(messages)[4])[1]
    _, offset = hc12_object(payload)
    dsp_offset = offset + ((0x8000 + 2) // 3) * 3 + 3
    records, _, _, _ = dsp_records(payload[dsp_offset:])
    if len(records) != len(reference["records"]):
        raise ValueError("Changed original voice record count")
    xdata, ydata = {}, {}
    for number, (record, expected) in enumerate(zip(records, reference["records"])):
        observed = (number, record["space"], record["address"], record["words"],
                    hashlib.sha256(record["data"]).hexdigest())
        if observed != (expected["record"], expected["space"], expected["address"],
                        expected["words"], expected["sha256"]):
            raise ValueError("Changed original voice record: " + str(number))
        if record["space"] in (1, 2):
            target = xdata if record["space"] == 1 else ydata
            data = record["data"]
            target.update({record["address"] + index // 3:
                           int.from_bytes(data[index:index + 3], "big")
                           for index in range(0, len(data), 3)})
    return xdata, ydata


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original voice regression: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    xdata, ydata = original_records(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(trace.resolve())], check=False, timeout=280)
    if result.returncode:
        return result.returncode
    totals = validate(trace.read_text(), xdata, ydata, reference)
    report.write_text(json.dumps({"passed": True, "scope": reference["qualification"],
                                  "streams": reference["streams"],
                                  "blocks_per_stream": reference["blocks_per_stream"],
                                  **totals}, indent=2) + "\n")
    print(f"Verified original voice streams: {totals['blocks']} complete returns, "
          f"{totals['dependent_blocks']} dependent blocks; "
          "full boot, MIDI allocation, PCM transport and board timing remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
