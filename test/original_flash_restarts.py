# SPDX-License-Identifier: GPL-3.0-only
"""Check original updater exports independently, then cold-start each in a new process."""

import binascii
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

PHASES = ("selected", "launched", "active", "completed")
JOBS = [("erase", 0x8200, None), ("program", 0x8200, 0x1234),
        ("program", 0x8202, 0xabcd)]
JOBS += [(kind, address, value) for _ in range(4)
         for kind, address, value in (("erase", 0xfe00, None), ("program", 0xff0e, 0xfffe))]
JOBS += [("program", address, 0x8000) for address in (0xfffa, 0xfffc, 0xfffe)]


def encoded(data, known, fresh):
    def bits(values):
        return bytes(sum(int(value) << bit for bit, value in enumerate(values[pos:pos + 8]))
                     for pos in range(0, len(values), 8))
    body = b"BLFNVM\r\n" + struct.pack("<II", 1, 38912) + bytes(data) + bits(known) + bits(fresh)
    return body + struct.pack("<I", binascii.crc32(body))


def outcome(data, known):
    if not all(known[pos] for pos in (0x7f0d, 0x7f0f)):
        return "options-unknown"
    if not all(known[-2:]):
        return "vector-unknown"
    vector = int.from_bytes(data[-2:], "big")
    if vector != 0x8000:
        assert vector == 0xffff, hex(vector)
        return "vector-erased"
    if not all(known[0x200:0x7f00]) or not all(known[0x7f10:0x7f12]):
        return "checksum-unknown"
    total = sum(int.from_bytes(data[pos:pos + 2], "big") for pos in range(0x200, 0x7f00, 2)) & 65535
    return "application" if total == int.from_bytes(data[0x7f10:0x7f12], "big") else "recovery"


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, timeout=45, check=False)
    if result.returncode != 0:
        raise ValueError(f"Reference subprocess failed: {command}\n{result.stdout}\n{result.stderr}")
    return result.stdout


def check_cases(producer, consumer, firmware, root, original):
    image_dir = root / "images"
    run([producer, firmware, root / "programming.trace", image_dir])
    expected_names = {f"command-{index}-{phase}.bfnvm" for index in range(1, 15) for phase in PHASES}
    assert {path.name for path in image_dir.iterdir()} == expected_names
    data, known, fresh = bytearray(original), [True] * 32768, [False] * 16384
    counts, cases = Counter(), []
    for index, (kind, address, value) in enumerate(JOBS, start=1):
        offset = address - 0x8000
        first, limit = (offset // 512 * 512, offset // 512 * 512 + 512) if kind == "erase" \
            else (offset, offset + 2)
        for phase in PHASES:
            saved_data, saved_known, saved_fresh = data.copy(), known.copy(), fresh.copy()
            if phase == "active":
                saved_data[first:limit] = bytes(limit - first)
                saved_known[first:limit] = [False] * (limit - first)
                saved_fresh[first // 2:limit // 2] = [False] * ((limit - first) // 2)
            elif phase == "completed":
                saved_data[first:limit] = bytes([255]) * (limit - first) if kind == "erase" \
                    else value.to_bytes(2, "big")
                saved_fresh[first // 2:limit // 2] = [kind == "erase"] * ((limit - first) // 2)
            path = image_dir / f"command-{index}-{phase}.bfnvm"
            assert path.read_bytes() == encoded(saved_data, saved_known, saved_fresh), path.name
            expected = outcome(saved_data, saved_known)
            trace = root / f"command-{index}-{phase}.trace"
            text = run([consumer, firmware, path, trace, expected])
            assert f"outcome={expected}" in text, text
            counts[expected] += 1
            cases.append({"command": index, "phase": phase, "outcome": expected,
                          "image_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                          "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest()})
            print(f"command={index} phase={phase} cold-outcome={expected}", flush=True)
            if phase == "completed":
                data, known, fresh = saved_data, saved_known, saved_fresh
    assert sum(counts.values()) == 56
    assert set(counts) == {"application", "recovery", "checksum-unknown", "options-unknown",
                           "vector-unknown", "vector-erased"}
    return {"cases": cases, "outcomes": dict(counts)}


def recovery_packets(helper, signature):
    # Own framing inferred from the resident receiver, distinct from normal updates.
    padded = helper + bytes((-len(helper)) % 5)
    chunks = [padded[pos:pos + 320] for pos in range(0, len(padded), 320)]
    packets = []
    for sequence, chunk in enumerate(chunks):
        packet = bytearray(signature + bytes([len(chunk) // 5, 0, len(chunks) - 1, 0, sequence]))
        for pos in range(0, len(chunk), 5):
            group = chunk[pos:pos + 5]
            packet.append(sum((byte & 1) << index for index, byte in enumerate(group)))
            packet.extend(byte >> 1 for byte in group)
        packet.append(0xf7)
        packets.append(bytes(packet))
    return packets


def check_wire(producer, consumer, firmware, root, blocks, original):
    image_dir = root / "images"
    run([producer, firmware, root / "programming.trace", image_dir])
    image = image_dir / "command-14-completed.bfnvm"
    helper, _ = hc12_object(decode_block(blocks[0])[1])
    assert len(helper) == 761
    packets = recovery_packets(helper, original[0x194:0x199])
    valid = b"".join(packets)
    ordinary = b"".join(b"\xf0" + message for message in [blocks[0]["header"], *blocks[0]["packets"]])
    cases = (("valid", valid, "receiver-transfer"),
             ("interrupted", packets[0], "receiver-wait"),
             ("ordinary-update", ordinary, "receiver-wait"),
             ("reordered", b"".join(reversed(packets)), "receiver-mismatch"))
    record = []
    for name, wire, expected in cases:
        path = root / f"{name}.syx"
        trace = root / f"{name}.trace"
        path.write_bytes(wire)
        text = run([consumer, firmware, image, trace, expected, path])
        assert f"outcome={expected}" in text, text
        record.append({"case": name, "outcome": expected, "wire_bytes": len(wire),
                       "wire_sha256": hashlib.sha256(wire).hexdigest(),
                       "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest()})
        print(f"wire-case={name} outcome={expected}", flush=True)
    return {"wire_cases": record, "helper_bytes": len(helper), "origin": "own recovery framing"}


def main():
    if len(sys.argv) not in (5, 6) or (len(sys.argv) == 6 and sys.argv[5] != "--wire-only"):
        raise ValueError("Expected producer consumer firmware output [--wire-only]")
    wire_only = len(sys.argv) == 6
    producer, consumer, firmware, output = map(Path, sys.argv[1:5])
    if not firmware.is_file():
        print("SKIP original updater cold restarts: private firmware absent")
        return 77
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    assert hashlib.sha256(firmware.read_bytes()).hexdigest() == expected
    midi, _ = load_input(firmware)
    blocks = split_blocks(read_smf(midi)[0])
    main_payload = decode_block(blocks[4])[1]
    _, host_offset = hc12_object(main_payload)
    original = main_payload[host_offset:host_offset + 32768]
    assert len(original) == 32768
    output.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="original-restarts-", dir=output))
    if wire_only:
        record = check_wire(producer.resolve(), consumer.resolve(), firmware.resolve(),
                            directory, blocks, original)
    else:
        record = check_cases(producer.resolve(), consumer.resolve(), firmware.resolve(),
                             directory, original)
    record["private_directory"] = str(directory)
    filename = "original-recovery-wire.json" if wire_only else "original-flash-restarts.json"
    (output / filename).write_text(json.dumps(record, indent=2) + "\n")
    print("Verified original recovery wire cases" if wire_only
          else f"Verified 56 original-command cold restarts: {record['outcomes']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
