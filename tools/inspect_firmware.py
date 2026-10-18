#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Offline Blofeld SMF/SysEx inspection. Never opens a MIDI device."""

import argparse
from collections import Counter
from functools import reduce
import hashlib
import json
from operator import xor
from pathlib import Path
import struct
from zipfile import ZipFile


class Reader:
    def __init__(self, data):
        self.data = data
        self.pos = 0

    def take(self, count):
        if count < 0 or self.pos + count > len(self.data):
            raise ValueError(f"Truncated input at {self.pos:#x}, need {count} bytes")
        result = self.data[self.pos:self.pos + count]
        self.pos += count
        return result

    def byte(self):
        return self.take(1)[0]

    def vlq(self):
        result = 0
        for _ in range(4):
            byte = self.byte()
            result = (result << 7) | (byte & 127)
            if byte < 128:
                return result
        raise ValueError("MIDI variable-length quantity exceeds four bytes")


def read_smf(data):
    """Return complete SysEx messages, without F0 and including F7."""
    source = Reader(data)
    if source.take(4) != b"MThd":
        raise ValueError("Missing MThd")
    header = source.take(int.from_bytes(source.take(4), "big"))
    if len(header) != 6:
        raise ValueError("Expected six-byte SMF header")
    fmt, tracks, division = struct.unpack(">HHH", header)
    if fmt not in (0, 1) or not tracks or (fmt == 0 and tracks != 1):
        raise ValueError("Unsupported SMF track layout")
    messages, events = [], Counter()
    for _ in range(tracks):
        if source.take(4) != b"MTrk":
            raise ValueError("Missing MTrk")
        track = Reader(source.take(int.from_bytes(source.take(4), "big")))
        running, pending, ended = None, None, False
        while track.pos < len(track.data):
            track.vlq()
            status = track.data[track.pos] if track.pos < len(track.data) else 0
            if status >= 128:
                track.byte()
            elif running is not None:
                status = running
            else:
                raise ValueError("Running status without channel status")
            events[f"{status:02x}"] += 1
            if status == 255:
                kind = track.byte()
                payload = track.take(track.vlq())
                if kind == 0x2f:
                    if payload or track.pos != len(track.data):
                        raise ValueError("Invalid End of Track")
                    ended = True
            elif status in (240, 247):
                running = None
                payload = track.take(track.vlq())
                if status == 240:
                    if pending is not None:
                        raise ValueError("Nested SysEx")
                    pending = bytearray()
                elif pending is None:
                    raise ValueError("Unsupported standalone F7 escape")
                pending.extend(payload)
                if pending and pending[-1] == 247:
                    if any(byte >= 128 for byte in pending[:-1]):
                        raise ValueError("Non-seven-bit SysEx payload")
                    messages.append(bytes(pending))
                    pending = None
            elif 128 <= status < 240:
                running = status
                size = 1 if status & 0xf0 in (0xc0, 0xd0) else 2
                if any(byte >= 128 for byte in track.take(size)):
                    raise ValueError("Invalid MIDI channel data")
            else:
                raise ValueError(f"Unsupported MIDI status {status:#x}")
        if pending is not None or not ended:
            raise ValueError("Incomplete MIDI track")
    if source.pos != len(data):
        raise ValueError("Trailing SMF bytes")
    return messages, {"format": fmt, "tracks": tracks, "division": division,
                      "events": dict(events)}


def split_blocks(messages):
    blocks = []
    for message in messages:
        if len(message) < 5 or message[:3] != b"\x3e\x13\x7f":
            raise ValueError("Unexpected firmware SysEx prefix")
        if message[3] == 0x73:
            if len(message) != 14:
                raise ValueError("Invalid block header length")
            blocks.append({"header": message, "packets": []})
        elif message[3] == 0x74 and blocks:
            if len(message) < 8 or len(message) != message[5] + 8:
                raise ValueError("Invalid data packet length")
            expected = len(blocks[-1]["packets"]) % 128
            if message[4] != expected:
                raise ValueError(f"Packet sequence {message[4]} != {expected}")
            if reduce(xor, message[:-1], 0) != 0:
                raise ValueError("Firmware data packet XOR mismatch")
            blocks[-1]["packets"].append(message)
        else:
            raise ValueError(f"Unknown firmware command {message[3]:#x}")
    return blocks


def unpack_payload(packed):
    """Each flag byte contains the following seven bytes' MSBs, high bit first."""
    if len(packed) % 8:
        raise ValueError("Packed payload must contain complete eight-byte groups")
    result = bytearray()
    for pos in range(0, len(packed), 8):
        flags = packed[pos]
        for index, byte in enumerate(packed[pos + 1:pos + 8]):
            if flags >= 128 or byte >= 128:
                raise ValueError("Non-seven-bit packed byte")
            result.append(byte | (((flags >> (6 - index)) & 1) << 7))
    return bytes(result)


def decode_block(block):
    header = block["header"]
    length = sum(byte << (7 * index) for index, byte in enumerate(header[5:9]))
    address = sum(byte << (7 * index) for index, byte in enumerate(header[9:13]))
    packed = b"".join(packet[6:-2] for packet in block["packets"])
    decoded = unpack_payload(packed)
    if length % 3 or len(decoded) != ((length + 6) // 7) * 7:
        raise ValueError("Block payload does not match declared 24-bit byte length")
    # The transport sends each 24-bit word least significant byte first.
    data = decoded[:length]
    big_endian = b"".join(data[pos:pos + 3][::-1] for pos in range(0, length, 3))
    return packed, big_endian, {"mode": header[4], "declared_bytes": length,
                                "address_words": address,
                                "transport_padding_hex": decoded[length:].hex()}


def load_input(path):
    if path.suffix.lower() != ".zip":
        return path.read_bytes(), path.name
    with ZipFile(path) as archive:
        names = [name for name in archive.namelist() if name.lower().endswith(".mid")]
        if len(names) != 1:
            raise ValueError("Expected exactly one MIDI file in archive")
        if archive.getinfo(names[0]).file_size > 16 * 1024 * 1024:
            raise ValueError("MIDI input exceeds inspection size limit")
        return archive.read(names[0]), names[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data, name = load_input(args.input)
    messages, smf = read_smf(data)
    blocks = split_blocks(messages)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"input": str(args.input), "member": name, "bytes": len(data),
              "sha256": hashlib.sha256(data).hexdigest(), "smf": smf,
              "sysex_messages": len(messages), "blocks": []}
    for index, block in enumerate(blocks):
        payload, decoded, fields = decode_block(block)
        stem = f"block-{index:02d}"
        (args.output / f"{stem}.packed").write_bytes(payload)
        (args.output / f"{stem}.be24.bin").write_bytes(decoded)
        info = {"index": index, "header_hex": block["header"].hex(" "),
                "packets": len(block["packets"]), "packed_bytes": len(payload),
                "sha256_packed": hashlib.sha256(payload).hexdigest(),
                "sha256_decoded": hashlib.sha256(decoded).hexdigest(), **fields}
        report["blocks"].append(info)
        print(info)
    (args.output / "transport.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
