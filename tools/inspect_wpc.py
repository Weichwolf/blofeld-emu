#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Inspect the pinned local WPC reference without publishing its data or mask."""

import hashlib
import json
from io import BytesIO
from pathlib import Path
import struct
import sys
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
PAGE_BYTES = 528


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def matches(data, expected):
    return (len(data) == expected["bytes"]
            and hashlib.sha256(data).hexdigest() == expected["sha256"])


def read_pinned(path, expected):
    require(path.stat().st_size == expected["bytes"], f"Changed reference size: {path.name}")
    with path.open("rb") as stream:
        data = stream.read(expected["bytes"] + 1)
    require(matches(data, expected), f"Changed reference SHA-256: {path.name}")
    return data


def macho_bytes(data, address, length):
    require(len(data) >= 32 and length > 0 and address >= 0, "Invalid Mach-O input/span")
    header = struct.unpack_from("<8I", data)
    require(header[0] == 0xfeedfacf and header[1] == 0x1000007,
            "Expected a little-endian 64-bit x86 Mach-O reference")
    count, command_bytes = header[4:6]
    end = 32 + command_bytes
    require(0 < count <= 128 and count * 8 <= command_bytes and end <= len(data),
            "Invalid Mach-O command extent")
    cursor = 32
    found = []
    for _ in range(count):
        require(cursor + 8 <= end, "Truncated Mach-O command")
        command, size = struct.unpack_from("<II", data, cursor)
        require(size >= 8 and size % 8 == 0 and cursor + size <= end,
                "Invalid Mach-O command size")
        if command == 0x19:
            require(size >= 72, "Truncated Mach-O segment")
            segment = struct.unpack_from("<16s4Q4I", data, cursor + 8)
            vmaddr, vmsize, fileoff, filesize = segment[1:5]
            require(size == 72 + segment[7] * 80 and filesize <= vmsize
                    and fileoff + filesize <= len(data), "Invalid Mach-O segment extent")
            if vmaddr <= address and address + length <= vmaddr + filesize:
                offset = fileoff + address - vmaddr
                found.append(data[offset:offset + length])
        cursor += size
    require(cursor == end and len(found) == 1, "Missing or ambiguous file-backed Mach-O span")
    return found[0]


def reference_mask(root, reference, downloads):
    expected = next(row for row in downloads if row["file"] == reference["archive"])
    archive = root / "doc" / expected["file"]
    data = read_pinned(archive, expected)
    member = reference["member"]
    with ZipFile(BytesIO(data)) as source:
        info = source.getinfo(member)
        require(info.file_size == reference["bytes"], "Changed Spectre executable member size")
        data = source.read(member)
    require(matches(data, reference), "Changed Spectre executable member SHA-256")
    return macho_bytes(data, reference["mask_address"], PAGE_BYTES)


def decode_pages(data, mask):
    require(len(mask) == PAGE_BYTES, "WPC page mask must contain exactly 528 bytes")
    require(data and len(data) % PAGE_BYTES == 0, "WPC input must contain complete 528-byte pages")
    return bytes(value ^ mask[index % PAGE_BYTES] for index, value in enumerate(data))


def inspect(root=ROOT):
    reference = json.loads((root / "doc/wpc-reference.json").read_text())
    downloads = json.loads((root / "doc/downloads.json").read_text())["files"]
    masks = [reference_mask(root, row, downloads) for row in reference["spectre"]]
    require(len(masks) == 2 and masks[0] == masks[1]
            and matches(masks[0], reference["mask"]), "Changed original Spectre page-mask identity")
    entries = json.loads((root / "doc/library/sources.json").read_text())["files"]
    expected = next(row for row in entries if row["file"] == reference["wpc_file"])
    encoded = read_pinned(root / "doc/library" / expected["file"], expected)
    decoded = decode_pages(encoded, masks[0])
    require(matches(decoded, reference["decoded"]), "Changed decoded fixed WPC reference")
    summary = {"encoded_bytes": len(encoded), "encoded_sha256": expected["sha256"],
               "decoded_bytes": len(decoded), "decoded_sha256": reference["decoded"]["sha256"],
               "pages": len(decoded) // PAGE_BYTES,
               "spectre_versions": [row["version"] for row in reference["spectre"]],
               "mask_sha256": reference["mask"]["sha256"],
               "qualification": reference["qualification"]}
    return decoded, summary


def main():
    require(len(sys.argv) == 1, "Usage: inspect_wpc.py (pinned local references only)")
    _, summary = inspect()
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, StopIteration) as error:
        sys.exit(str(error))
