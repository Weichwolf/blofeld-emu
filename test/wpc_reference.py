# SPDX-License-Identifier: GPL-3.0-only
"""Check own codec controls and exact local WPC/MIDI page correspondence."""

from collections import defaultdict
import hashlib
import json
from pathlib import Path
import struct
import sys
import tempfile

from original_sample_preparation import load_reference, sample_blocks, extract_inputs

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from inspect_wpc import decode_pages, inspect, macho_bytes, read_pinned, require


def rejects(operation, reason):
    try:
        operation()
    except ValueError as error:
        require(reason in str(error), "A fixed negative control failed for the wrong reason")
        return
    raise ValueError("A fixed invalid codec input reported success")


def controls():
    mask = bytes([0x12]) * 527 + bytes([0x34])
    expected = (bytes([0x44]) * 527 + bytes([0x62])) * 2
    require(decode_pages(bytes([0x56]) * 1056, mask) == expected,
            "Two own pages must reset their mask at the literal byte-528 boundary")
    rejects(lambda: decode_pages(b"", mask), "complete 528-byte pages")
    rejects(lambda: decode_pages(bytes(527), mask), "complete 528-byte pages")
    rejects(lambda: decode_pages(bytes(528), mask[:-1]), "exactly 528 bytes")
    header = struct.pack("<8I", 0xfeedfacf, 0x1000007, 3, 2, 1, 72, 0, 0)
    segment = struct.pack("<II16s4Q4I", 0x19, 72, b"__TEXT", 0x1000, 1024, 0, 1024, 5, 5, 0, 0)
    data = bytearray(1024)
    data[:104] = header + segment
    data[256:784] = mask
    require(macho_bytes(data, 0x1100, 528) == mask,
            "Independent own Mach-O VM/file offsets select exactly the declared span")
    rejects(lambda: macho_bytes(data[:100], 0x1100, 528), "command extent")
    rejects(lambda: macho_bytes(data, 0x1400, 528), "Missing or ambiguous")
    data[36:40] = struct.pack("<I", 64)
    rejects(lambda: macho_bytes(data, 0x1100, 528), "Truncated Mach-O segment")
    with tempfile.TemporaryDirectory(prefix="wpc-codec-") as directory:
        path = Path(directory) / "own-reference"
        payload = b"Own fixed reference identity"
        expected = {"bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}
        path.write_bytes(payload)
        require(read_pinned(path, expected) == payload, "The literal pinned file must be accepted")
        path.write_bytes(payload + b"x")
        rejects(lambda: read_pinned(path, expected), "Changed reference size")
        path.write_bytes(b"x" + payload[1:])
        rejects(lambda: read_pinned(path, expected), "Changed reference SHA-256")
    print("Verified eleven fixed page/segment/reference-identity controls; no fuzzing")


def correspondence(decoded, frames, reference):
    page_bytes = 528
    block_bytes = 32 * page_bytes
    stride = 4 + reference["group_bytes"]
    require(len(frames) == reference["counts"]["groups"] * stride, "Changed prepared-frame extent")
    pages = defaultdict(list)
    blocks = set()
    for group in range(reference["counts"]["groups"]):
        start = group * stride + 4
        data = frames[start:start + block_bytes]
        blocks.add(hashlib.sha256(data).digest())
        for page in range(32):
            payload = data[page * page_bytes:(page + 1) * page_bytes]
            pages[hashlib.sha256(payload).digest()].append(group * 32 + page)
    matching = unique = 0
    for page in range(len(decoded) // page_bytes):
        raw = decoded[page * page_bytes:(page + 1) * page_bytes]
        hits = pages.get(hashlib.sha256(raw).digest(), [])
        matching += bool(hits)
        unique += len(hits) == 1
    return {"wpc_pages": len(decoded) // page_bytes,
            "midi_prepared_pages": reference["counts"]["pages"],
            "matching_pages": matching, "matching_bytes": matching * page_bytes,
            "unique_matching_pages": unique,
            "unmatched_wpc_pages": len(decoded) // page_bytes - matching,
            "matching_32_page_blocks": sum(
                hashlib.sha256(decoded[start:start + block_bytes]).digest() in blocks
                for start in range(0, len(decoded), block_bytes))}


def original(output):
    reference = json.loads((ROOT / "doc/wpc-reference.json").read_text())
    required = [ROOT / "doc" / row["archive"] for row in reference["spectre"]]
    required += [ROOT / "doc/library" / reference["wpc_file"],
                 ROOT / "doc/library" / reference["midi_archive"]]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    if not all(path.is_file() for path in required):
        print("SKIP fixed WPC/MIDI reference: private official reference assets absent")
        return 77
    decoded, summary = inspect()
    native = json.loads((ROOT / "doc/sample-preparation-reference.json").read_text())
    messages, observations = load_reference(required[-1])
    _, frames = extract_inputs(sample_blocks(messages), native, observations)
    observed = correspondence(decoded, frames, native)
    require(observed == reference["comparison"], "Changed exact WPC/MIDI page correspondence")
    require(summary["pages"] == observed["wpc_pages"], "Changed WPC page extent")
    output.write_text(json.dumps({"passed": True, **summary, **observed,
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print(f"Verified {observed['matching_pages']} exact WPC/MIDI page matches and "
          f"{observed['unmatched_wpc_pages']} unresolved WPC pages; physical mapping remains open")
    return 0


if __name__ == "__main__":
    try:
        if sys.argv[1:] == ["--synthetic"]:
            controls()
            sys.exit(0)
        require(len(sys.argv) == 2, "Usage: wpc_reference.py --synthetic|private-report.json")
        sys.exit(original(Path(sys.argv[1])))
    except (OSError, ValueError, KeyError, StopIteration) as error:
        sys.exit(str(error))
