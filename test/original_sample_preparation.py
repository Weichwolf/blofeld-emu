# SPDX-License-Identifier: GPL-3.0-only
"""Execute original sample preparation only after exact offline reference checks."""

from collections import Counter
from functools import reduce
import hashlib
import json
from operator import xor
from pathlib import Path
import subprocess
import sys
import tempfile
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from inspect_firmware import decode_block, read_smf


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def matches(data, reference):
    return (len(data) == reference["bytes"]
            and hashlib.sha256(data).hexdigest() == reference["sha256"])


def load_reference(path):
    entry = json.loads((ROOT / "doc/library/sources.json").read_text())["files"][0]
    require(path.stat().st_size == entry["bytes"], "Changed sample archive size")
    with path.open("rb") as stream:
        require(hashlib.file_digest(stream, "sha256").hexdigest() == entry["sha256"],
                "Changed sample archive SHA-256")
    member = entry["midi_member"]
    with ZipFile(path) as archive:
        require(archive.getinfo(member["file"]).file_size == member["bytes"],
                "Changed sample MID member size")
        midi = archive.read(member["file"])
    require(matches(midi, member), "Changed contained sample MID identity")
    messages, _ = read_smf(midi)
    observed = member["transport_observations"]
    require(len(messages) == observed["sysex_messages"], "Changed sample message count")
    census = {hex(key): value for key, value in Counter(m[3] for m in messages).items()}
    require(census == observed["commands"], "Changed sample command census")
    return messages, observed


def sample_blocks(messages):
    blocks = []
    for message in messages:
        require(message[:3] == b"\x3e\x13\x7f" and message[-1:] == b"\xf7",
                "Changed sample SysEx identity/framing")
        if message[3] == 0x73:
            require(len(message) == 14, "Changed sample header length")
            blocks.append({"header": message, "packets": []})
            continue
        require(message[3] in (0x74, 0x75) and blocks, "Unsupported sample packet command")
        require(len(message) >= 8, "Truncated sample packet")
        packets = blocks[-1]["packets"]
        require(message[4] == len(packets) % 128, "Changed sample packet sequence")
        require(reduce(xor, message[:-1], 0) == 0, "Changed sample packet XOR")
        scale = 8 if message[3] == 0x75 else 1
        require(len(message) == scale * message[5] + 8, "Changed sample packet length")
        packets.append(message)
    return blocks


def extract_inputs(blocks, reference, observations):
    selected = reference["helper"]
    helper = None
    frames = bytearray()
    groups = 0
    duplicates = 0
    for index, block in enumerate(blocks):
        if not block["packets"]:
            require(index + 1 < len(blocks) and blocks[index + 1]["header"] == block["header"],
                    "Changed duplicate sample header")
            duplicates += 1
            continue
        _, decoded, metadata = decode_block(block)
        if index == selected["header_index"]:
            require(metadata["mode"] == selected["mode"]
                    and metadata["address_words"] == selected["address_words"]
                    and matches(decoded, selected), "Changed original sample helper")
            helper = decoded
        bulk = any(packet[3] == 0x75 for packet in block["packets"])
        if not bulk:
            continue
        require(metadata["mode"] == 3 and len(decoded) == reference["group_bytes"],
                "Changed bulk sample group")
        packets = block["packets"]
        require(len(packets) == reference["packets_per_group"]
                and all(packet[3] == 0x75 for packet in packets), "Changed bulk packet group")
        lengths = reference["packet_lengths"]
        require(all(len(packet) == lengths[0] for packet in packets[:-1])
                and len(packets[-1]) == lengths[1], "Changed bulk packet boundaries")
        address = reference["staging_addresses"][groups % 4]
        require(metadata["address_words"] == address, "Changed sample staging address")
        frames.extend(address.to_bytes(4, "big"))
        frames.extend(decoded)
        groups += 1
    require(helper is not None, "Missing original preparation helper")
    require(duplicates == observations["duplicate_empty_headers"]
            and groups == observations["bulk_groups"] == reference["counts"]["groups"],
            "Incomplete original sample groups")
    require(matches(frames, reference["frames"]), "Changed complete original sample frames")
    return helper, frames


def execute(binary, archive, trace):
    if not archive.is_file():
        print("SKIP original sample preparation: private official sample archive absent")
        return 77
    reference = json.loads((ROOT / "doc/sample-preparation-reference.json").read_text())
    messages, observations = load_reference(archive)
    helper, frames = extract_inputs(sample_blocks(messages), reference, observations)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix="sample-preparation-", dir=trace.parent) as directory:
        temporary = Path(directory)
        helper_path = temporary / "helper.be24.bin"
        frames_path = temporary / "frames.be24.bin"
        helper_path.write_bytes(helper)
        frames_path.write_bytes(frames)
        result = subprocess.run([str(binary.resolve()), str(helper_path.resolve()),
                                 str(frames_path.resolve()), str(trace.resolve())],
                                check=False, timeout=280)
    if result.returncode:
        return result.returncode
    rows = dict(line.split(maxsplit=1) for line in trace.read_text().splitlines())
    counts = reference["counts"]
    require(list(map(int, rows["coverage"].split())) == [counts["groups"], counts["pages"],
                                                      counts["independent_byte_comparisons"]],
            "Incomplete original native preparation coverage")
    require("Unknown DSP memory read at X:0x000046" in rows["missing-status"]
            and "pc=0x002e28" in rows["missing-status"]
            and "omr=0x600389" in rows["missing-status"], "Missing original status frontier")
    require(rows["scope"] == "isolated-preparation no-NAND-completion", "Changed test scope")
    print(f"Original preparation: {counts['groups']} scans, {counts['pages']} pages, "
          f"{counts['independent_byte_comparisons']} independent byte comparisons.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit("Usage: original_sample_preparation.py binary sample-archive trace")
    try:
        sys.exit(execute(*map(Path, sys.argv[1:])))
    except (OSError, ValueError, KeyError) as error:
        sys.exit(str(error))
