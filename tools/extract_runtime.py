#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Extract and verify the observed Blofeld 1.25 runtime layout, offline only.

This is an analysis tool, not an installer or a complete emulator boot loader.
Memory-space labels below describe the observed DSP record format.
"""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def words24(data):
    if len(data) % 3:
        raise ValueError("Expected complete 24-bit words")
    return [int.from_bytes(data[pos:pos + 3], "big") for pos in range(0, len(data), 3)]


def verify_sum(data):
    if sum(words24(data)) & 0xffffff:
        raise ValueError("24-bit additive object checksum mismatch")


def hc12_object(data):
    if len(data) < 2:
        raise ValueError("Truncated HC12 object header")
    length = int.from_bytes(data[:2], "big")
    end = ((length + 2 + 2) // 3) * 3 + 3
    if end > len(data):
        raise ValueError("HC12 object exceeds payload")
    verify_sum(data[:end])
    return data[2:2 + length], end


def dsp_records(data):
    words = words24(data)
    cursor, records, writes = 0, [], {0: {}, 1: {}, 2: {}}
    overwritten = []
    while cursor + 3 <= len(words):
        space, count, address = words[cursor:cursor + 3]
        if (space, count, address) == (0x100, 0x100, 0x100):
            end = (cursor + 4) * 3
            if cursor + 4 > len(words):
                raise ValueError("Missing DSP checksum")
            verify_sum(data[:end])
            return records, writes, overwritten, end
        if space not in writes or not count or cursor + 3 + count > len(words):
            raise ValueError(f"Invalid DSP record at word {cursor:#x}")
        if address + count > 0x1000000:
            raise ValueError("DSP record exceeds 24-bit address space")
        contents = words[cursor + 3:cursor + 3 + count]
        for index, value in enumerate(contents):
            target = address + index
            if target in writes[space]:
                overwritten.append({"space": space, "address": target,
                                    "before": writes[space][target], "after": value})
            writes[space][target] = value
        records.append({"space": space, "address": address, "words": count,
                        "source_byte_offset": cursor * 3,
                        "data": b"".join(value.to_bytes(3, "big") for value in contents)})
        cursor += 3 + count
    raise ValueError("Missing DSP end marker")


def extract(input_path, output):
    midi, name = load_input(input_path)
    messages, _ = read_smf(midi)
    blocks = split_blocks(messages)
    decoded = [decode_block(block) for block in blocks]
    expected = [(3, 768, 0xef00), (0, 189, 0x2e00), (3, 8649, 0x10000),
                (0, 1164, 0x2e00), (3, 89643, 0xd000), (0, 1200, 0x2e00)]
    observed = [(item[2]["mode"], item[2]["declared_bytes"],
                 item[2]["address_words"]) for item in decoded]
    if observed != expected:
        raise ValueError("This extractor supports the observed 1.25 layout only")
    main = decoded[4][1]
    initial_helper, end = hc12_object(decoded[0][1])
    if end != len(decoded[0][1]):
        raise ValueError("Unexpected bytes after initial HC12 helper")
    flash_helper, host_offset = hc12_object(main)
    host = main[host_offset:host_offset + 0x8000]
    if len(host) != 0x8000:
        raise ValueError("Missing 32 KiB MCU image")
    dsp_offset = host_offset + ((0x8000 + 2) // 3) * 3 + 3
    verify_sum(main[host_offset:dsp_offset])
    records, memory, overwritten, dsp_end = dsp_records(main[dsp_offset:])
    after_dsp = dsp_offset + dsp_end
    marker_end = after_dsp + 15
    expected_markers = [0x100, 0x100, 0x100, 0x100, 0x123123]
    if words24(main[after_dsp:marker_end]) != expected_markers:
        raise ValueError("Unexpected main payload trailer")
    error_helper, error_end = hc12_object(main[marker_end:])
    if marker_end + error_end != len(main):
        raise ValueError("Unexpected bytes after error helper")
    page_data = decoded[2][1]
    page_words = words24(page_data)
    verify_sum(page_data[3:0x2106])
    small_error, small_end = hc12_object(page_data[0x2106:])
    if small_end + 0x2106 != len(page_data):
        raise ValueError("Unexpected secondary payload trailer")
    host_sum = sum(int.from_bytes(host[p:p + 2], "big")
                   for p in range(0x200, 0x7f00, 2)) & 0xffff
    stored_sum = int.from_bytes(host[0x7f10:0x7f12], "big")
    if host_sum != stored_sum:
        raise ValueError("MCU boot checksum over 0x8200..0xfeff does not match 0xff10")
    output.mkdir(parents=True, exist_ok=True)
    files = {"host-mcu-8000.bin": host, "hc12-initial-helper.bin": initial_helper,
             "hc12-flash-helper.bin": flash_helper, "hc12-error-helper.bin": error_helper,
             "secondary-flash-pages.bin": page_data[3:0x2103],
             "hc12-secondary-error-helper.bin": small_error,
             "dsp-records-verified.bin": main[dsp_offset:after_dsp]}
    for index, record in enumerate(records):
        filename = f"sections/{index:03d}-s{record['space']}-{record['address']:06x}.bin"
        files[filename] = record.pop("data")
        record["file"] = filename
    for filename, data in files.items():
        path = output / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    report = {"input_member": name, "input_sha256": hashlib.sha256(midi).hexdigest(),
              "offline_only": True, "hardware_execution_verified": False,
              "hc12_helper_bytes": [len(initial_helper), len(flash_helper), len(error_helper)],
              "host_offset_in_block4": host_offset, "host_bytes": len(host),
              "host_address_range": [0x8000, 0xffff], "host_reset_vector": int.from_bytes(host[-2:], "big"),
              "host_boot_checksum": host_sum, "host_boot_checksum_verified": True,
              "dsp_offset_in_block4": dsp_offset, "dsp_records": records,
              "dsp_overwrites_in_record_order": overwritten,
              "dsp_spaces": {space: {"sections": Counter(r['space'] for r in records)[space],
                                       "unique_words": len(mem), "max_address": max(mem)}
                             for space, mem in memory.items()},
              "dsp_reset_words": [memory[0].get(0), memory[0].get(1)],
              "secondary_flash_first_page": page_words[0],
              "files": [{"file": filename, "bytes": len(data),
                         "sha256": hashlib.sha256(data).hexdigest()}
                        for filename, data in files.items()]}
    (output / "runtime.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = extract(args.input, args.output)
    print(f"Verified MCU: {report['host_bytes']} bytes; reset {report['host_reset_vector']:#06x}")
    print(f"Verified DSP: {len(report['dsp_records'])} records; spaces {report['dsp_spaces']}")
    print(f"Hardware execution verified: {report['hardware_execution_verified']}")


if __name__ == "__main__":
    main()
