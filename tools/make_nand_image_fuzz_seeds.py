# SPDX-License-Identifier: GPL-3.0-only
"""Generate synthetic NAND image inputs, never manufacturer memory contents."""

import argparse
import binascii
from pathlib import Path
import struct


def encoded(pages, blocks):
    body = bytearray(b"BLFNAND\n" + struct.pack("<IIIIII", 1, 1, len(pages), len(blocks), 528, 0))
    for block, (known, value) in sorted(blocks.items()):
        body.extend(struct.pack("<HBB", block, known, value))
    for page, (main, spare, value, known) in sorted(pages.items()):
        body.extend(struct.pack("<IBBH", page, main, spare, 0))
        body.extend(bytes((value, known)) * 528)
    return bytes(body) + struct.pack("<I", binascii.crc32(body))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    output = parser.parse_args().output
    output.mkdir(parents=True, exist_ok=True)
    cases = {
        "empty-unknown": ({}, {}),
        "explicit-erased-history": ({37: (0, 0, 255, 255)}, {1: (7, 1)}),
        "observed-ff-unknown-history": ({37: (255, 255, 255, 255)}, {1: (7, 1)}),
        "consumed-main-spare-history": ({37: (1, 2, 165, 255)}, {1: (7, 1)}),
        "interrupted-write": ({37: (255, 255, 0, 0)}, {1: (0, 0)}),
        "high-row-partial-knowledge": ({131071: (255, 255, 160, 240)}, {4095: (6, 0)}),
        "invalid-fresh-programmed-bits": ({37: (0, 0, 66, 255)}, {1: (7, 1)}),
        "invalid-mixed-history-sentinel": ({37: (255, 0, 0, 0)}, {1: (7, 1)}),
        "invalid-high-protected-block": ({}, {4095: (7, 3)}),
    }
    for name, (pages, blocks) in cases.items():
        (output / name).write_bytes(encoded(pages, blocks))


if __name__ == "__main__":
    main()
