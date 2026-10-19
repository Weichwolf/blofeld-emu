#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Generate only synthetic completed LCD transactions, never firmware data."""

import argparse
from pathlib import Path


def record(action, value=0, extra=0):
    return bytes((action, value, extra))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    output = parser.parse_args().directory
    output.mkdir(parents=True, exist_ok=True)
    reset = record(0)
    samples = {"cold-unknown": record(3) + record(4) + record(5)}
    for command in range(256):
        samples[f"command-{command:02x}"] = (
            reset + record(1, command) + record(1, 0xff)
            + record(2, 0xa5) + record(3) + record(3) + record(5))
    for page in range(9):
        row = reset + record(1, 0xb0 | page)
        row += b"".join(record(2, (column * 37) & 255) for column in range(132))
        row += record(1, 0x10) + record(1, 0)
        row += record(3) * 134 + record(0) + record(4, page, 131)
        samples[f"page-{page}"] = row
    for column in (0, 1, 15, 16, 127, 128, 131, 132, 255):
        address = record(1, 0x10 | (column >> 4)) + record(1, column & 15)
        samples[f"rmw-{column}"] = (
            reset + address + record(2, 0x55) + address + record(1, 0xe0)
            + record(3) * 2 + record(2, 0xaa) + record(3) * 2
            + record(1, 0xee) + record(3) * 2 + record(1, 0xe2) + record(5))
    for name, sample in samples.items():
        (output / name).write_bytes(sample)
    print(f"Wrote {len(samples)} synthetic LCD seeds to {output}")


if __name__ == "__main__":
    main()
