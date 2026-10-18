# SPDX-License-Identifier: GPL-3.0-only
"""Create synthetic FTS32K configuration seeds for the MCU bus/model fuzzer."""

import argparse
from pathlib import Path


def record(action, address=0, value=0, extra=0):
    """Encode one six-byte MCU model-fuzzer operation."""
    return bytes([action, address >> 8, address & 255, value, extra, 0])


def make_seed(value):
    # Action 26 constructs a fresh image with explicitly supplied option fields.
    data = record(26, value=value, extra=value ^ 255)
    for address in (0x100, 0x101, 0x104):
        data += record(0, address)
    data += record(1, 0x100, value) + record(1, 0x100, value ^ 255) + record(0, 0x100)
    for requested in (0, 255, value ^ 0xa4, value ^ 0x1b, value):
        data += record(1, 0x104, requested) + record(0, 0x104)
    data += record(4) + record(0, 0x104) + record(0, 0x101)
    data += record(1, 0x11, 0x40) + record(2)
    data += record(0, 0x4100) + record(0, 0x4101)
    data += record(1, 0x4104, value) + record(0, 0x4104)
    # Unsupported commands/array writes are expected rejections, not flash success.
    data += record(1, 0x4105, 0x80) + record(1, 0x4106, 0x20)
    data += record(1, 0x8200, value) + record(4)
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    directory = parser.parse_args().directory
    directory.mkdir(parents=True, exist_ok=True)
    for value in range(256):
        (directory / f"options-{value:02x}.seed").write_bytes(make_seed(value))
    print("Created 256 synthetic flash-option/control/reset/remap seeds; no manufacturer data.")


if __name__ == "__main__":
    main()
