#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Generate synthetic logical 6800 bus samples, with no firmware program bytes."""

import argparse
from pathlib import Path


def pins(control, value=None, known=63):
    drive = 255 if value is not None else 0
    return bytes((control, known, drive, value or 0, drive))


def write(value, data=False):
    low = 50 if data else 34
    return pins(low, value) + pins(low | 4, value) + pins(low, value)


def read():
    # CS assertion enables the read while E is already high.
    return pins(63) + pins(62) * 2 + pins(58)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    output = parser.parse_args().directory
    output.mkdir(parents=True, exist_ok=True)
    reset = pins(0, known=32) + pins(33)
    samples = {"cold-read": read(), "reset-abort": reset + pins(54, 0xa5) + reset + read()}
    samples["end-address-data-write"] = reset + pins(54, 0xa5) + pins(33, 0xa5) + reset
    samples["end-address-read"] = (
        reset + write(0x55, True) + write(0) + pins(63) + pins(62)
        + pins(33) + pins(62) + pins(37) + reset)
    samples["end-unknown-address"] = reset + pins(62) + pins(33, known=33) + reset
    samples["mid-cycle-address-reject"] = reset + pins(54, 0xa5) + pins(38, 0xa5) + reset
    for command in range(256):
        samples[f"command-{command:02x}"] = (
            reset + write(command) + write(0) + write(0xa5, True) + read() * 2)
    for control in range(64):
        samples[f"controls-{control:02x}"] = reset + pins(control) * 2 + pins(33)
    frame = reset
    for page in range(9):
        row = write(0xb0 | page) + write(0x10) + write(0)
        row += b"".join(write((column * 37) & 255, True) for column in range(132))
        frame += row
        samples[f"page-{page}"] = reset + row + write(0x10) + write(0) + read() * 134
    samples["whole-ram"] = frame + reset + read() * 2
    for column in (0, 1, 15, 16, 127, 128, 131, 132, 255):
        address = write(0x10 | (column >> 4)) + write(column & 15)
        samples[f"rmw-{column}"] = (
            reset + address + write(0x55, True) + address + write(0xe0)
            + read() * 2 + write(0xaa, True) + read() * 2 + write(0xee) + reset)
    for mask in (0, 1, 127, 254, 255):
        samples[f"unknown-write-{mask}"] = (
            reset + bytes((54, 63, 255, 0, mask)) + bytes((50, 63, 255, 0, mask)) + reset)
    for name, sample in samples.items():
        (output / name).write_bytes(sample)
    print(f"Wrote {len(samples)} synthetic LCD pin seeds to {output}")


if __name__ == "__main__":
    main()
