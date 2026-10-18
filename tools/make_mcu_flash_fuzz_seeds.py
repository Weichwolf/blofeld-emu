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


def word_stage(address, value):
    return bytes([16, address >> 8, address & 255, 1, value >> 8, value & 255])


def command_prefix(address=0x8234):
    data = record(26, value=255, extra=254) + record(1, 0x100, 0x48)
    # Explicit 12 MHz oscillator and 24 MHz bus in action 27's scaled encoding.
    data += bytes([27, 0x2d, 0xc6, 0xc0, 0x5d, 0xc0])
    data += word_stage(address, 0xffff) + record(1, 0x106, 0x40)
    return data + record(1, 0x105, 0x80) + record(28) + record(29)


def command_seed(opcode, phase, reset_action):
    data = command_prefix() + word_stage(0x8234, 0x1234)
    if phase >= 1:
        data += record(1, 0x106, opcode)
    if phase >= 2:
        data += record(1, 0x105, 0x80)
    if phase >= 3:
        data += record(28)
    if phase >= 4:
        data += record(29)
    data += record(reset_action)
    for address in (0x8000, 0x8200, 0x8234, 0x8236, 0xff0d, 0xff0f, 0xfffe, 0x105):
        data += record(0, address)
    return data + record(5, 0x8234)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    directory = parser.parse_args().directory
    directory.mkdir(parents=True, exist_ok=True)
    for value in range(256):
        (directory / f"options-{value:02x}.seed").write_bytes(make_seed(value))
    for opcode in (0x20, 0x40, 0x41, 5):
        for phase in range(5):
            for reset_action in (4, 30):
                filename = f"command-{opcode:02x}-phase-{phase}-reset-{reset_action}.seed"
                (directory / filename).write_bytes(command_seed(opcode, phase, reset_action))
    for address in (0xff0e, 0xfffe):
        data = command_prefix(address) + word_stage(address, 0x8000)
        data += record(1, 0x106, 0x20) + record(1, 0x105, 0x80) + record(28) + record(30)
        data += record(0, address) + record(0, 0xff0d) + record(0, 0xff0f)
        (directory / f"interrupted-vector-options-{address:04x}.seed").write_bytes(data)
    print("Created 298 synthetic flash-option/command/reset/power-loss seeds; no manufacturer data.")


if __name__ == "__main__":
    main()
