# SPDX-License-Identifier: GPL-3.0-only
"""Generate only synthetic SHI register/pin sequences for the offline fuzzer."""

import argparse
from pathlib import Path


def register_write(register: int, value: int) -> bytes:
    return bytes((0, register, (value >> 16) & 255, (value >> 8) & 255, value & 255, 0))


def pins(selected: bool, clock: bool, mosi: bool = False) -> bytes:
    return bytes((2, int(selected) | (int(clock) << 1), 1, int(mosi), 0, 0))


def register_read(register: int) -> bytes:
    return bytes((1, register, 0, 0, 0, 0))


def frame(mode: int, width: int, value: int) -> bytes:
    idle = bool(mode & 2)
    data = pins(True, idle)
    for bit in range(width):
        mosi = bool((value >> (23 - bit)) & 1)
        data += pins(True, not idle, mosi) + pins(True, idle, mosi)
    return data + pins(False, idle)


def make_seeds(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for mode in range(4):
        for width in (8, 16, 24):
            control = 0x21 | (((width // 8) - 1) << 2)
            data = pins(False, bool(mode & 2))
            data += register_write(0, mode) + register_write(1, control)
            data += register_write(3, 0xA15C37) + frame(mode, width, 0x123456)
            data += register_read(1) + register_read(4) + bytes((5, 0, 0, 0, 0, 0))
            (output / f"mode-{mode}-width-{width}").write_bytes(data)
    for large in (False, True):
        data = pins(False, False) + register_write(0, 0)
        data += register_write(1, 0x3821 if large else 0x1801)
        for word in range(11 if large else 2):
            data += frame(0, 8, word << 16)
        data += register_read(4) + register_read(1) + register_read(4)
        data += register_write(1, 0x20 if large else 0) + bytes((3, 0, 0, 0, 1, 0))
        (output / f"fifo-{large}-overrun-reset").write_bytes(data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Directory for 14 synthetic seed files")
    make_seeds(parser.parse_args().output)
