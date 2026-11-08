# SPDX-License-Identifier: GPL-3.0-only
"""Read the emulator's bounded, binary 128x64 PBM publication."""

from dataclasses import dataclass
import os
from pathlib import Path
import stat

WIDTH = 128
HEIGHT = 64
HEADER = b"P4\n128 64\n"
PAYLOAD_BYTES = WIDTH * HEIGHT // 8
FILE_BYTES = len(HEADER) + PAYLOAD_BYTES


@dataclass(frozen=True)
class Frame:
    bits: bytes

    def __post_init__(self):
        if len(self.bits) != PAYLOAD_BYTES:
            raise ValueError("Invalid LCD pixel payload")

    def on(self, x, y):
        return bool(self.bits[y * (WIDTH // 8) + x // 8] & (0x80 >> (x % 8)))


def read_frame(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        information = os.fstat(stream.fileno())
        if not stat.S_ISREG(information.st_mode) or information.st_size != FILE_BYTES:
            raise ValueError("Display must be a complete 128 x 64 binary PBM")
        data = stream.read(FILE_BYTES + 1)
    if len(data) != FILE_BYTES or not data.startswith(HEADER):
        raise ValueError("Invalid display format; waiting for a complete PBM frame")
    return Frame(data[len(HEADER):])


def runtime_directory():
    path = Path(os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}")
    if not path.is_absolute():
        raise ValueError("XDG_RUNTIME_DIR must be an absolute path")
    return path
