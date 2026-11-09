# SPDX-License-Identifier: GPL-3.0-only
"""Read the emulator's bounded, binary 128x64 PBM publication."""

from dataclasses import dataclass
import os
from pathlib import Path
import stat
import subprocess
import sys

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


def publisher_present(path):
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC | os.O_NOFOLLOW)
    except FileNotFoundError:
        return False
    try:
        if not stat.S_ISFIFO(os.fstat(descriptor).st_mode):
            raise ValueError("Invalid display presence channel")
        try:
            if os.read(descriptor, 1):
                raise ValueError("Unexpected display presence data")
            return False
        except BlockingIOError:
            return True
    finally:
        os.close(descriptor)


def runtime_directory():
    configured = os.environ.get("XDG_RUNTIME_DIR")
    if configured:
        path = Path(configured)
    elif sys.platform == "darwin":
        try:
            path = Path(subprocess.check_output(
                ["/usr/bin/getconf", "DARWIN_USER_TEMP_DIR"], text=True, timeout=1).strip())
        except (OSError, subprocess.SubprocessError) as error:
            raise ValueError("User runtime directory is unavailable") from error
    else:
        path = Path(f"/run/user/{os.getuid()}")
    if not path.is_absolute():
        raise ValueError("XDG_RUNTIME_DIR must be an absolute path")
    return path
