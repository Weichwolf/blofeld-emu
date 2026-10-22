# SPDX-License-Identifier: GPL-3.0-only
"""Independent PNG oracle and Linux directory-notification/concurrent-reader gate."""

import binascii
import ctypes
import os
from pathlib import Path
import select
import stat
import struct
import subprocess
import sys
import tempfile
import time
import zlib


def require(condition, message):
    if not condition:
        raise ValueError(message)


def png(path, width, rows):
    data = path.read_bytes()
    require(data[:8] == b"\x89PNG\r\n\x1a\n", "PNG signature")
    offset = 8
    chunks = []
    while offset < len(data):
        length = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        payload = data[offset + 8:offset + 8 + length]
        crc = struct.unpack_from(">I", data, offset + 8 + length)[0]
        require(crc == binascii.crc32(kind + payload), "Independent PNG chunk CRC")
        chunks.append((kind, payload))
        offset += length + 12
    require(offset == len(data) and [kind for kind, _ in chunks] == [b"IHDR", b"IDAT", b"IEND"],
            "Complete minimal PNG chunk sequence")
    require(chunks[0][1] == struct.pack(">IIBBBBB", width, 64, 1, 0, 0, 0, 0),
            "PNG dimensions, 1-bit grayscale, no interlace")
    require(zlib.decompress(chunks[1][1]) == rows,
            "Independent literal PNG rows, padding, filter None and grayscale polarity")
    require(chunks[2][1] == b"", "IEND empty")


def events(descriptor):
    names = []
    while select.select([descriptor], [], [], 0)[0]:
        data = os.read(descriptor, 65536)
        offset = 0
        while offset < len(data):
            _, mask, _, length = struct.unpack_from("iIII", data, offset)
            offset += 16
            name = data[offset:offset + length].split(b"\0", 1)[0].decode()
            offset += length
            require(not mask & 0x4000, "No inotify queue overflow in bounded regression")
            if mask & 0x80:  # IN_MOVED_TO, watched on the stable directory.
                names.append(name)
    return names


def reply(process):
    require(select.select([process.stdout], [], [], 30)[0], "Bounded native test reply")
    return process.stdout.readline().strip()


def check_readers(process, directory, descriptor):
    bitmap = {b"P4\n128 64\n" + bytes(1024), b"P4\n128 64\n" + b"\xff" * 1024}
    text = {(b" " * 128 + b"\n") * 64, (b"#" * 128 + b"\n") * 64}
    original_inode = (directory / "display.pbm").stat().st_ino
    process.stdin.write("same\n")
    process.stdin.flush()
    require(reply(process) == "done", "Unchanged frame acknowledged")
    require(events(descriptor) == [] and (directory / "display.pbm").stat().st_ino == original_inode,
            "No rename or inode change for unchanged pixels")
    process.stdin.write("burst\n")
    process.stdin.flush()
    reads = 0
    names = []
    deadline = time.monotonic() + 30
    while not select.select([process.stdout], [], [], 0)[0]:
        require(time.monotonic() < deadline, "Bounded frame publication")
        require((directory / "display.pbm").read_bytes() in bitmap, "Concurrent PBM reader sees a full frame")
        require((directory / "display.txt").read_bytes() in text, "Concurrent text reader sees a full frame")
        reads += 1
        names.extend(events(descriptor))
    require(reply(process) == "done" and reads > 0, "Native publisher and external readers overlap")
    names.extend(events(descriptor))
    require(names.count("display.pbm") == 64 and names.count("display.txt") == 64,
            "Every changed image creates the directory rename notification needed by a GUI")
    require((directory / "display.pbm").read_bytes() == b"P4\n128 64\n" + bytes(1024),
            "A late/resynchronizing reader obtains the latest complete image")
    for name in ("display.pbm", "display.txt", ".display.lock"):
        info = (directory / name).stat()
        require(stat.S_IMODE(info.st_mode) == 0o600 and info.st_uid == os.getuid(),
                "Runtime artifacts retain private user ownership and permissions")
    return reads


def main():
    with tempfile.TemporaryDirectory(prefix="blofeld-lcd-export-") as temporary:
        directory = Path(temporary)
        native = subprocess.Popen([sys.argv[1], str(directory)], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        descriptor = -1
        try:
            require(reply(native) == "ready", "Native publisher/failure gates initialize")
            png(directory / "black.png", 128, bytes(17 * 64))
            png(directory / "pattern.png", 9,
                b"\x00\x7f\x7f" + b"\x00\xff\xff" * 62 + b"\x00\xfe\xff")
            libc = ctypes.CDLL(None, use_errno=True)
            descriptor = libc.inotify_init1(os.O_CLOEXEC | os.O_NONBLOCK)
            require(descriptor >= 0, "Create user inotify descriptor")
            watch = libc.inotify_add_watch(descriptor, os.fsencode(directory), 0x80)
            require(watch >= 0, "Watch directory rename publications")
            reads = check_readers(native, directory, descriptor)
            native.stdin.write("stop\n")
            native.stdin.flush()
            output, errors = native.communicate(timeout=30)
            require(native.returncode == 0 and not output and not errors,
                    f"Native publisher exits without sanitizer findings: {errors}")
            print(f"LCD export: PNG oracle, failure repair, 64 atomic publications, inotify and {reads} concurrent reads passed")
        finally:
            if descriptor >= 0:
                os.close(descriptor)
            if native.poll() is None:
                native.kill()
            native.wait()


if __name__ == "__main__":
    main()
