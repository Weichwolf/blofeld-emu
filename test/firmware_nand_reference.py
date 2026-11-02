# SPDX-License-Identifier: GPL-3.0-only
"""Pin complete original source-only NAND contents without publishing the bytes."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import zlib

EXPECTED_ZIP = "095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72"
EXPECTED_IMAGE = "b422235d6e6b3f52a36d7bb9f18509b2e5111fc804ba08381efb2fe028bfaead"


def main():
    binary, firmware, output = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP initial NAND reference: private original1.25 archive is absent")
        return 77
    original = firmware.read_bytes()
    if hashlib.sha256(original).hexdigest() != EXPECTED_ZIP:
        raise ValueError("Original firmware archive identity differs")
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware), str(output)],
                            capture_output=True, text=True, timeout=30, check=False)
    if result.returncode or result.stderr or result.stdout != (
            "Source-only firmware NAND pages=123 blocks=0\n"):
        raise ValueError(result.stdout + result.stderr)
    image = output.read_bytes()
    if len(image) != 130908 or hashlib.sha256(image).hexdigest() != EXPECTED_IMAGE:
        raise ValueError("Native initial NAND differs from the independent whole-image oracle")
    if image[:8] != b"BLFNAND\n" or struct.unpack_from("<6I", image, 8) != (1, 1, 123, 0, 528, 0):
        raise ValueError("Initial NAND image envelope differs")
    if struct.unpack_from("<I", image, len(image) - 4)[0] != zlib.crc32(image[:-4]):
        raise ValueError("Initial NAND image CRC differs")
    known = 0
    for index in range(123):
        offset = 32 + index * 1064
        expected_page = index if index < 107 else 896 + index - 107
        if struct.unpack_from("<IBBH", image, offset) != (expected_page, 255, 255, 0):
            raise ValueError("Ordered source pages or unknown programming history differ")
        cells = image[offset + 8:offset + 1064]
        known += cells[1::2].count(255)
    if known != 64686 or firmware.read_bytes() != original:
        raise ValueError("Known source-byte count or immutable input differs")
    print("Original initial NAND: all runtime/trailer/transport bytes and16 secondary pages "
          "match the independent123-page oracle; no updater or complete boot claim")
    return 0


if __name__ == "__main__":
    sys.exit(main())
