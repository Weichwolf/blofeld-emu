# SPDX-License-Identifier: GPL-3.0-only
"""Create synthetic NVM parser seeds without manufacturer bytes."""

import argparse
import binascii
from pathlib import Path
import struct


def image(pattern, known, fresh, version=1):
    body = (b"BLFNVM\r\n" + struct.pack("<II", version, 38912)
            + bytes([pattern]) * 32768 + bytes([known]) * 4096 + bytes([fresh]) * 2048)
    return body + struct.pack("<I", binascii.crc32(body))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    valid = image(165, 255, 0)
    seeds = {
        "known": valid,
        "erased": image(255, 255, 255),
        "unknown": image(0, 0, 0),
        "partial-unknown": image(0, 170, 0),
        "invalid-history": image(165, 255, 1),
        "noncanonical-unknown": image(165, 0, 0),
        "unsupported-version": image(0, 255, 0, version=2),
        "bad-crc": valid[:-4] + bytes(4),
        "truncated": valid[:-1],
        "trailing": valid + b"x",
        "empty": b"",
    }
    for name, data in seeds.items():
        (args.output / name).write_bytes(data)
    print(f"Created {len(seeds)} synthetic NVM parser seeds in {args.output}")


if __name__ == "__main__":
    main()
