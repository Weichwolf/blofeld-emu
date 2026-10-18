# SPDX-License-Identifier: GPL-3.0-only
"""Generate synthetic DSP bus/map/ROM inputs; never load manufacturer data."""

import argparse
from pathlib import Path


def record(action: int, area: int = 0, address: int = 0, value: int = 0) -> bytes:
    return bytes((action, area)) + address.to_bytes(3, "big") + value.to_bytes(4, "big")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    output = parser.parse_args().output
    output.mkdir(parents=True, exist_ok=True)
    switch = [(1, 1, 0x10000, 0x123456), (3, 0, 0, 0x600080), (4, 0, 0, 2),
              (0, 0, 0x1000, 0), (4, 0, 0, 1), (0, 0, 0x1000, 0),
              (1, 0, 0x1000, 0xABCDEF), (3, 0, 0, 0), (4, 0, 0, 3),
              (0, 1, 0x10000, 0), (5, 0, 0, 0), (0, 1, 0x10000, 0), (7, 0, 0, 0)]
    (output / "memory-switch-retention").write_bytes(b"".join(record(*row) for row in switch))
    pll = [(0, 1, 0xFFFFFD, 0), (10, 0, 0, 0), (9, 0, 0, 1), (5, 0, 0, 0),
           (0, 1, 0xFFFFFD, 0), (1, 1, 0xFFFFFD, 0x03E10B), (10, 0, 0, 0),
           (1, 1, 0xFFFFFD, 0x03A00B), (10, 0, 0, 0), (1, 1, 0xFFFFFD, 0x04681D),
           (1, 1, 0xFFFFFD, 0x00601D), (1, 1, 0xFFFFFD, 0x046000),
           (9, 0, 0, 2), (5, 0, 0, 0), (0, 1, 0xFFFFFD, 0),
           (1, 1, 0xFFFFFD, 0x1FE7FF), (10, 0, 0, 0)]
    (output / "pll-pinit-and-targets").write_bytes(b"".join(record(*row) for row in pll))
    for area in range(3):
        base = 0xFF0000 if area == 0 else 0x4000
        rows = [(6, area, base, 0x123456), (0, area, base, 0), (0, area, base + 2, 0),
                (1, area, base, 0), (6, area, base, 0xFFFFFFFF), (2, 0, base, 0),
                (1, 1, 0xFFFFFC, 0), (0, 1, 0xFFFFFC, 0), (1, 1, 0xFFFFFE, 1),
                (3, 0, 0, 0x80), (0, 3, 0, 0)]
        (output / f"rom-space-{area}-rejections").write_bytes(b"".join(record(*row) for row in rows))


if __name__ == "__main__":
    main()
