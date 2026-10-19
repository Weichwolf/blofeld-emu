#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Fetch or verify the pinned zlib source archive, without running a build."""

import argparse
import json
from pathlib import Path

from references import fetch, matches

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT / "doc/dependencies.json").read_text())["zlib"]
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--fetch", action="store_true")
    mode.add_argument("--verify", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "doc/local" / manifest["file"])
    args = parser.parse_args()
    if not matches(args.output, manifest) and args.fetch:
        fetch(args.output, manifest)
    if not matches(args.output, manifest):
        parser.exit(1, f"Missing or mismatching pinned zlib archive: {args.output}\n")
    print(f"Verified zlib {manifest['version']} archive (size and SHA-256): {args.output}")


if __name__ == "__main__":
    main()
