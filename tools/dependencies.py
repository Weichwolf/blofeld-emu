#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Fetch or verify a pinned native dependency or Python build-tool archive."""

import argparse
import json
from pathlib import Path

from references import fetch, matches

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifests = json.loads((ROOT / "doc/dependencies.json").read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--fetch", action="store_true")
    mode.add_argument("--verify", action="store_true")
    parser.add_argument("--component", choices=("zlib", "pulseaudio", "libsndfile", "meson"),
                        default="zlib")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    manifest = manifests[args.component]
    args.output = args.output or ROOT / "doc/local" / manifest["file"]
    if not matches(args.output, manifest) and args.fetch:
        fetch(args.output, manifest)
    if not matches(args.output, manifest):
        parser.exit(1, f"Missing or mismatching pinned {args.component} archive: {args.output}\n")
    print(f"Verified {args.component} {manifest['version']} archive (size and SHA-256): {args.output}")


if __name__ == "__main__":
    main()
