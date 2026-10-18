#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Verify or fetch the exact local reference files listed in tracked manifests."""

import argparse
import hashlib
import json
from pathlib import Path
import tempfile
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MANIFESTS = (
    "doc/downloads.json",
    "doc/library/sources.json",
    "doc/hardware/sources.json",
    "doc/produktbilder/quellen.json",
)


def matches(path, entry):
    if not path.is_file() or path.stat().st_size != entry["bytes"]:
        return False
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() == entry["sha256"]


def fetch(path, entry):
    url = entry.get("download", entry["source"])
    if urllib.parse.urlsplit(url).scheme != "https":
        raise ValueError("Reference downloads must use HTTPS")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        request = urllib.request.Request(url, headers={"User-Agent": "Blofeld-reference-fetch/0.1"})
        with urllib.request.urlopen(request, timeout=30) as response:
            if urllib.parse.urlsplit(response.url).scheme != "https":
                raise ValueError("Reference redirected outside HTTPS")
            with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as output:
                temporary = Path(output.name)
                remaining = entry["bytes"]
                while True:
                    data = response.read(min(65536, remaining + 1))
                    if not data:
                        break
                    if len(data) > remaining:
                        raise ValueError(f"Download exceeds pinned size: {path.name}")
                    output.write(data)
                    remaining -= len(data)
        if not matches(temporary, entry):
            raise ValueError(f"Reference size or SHA-256 mismatch: {path.name}")
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--verify", action="store_true", help="Offline verification only")
    mode.add_argument("--fetch", action="store_true", help="Download missing or damaged references")
    args = parser.parse_args()
    checked = 0
    failed = []
    for relative in MANIFESTS:
        manifest = ROOT / relative
        document = json.loads(manifest.read_text())
        entries = document if isinstance(document, list) else document["files"]
        for entry in entries:
            name = entry["file"]
            if Path(name).name != name or name in (".", ".."):
                raise ValueError("Manifest reference must be a single filename")
            path = manifest.parent / name
            if not matches(path, entry) and args.fetch:
                fetch(path, entry)
            if not matches(path, entry):
                failed.append(str(path.relative_to(ROOT)))
            checked += 1
    if failed:
        parser.exit(1, "Missing or mismatching references:\n" + "\n".join(failed) + "\n")
    print(f"Verified {checked} local reference files (size and SHA-256).")


if __name__ == "__main__":
    main()
