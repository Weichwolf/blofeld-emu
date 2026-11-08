# SPDX-License-Identifier: GPL-3.0-only
"""Small independently encoded ZIP unit inputs, using Python's standard library."""

from pathlib import Path
import sys
import zipfile

root = Path(sys.argv[1])
root.mkdir(parents=True, exist_ok=True)
for name, method in (("stored", zipfile.ZIP_STORED), ("deflated", zipfile.ZIP_DEFLATED)):
    with zipfile.ZipFile(root / f"{name}.zip", "w", compression=method) as archive:
        archive.writestr("ignored.txt", b"not selected")
        archive.writestr("größere/INPUT.MID", b"abc\0def")
        archive.comment = b"ZIP unit comment"
with zipfile.ZipFile(root / "ambiguous.zip", "w") as archive:
    archive.writestr("a.mid", b"one")
    archive.writestr("b.mid", b"two")
with zipfile.ZipFile(root / "empty.zip", "w", compression=zipfile.ZIP_DEFLATED) as archive:
    archive.writestr("a.mid", b"")
damaged = bytearray((root / "stored.zip").read_bytes())
offset = damaged.index(b"abc\0def")
damaged[offset] ^= 1
(root / "damaged.zip").write_bytes(damaged)
(root / "zip.done").write_text("generated\n")
