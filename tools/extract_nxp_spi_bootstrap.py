# SPDX-License-Identifier: GPL-3.0-only
"""Assemble the published NXP SPI bootstrap prefix into ignored local references."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PREFIX_SHA256 = "8d5193c52b22cfdd633bcdd2acf8651a498e361ff06e4716454fddf92be11901"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembler", required=True, type=Path,
                        help="Path to xemu-project/dsp56300 dsp56300-asm v1.0.0")
    args = parser.parse_args()
    reference = next(row for row in json.loads((ROOT / "doc/hardware/sources.json").read_text())
                     if row["file"] == "DSP56371UM.pdf")
    manual = ROOT / "doc/hardware" / reference["file"]
    if hashlib.sha256(manual.read_bytes()).hexdigest() != reference["sha256"]:
        raise ValueError("The downloaded NXP manual does not match the pinned reference")
    output = ROOT / "doc/local"
    output.mkdir(parents=True, exist_ok=True)
    text_path = output / "DSP56371UM.txt"
    subprocess.run(["pdftotext", "-layout", str(manual), str(text_path)], check=True)
    text = text_path.read_text()
    first = text.index("PROMADDR equ")
    last = text.index("; Code for burn-in", first)
    lines = []
    for line in text[first:last].replace("\f", "\n").splitlines():
        if ("DSP56371 24-Bit" in line or "Freescale Semiconductor" in line
                or re.match(r"^\s*A-\d+\s*$", line)):
            continue
        if re.match(r"^[A-Za-z_][A-Za-z0-9_]*\s*$", line):
            line = line.rstrip() + ":"
        lines.append(line)
    # Resolve references to omitted modes using the source's layout. No body is
    # supplied for either target; a DSP access outside the prefix stays unknown.
    lines.extend([
        "SerialEEPROM equ $ffff00 ; Address from the separate published EEPROM listing.",
        "BURN: ; Original following code begins here; that body is deliberately absent.",
    ])
    source = output / "nxp-spi-bootstrap.asm"
    source.write_text("\n".join(lines) + "\n")
    listing = output / "nxp-spi-bootstrap.lod"
    assembler = args.assembler.resolve()
    version = subprocess.check_output([str(assembler), "--version"], text=True).strip()
    if version != "dsp56300-asm 1.0.0":
        raise ValueError(f"Unexpected assembly tool version: {version}")
    subprocess.run([str(assembler), str(source), "-f", "lod", "-o", str(listing)], check=True)
    words = []
    for line in listing.read_text().splitlines():
        fields = line.split()
        if fields[0] == "P":
            address, word = int(fields[1], 16), int(fields[2], 16)
            if address != 0xFF0000 + len(words):
                raise ValueError("Unexpected gaps/overlap in the document-derived prefix")
            words.append(word)
    data = b"".join(word.to_bytes(3, "big") for word in words)
    if len(words) != 58 or hashlib.sha256(data).hexdigest() != PREFIX_SHA256:
        raise ValueError("Assembly differs from the pinned document-derived prefix")
    binary = output / "nxp-spi-bootstrap.bin"
    binary.write_bytes(data)
    provenance = {
        "manual_sha256": reference["sha256"],
        "assembler_version": version,
        "assembler_sha256": hashlib.sha256(assembler.read_bytes()).hexdigest(),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "binary_sha256": PREFIX_SHA256,
        "words": len(words),
        "status": "Document-derived partial SPI ROM, not a hardware dump; other modes absent.",
    }
    (output / "nxp-spi-bootstrap-provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Wrote {len(words)} document-derived words to ignored {binary.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
