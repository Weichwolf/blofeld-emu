# SPDX-License-Identifier: GPL-3.0-only
"""Assemble the published NXP SPI bootstrap prefix into ignored local references."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PREFIX_SHA256 = "26531751e85d9d796795e443869330c84689b6bdb1eccbafc2cbd1554e377f16"


def nxp_words(listing: Path) -> list[int]:
    text = listing.read_text()
    if not re.search(r"^0\s+Errors\s*$", text, re.M) or not re.search(
            r"^0\s+Warnings\s*$", text, re.M):
        raise ValueError("NXP assembly must report zero errors and warnings")
    words = {}
    next_address = None
    for line in text.splitlines():
        match = re.match(r"^\d+\s+P:([A-F0-9]{6})\s+([A-F0-9]{6})\b", line)
        if match:
            address, word = (int(value, 16) for value in match.groups())
            if address in words:
                raise ValueError("Overlapping NXP program words")
            words[address] = word
            next_address = address + 1
        elif next_address is not None:
            match = re.fullmatch(r"\s+([A-F0-9]{6})\s*", line)
            if match:
                if next_address in words:
                    raise ValueError("Overlapping NXP extension words")
                words[next_address] = int(match[1], 16)
                next_address += 1
    if sorted(words) != list(range(0xFF0000, 0xFF0000 + len(words))):
        raise ValueError("Unexpected gaps in the NXP prefix")
    return [words[address] for address in sorted(words)]


def assemble_nxp(assembler: Path, source: Path, output: Path) -> tuple[list[int], str]:
    assembler = assembler.resolve()
    tools = json.loads((ROOT / "doc/reference-tools.json").read_text())["files"][0]["tools"]
    expected = next(tool for tool in tools if tool["file"].endswith("/asm56300.exe"))
    binary = assembler.read_bytes()
    if len(binary) != expected["bytes"] or hashlib.sha256(binary).hexdigest() != expected["sha256"]:
        raise ValueError("The NXP assembler does not match the pinned reference tool")
    command = [str(assembler)]
    if binary[:2] == b"MZ":
        if not os.environ.get("WINEPREFIX"):
            raise ValueError("Set the dedicated WINEPREFIX explicitly for the Windows assembler")
        command.insert(0, "wine")
    listing = output / "nxp-spi-bootstrap.lst"
    result = subprocess.run([*command, "-a", "-bnxp-spi-bootstrap.coff",
                             "-l" + listing.name, source.name], cwd=output,
                            capture_output=True, text=True, check=True)
    report = result.stdout + result.stderr
    (output / "nxp-spi-bootstrap-assembly.log").write_text(report)
    version = "Freescale Semiconductor DSP56300 Assembler  Version 6.3.28"
    if version not in report:
        raise ValueError("Unexpected NXP assembler version")
    return nxp_words(listing), version


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nxp-assembler", required=True, type=Path,
                        help="Path to the optional NXP asm56300 6.3.28 reference tool")
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
    assembler = args.nxp_assembler.resolve()
    words, version = assemble_nxp(assembler, source, output)
    data = b"".join(word.to_bytes(3, "big") for word in words)
    if len(words) != 55 or hashlib.sha256(data).hexdigest() != PREFIX_SHA256:
        raise ValueError("Assembly differs from the pinned document-derived prefix")
    binary = output / "nxp-spi-bootstrap.bin"
    binary.write_bytes(data)
    provenance = {
        "manual_sha256": reference["sha256"],
        "assembler_version": version,
        "assembler_archive": "doc/reference-tools.json",
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
