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


def native_listing(path: Path) -> tuple[list[int], dict[str, int]]:
    words, symbols = {}, {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) != 3 or fields[0] not in ("P", "I"):
            raise ValueError("Unexpected native assembler record")
        if fields[0] == "P":
            address, word = (int(value, 16) for value in fields[1:])
            if address in words or not 0 <= word <= 0xFFFFFF:
                raise ValueError("Overlapping or out-of-width native program words")
            words[address] = word
        else:
            name, value = fields[2].lower(), int(fields[1], 16)
            if name in symbols:
                raise ValueError("Duplicate native assembler symbol")
            symbols[name] = value
    if not words or sorted(words) != list(range(0xFF0000, 0xFF0000 + len(words))):
        raise ValueError("Unexpected gaps in the native SPI prefix")
    return [words[address] for address in sorted(words)], symbols


def native_assembly(assembler: Path, source: Path, output: Path):
    listing = output / (source.stem + ".native.lst")
    records = output / (source.stem + ".native.lod")
    result = subprocess.run([str(assembler), str(source), "-f", "lod", "-o", str(records),
                             "-l", str(listing)], capture_output=True, text=True,
                            check=True, timeout=10)
    report = result.stdout + result.stderr
    (output / (source.stem + ".native.log")).write_text(report)
    if re.search(r"\bwarning\b", report, re.I):
        raise ValueError("Native SPI assembly must report zero warnings")
    words, symbols = native_listing(records)
    return words, symbols, listing


def resolve_native_branches(source: Path, listing: Path, symbols: dict[str, int]):
    positions = {}
    for line in listing.read_text().splitlines():
        match = re.search(r":(\d+)\s+P:([A-F0-9]{6})\b", line)
        if match:
            positions[int(match[1])] = int(match[2], 16)
    pattern = re.compile(r"(\s*bra\s+)<\s*(\*|[A-Za-z_]\w*)(\s*(?:;.*)?)", re.I)
    lines = source.read_text().splitlines()
    branches = [(index, pattern.fullmatch(line)) for index, line in enumerate(lines)]
    branches = [(index, match) for index, match in branches if match]
    if len(branches) != 3 or any(index + 1 not in positions for index, _ in branches):
        raise ValueError("Unexpected NXP forced-short branch layout")
    addresses = [positions[index + 1] for index, _ in branches]
    bindings = []
    for index, match in branches:
        old_pc = positions[index + 1]
        target = old_pc if match[2] == "*" else symbols[match[2].lower()]
        relocated = target - sum(address < target for address in addresses)
        pc = old_pc - sum(address < old_pc for address in addresses)
        if not -256 <= relocated - pc <= 255:
            raise ValueError("NXP forced-short branch exceeds its signed nine-bit range")
        lines[index] = match[1] + f"<${relocated:06x}" + match[3]
        bindings.append({"line": index + 1, "target": match[2], "pc": pc, "resolved": relocated})
    return "\n".join(lines) + "\n", bindings


def assemble_native(assembler: Path, source: Path, output: Path) -> tuple[list[int], str]:
    version = subprocess.check_output([str(assembler), "--version"], text=True, timeout=10).strip()
    if version != "dsp56300-asm 1.0.0":
        raise ValueError("Unexpected native DSP assembler version")
    words, symbols, listing = native_assembly(assembler, source, output)
    if len(words) != 58:
        raise ValueError("Unexpected native pre-relaxation prefix size")
    # Version 1.0.0 discards symbolic forced-short BRA hints; retain NXP's exact encoding.
    resolved, bindings = resolve_native_branches(source, listing, symbols)
    normalized = output / "nxp-spi-bootstrap-resolved.asm"
    normalized.write_text(resolved)
    (output / "nxp-spi-bootstrap-relocations.json").write_text(json.dumps(bindings, indent=2) + "\n")
    words, _, _ = native_assembly(assembler, normalized, output)
    return words, version


def extract_source(manual: Path, output: Path) -> Path:
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
    lines.extend([
        "SerialEEPROM equ $ffff00 ; Address from the separate published EEPROM listing.",
        "BURN: ; Original following code begins here; that body is deliberately absent.",
    ])
    source = output / "nxp-spi-bootstrap.asm"
    source.write_text("\n".join(lines) + "\n")
    return source


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    assembler = parser.add_mutually_exclusive_group(required=True)
    assembler.add_argument("--nxp-assembler", type=Path,
                           help="Path to the optional NXP asm56300 6.3.28 reference tool")
    assembler.add_argument("--native-assembler", type=Path,
                           help="Path to the optional dsp56300-asm 1.0.0 offline tool")
    parser.add_argument("--manual", type=Path, help="Path to the pinned NXP DSP56371 manual")
    parser.add_argument("--output", type=Path, default=ROOT / "build/local")
    args = parser.parse_args()
    reference = next(row for row in json.loads((ROOT / "doc/hardware/sources.json").read_text())
                     if row["file"] == "DSP56371UM.pdf")
    local_manual = ROOT / "build/local/reference-inputs" / reference["file"]
    manual = args.manual or (local_manual if local_manual.exists()
                             else ROOT / "doc/hardware" / reference["file"])
    data = manual.read_bytes()
    if len(data) != reference["bytes"] or hashlib.sha256(data).hexdigest() != reference["sha256"]:
        raise ValueError("The downloaded NXP manual does not match the pinned reference")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    source = extract_source(manual.resolve(), output)
    executable = (args.nxp_assembler or args.native_assembler).resolve()
    assemble = assemble_nxp if args.nxp_assembler else assemble_native
    words, version = assemble(executable, source, output)
    data = b"".join(word.to_bytes(3, "big") for word in words)
    if len(words) != 55 or hashlib.sha256(data).hexdigest() != PREFIX_SHA256:
        raise ValueError("Assembly differs from the pinned document-derived prefix")
    binary = output / "nxp-spi-bootstrap.bin"
    binary.write_bytes(data)
    provenance = {
        "manual_sha256": reference["sha256"],
        "assembler_version": version,
        "manual_source": reference["source"],
        "manual_bytes": reference["bytes"],
        "assembler_reference": "doc/reference-tools.json" if args.nxp_assembler
                                else "cmake/dependencies.json:dsp56300_assembly_tool",
        "assembler_bytes": executable.stat().st_size,
        "assembler_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "binary_sha256": PREFIX_SHA256,
        "words": len(words),
        "status": "Document-derived partial SPI ROM, not a hardware dump; other modes absent.",
    }
    if args.nxp_assembler:
        provenance["assembler_archive"] = "doc/reference-tools.json"
    else:
        provenance["resolved_source_sha256"] = hashlib.sha256(
            (output / "nxp-spi-bootstrap-resolved.asm").read_bytes()).hexdigest()
        provenance["relocations_sha256"] = hashlib.sha256(
            (output / "nxp-spi-bootstrap-relocations.json").read_bytes()).hexdigest()
    (output / "nxp-spi-bootstrap-provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Wrote {len(words)} document-derived words to {binary}")


if __name__ == "__main__":
    main()
