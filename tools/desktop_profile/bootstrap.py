#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Assemble the common documented Blofeld DSP bootstrap from a local NXP PDF.

Manufacturer instructions/data are extracted locally, never embedded here.
The package holds the 192-word main block and the 81-word EEPROM block.
"""

import hashlib
import re
import subprocess

BOOT_SHA256 = "4d72a5bd75ad086ec7190c6c25b4438aa1cfaa2ddc0ac510055c2540e886de56"
PREFIX_SHA256 = "26531751e85d9d796795e443869330c84689b6bdb1eccbafc2cbd1554e377f16"
EEPROM_SHA256 = "8977eec109d4671b2af466c4b10cd63771d4d84ec5bf396c81bbbb195afbd1f6"
PACKAGE_SHA256 = "53dc6e190185c4e77995c758242ab2a7ae209f646a77d4ba142b95d8a89413e1"
BASE, END = 0xff0000, 0xff00c0
EEPROM_BASE, EEPROM_END = 0xffff00, 0xffff51


def require(condition, message):
    if not condition:
        raise ValueError(message)


def extract(text):
    start = text.index("PROMADDR equ")
    end = re.search(r"^\s*end\s*$", text[start:], re.I | re.M)
    require(end is not None, "Missing Appendix A.1 terminator")
    after = start + end.end()
    eeprom = re.search(r"\bBootROM\s+equ\s+\$([0-9a-f]+)", text[after:], re.I)
    require(eeprom is not None, "Missing separate EEPROM origin")
    lines = []
    for line in text[start:after].replace("\f", "\n").splitlines():
        if ("DSP56371 24-Bit" in line or "Freescale Semiconductor" in line or
                re.fullmatch(r"\s*A-\d+\s*", line)):
            continue
        body = line.split(";", 1)[0].strip()
        if body:
            lines.append(body)
    return lines, int(eeprom[1], 16)


def normalize(lines, eeprom):
    # Resolve the document's single EQUALDATA=0 choice and listing syntax.
    require(any(re.fullmatch(r"EQUALDATA\s+equ\s+0", line, re.I) for line in lines),
            "Unexpected RAM-layout conditional")
    require(any(re.fullmatch(r"dup\s+\$FF00C0\s*-\s*\*", line, re.I) for line in lines),
            "Missing documented address-fill directive")
    patterns = [line for line in lines if re.fullmatch(r"dc\s+\$[0-9a-f]+", line, re.I)]
    require(len(patterns) == 4 and any(re.fullmatch(r"PATTERNS\s+dsm\s+4", line, re.I)
                                     for line in lines), "Unexpected modulo-aligned data table")
    active, conditional, body = True, False, []
    for line in lines:
        if re.fullmatch(r"if\s*\(EQUALDATA\)", line, re.I):
            require(not conditional, "Unexpected nested conditional")
            active, conditional = False, True
        elif line.lower() == "else":
            require(conditional and not active, "Unexpected conditional branch")
            active = True
        elif line.lower() == "endif":
            require(conditional, "Unmatched conditional end")
            active, conditional = True, False
        elif active:
            if re.fullmatch(r"[a-z_]\w*", line, re.I) and line.lower() not in (
                    "nop", "enddo", "debug", "wait"):
                line += ":"
            line = re.sub(r"^move(\s+.*\bp:.*)$", r"movem\1", line, flags=re.I)
            body.append(line)
            if line.upper() == "BURN_END:":
                break
    require(body and body[-1].upper() == "BURN_END:", "Missing main-block end label")
    constants = {match[1].lower(): int(match[2][1:], 16) if match[2].startswith("$")
                 else int(match[2]) for line in body
                 if (match := re.fullmatch(r"(\w+)\s+equ\s+(\$[0-9a-f]+|\d+)", line, re.I))}
    constants["num_patterns"] = len(patterns)
    # Version 1.0.0 otherwise keeps long encodings for symbolic immediates.
    for index, line in enumerate(body):
        if re.match(r"\w+\s+equ\b", line, re.I):
            continue
        line = re.sub(r"(#(?:[<>])?)([a-z_]\w*)", lambda match:
                      match[1] + f"${constants[match[2].lower()]:x}"
                      if match[2].lower() in constants else match[0], line, flags=re.I)
        line = re.sub(r"#<\(NUM_PATTERNS-1\)", f"#<{len(patterns) - 1}", line, flags=re.I)
        body[index] = line
    header = f"SerialEEPROM equ ${eeprom:x}\nNUM_PATTERNS equ {len(patterns)}\n"
    return header + "\n".join(body) + "\nPATTERNS:\n" + "\n".join(patterns) + "\nend\n"


def extract_eeprom(text):
    start = re.search(r"^\s*TITLE\s+'SerialBootloader'", text, re.I | re.M)
    require(start is not None, "Missing Appendix A.2 source title")
    section = text[start.start():]
    end = re.search(r"^\s*ENDSEC\b[^\n]*", section, re.I | re.M)
    require(end is not None, "Missing Appendix A.2 terminator")
    lines = []
    for line in section[:end.end()].replace("\f", "\n").splitlines():
        if ("DSP56371 24-Bit" in line or "Freescale Semiconductor" in line or
                re.fullmatch(r"\s*A-\d+\s*", line)):
            continue
        body = line.split(";", 1)[0].strip()
        if not body or re.match(r"^(TITLE|SECTION|ENDSEC)\b", body, re.I):
            continue
        # Canonical spellings for assembler 1.0.0, with identical instructions.
        body = re.sub(r"^move(\s+.*\bp:.*)$", r"movem\1", body, flags=re.I)
        body = re.sub(r"^jmp\s+(r[0-7])$", r"jmp (\1)", body, flags=re.I)
        body = re.sub(r"^bra\s+([a-z_]\w*)$", r"bra <\1", body, flags=re.I)
        lines.append(body)
    require(any(re.fullmatch(r"BootROM\s+equ\s+\$ffff00", line, re.I)
                for line in lines), "Unexpected EEPROM origin")
    return "\n".join(lines) + "\nend\n"


def assemble(assembler, source, directory, name):
    asm, lod, listing = [directory / (name + suffix) for suffix in (".asm", ".lod", ".lst")]
    asm.write_text(source)
    result = subprocess.run([str(assembler), str(asm), "-f", "lod", "-o", str(lod),
                             "-l", str(listing)], capture_output=True, text=True, timeout=10)
    (directory / (name + ".log")).write_text(result.stdout + result.stderr)
    require(result.returncode == 0 and not re.search(r"\bwarning\b", result.stdout + result.stderr,
                                                   re.I), f"Assembly failed: {name}; inspect log")
    words, symbols = {}, {}
    for line in lod.read_text().splitlines():
        fields = line.split()
        require(len(fields) == 3 and fields[0] in ("P", "I"), "Unexpected assembler record")
        if fields[0] == "P":
            address, value = int(fields[1], 16), int(fields[2], 16)
            require(address not in words and 0 <= value <= 0xffffff, "Invalid/overlapping word")
            words[address] = value
        else:
            require(fields[2].lower() not in symbols, "Duplicate assembler symbol")
            symbols[fields[2].lower()] = int(fields[1], 16)
    return words, symbols, listing.read_text()


def relax(source, symbols, listing, expected_branches=4):
    positions = {int(match[1]): int(match[2], 16) for line in listing.splitlines()
                 if (match := re.search(r":(\d+)\s+P:([a-f0-9]{6})\b", line, re.I))}
    lines = source.splitlines()
    pattern = re.compile(r"(\s*bra\s+)<\s*(\*|[a-z_]\w*)\s*", re.I)
    branches = [(index, match) for index, line in enumerate(lines)
                if (match := pattern.fullmatch(line))]
    require(len(branches) == expected_branches and
            all(index + 1 in positions for index, _ in branches),
            "Unexpected document forced-short branches")
    addresses = [positions[index + 1] for index, _ in branches]
    for index, match in branches:
        old = positions[index + 1]
        target = old if match[2] == "*" else symbols[match[2].lower()]
        target -= sum(address < target for address in addresses)
        pc = old - sum(address < old for address in addresses)
        require(-256 <= target - pc <= 255, "Resolved branch is outside short range")
        lines[index] = match[1] + f"<${target:06x}"
    return "\n".join(lines) + "\n"


def build_boot_rom(manual, assembler, pdftotext, directory):
    version = subprocess.check_output([assembler, "--version"], text=True, timeout=10).strip()
    require(version == "dsp56300-asm 1.0.0", "Unexpected native assembler version")
    directory.mkdir()
    text = directory / "manual.txt"
    subprocess.run([pdftotext, "-layout", manual, text], check=True, timeout=20)
    lines, eeprom = extract(text.read_text())
    source = normalize(lines, eeprom)
    _, symbols, listing = assemble(assembler, source, directory, "initial")
    source = relax(source, symbols, listing)
    _, symbols, _ = assemble(assembler, source, directory, "short-branches")
    end = symbols["burn_end"]
    aligned = (end + 3) // 4 * 4
    require(BASE < end <= aligned + 4 <= END, "Invalid documented boot layout")
    before, after = source.split("PATTERNS:\n")
    patterns = after.split("end\n")[0]
    source = (before + "dc *\n" * (aligned - end) + "PATTERNS:\n" + patterns +
              "dc *\n" * (END - aligned - 4) + "end\n")
    words, _, _ = assemble(assembler, source, directory, "complete")
    require(sorted(words) == list(range(BASE, END)), "Incomplete or oversized main boot block")
    data = b"".join(words[address].to_bytes(3, "big") for address in range(BASE, END))
    require(hashlib.sha256(data).hexdigest() == BOOT_SHA256 and
            hashlib.sha256(data[:165]).hexdigest() == PREFIX_SHA256,
            "Bootstrap differs from the independently assembled common reference")
    require(eeprom == EEPROM_BASE, "Unexpected separate EEPROM origin")
    eeprom_source = extract_eeprom(text.read_text())
    _, symbols, listing = assemble(assembler, eeprom_source, directory, "eeprom-initial")
    eeprom_source = relax(eeprom_source, symbols, listing, expected_branches=3)
    eeprom_words, _, _ = assemble(assembler, eeprom_source, directory, "eeprom")
    require(sorted(eeprom_words) == list(range(EEPROM_BASE, EEPROM_END)),
            "Incomplete or oversized EEPROM boot block")
    data += b"".join(eeprom_words[address].to_bytes(3, "big")
                     for address in range(EEPROM_BASE, EEPROM_END))
    require(hashlib.sha256(data[(END - BASE) * 3:]).hexdigest() == EEPROM_SHA256 and
            hashlib.sha256(data).hexdigest() == PACKAGE_SHA256,
            "EEPROM bootstrap or packaged ROM differs from the documented reference")
    return data
