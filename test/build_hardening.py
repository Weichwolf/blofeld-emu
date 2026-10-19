#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check project/native compile hardening and actual executable ELF protections."""

import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

COMMON_FLAGS = {
    "-fstack-protector-strong", "-fstack-clash-protection",
    "-Wformat=2", "-Werror=format-security", "-D_FORTIFY_SOURCE=3",
}
WEAKENING_FLAGS = {
    "-fno-stack-protector", "-fstack-protector", "-fstack-protector-explicit",
    "-fno-stack-clash-protection", "-Wno-format", "-Wno-format-security",
    "-Wno-error=format-security", "-fno-PIC", "-fno-pic", "-fno-PIE", "-fno-pie",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_flags(arguments, cxx, libstdcxx, cf_protection):
    require(COMMON_FLAGS <= set(arguments),
            f"Missing shared hardening flags: {sorted(COMMON_FLAGS - set(arguments))}")
    require(not (WEAKENING_FLAGS & set(arguments)), "Hardening was explicitly weakened")
    macros = [arg for arg in arguments if arg.startswith(("-D_FORTIFY_SOURCE", "-U_FORTIFY_SOURCE"))]
    require(all(arg == "-D_FORTIFY_SOURCE=3" for arg in macros), "FORTIFY must remain at level 3")
    optimized = [arg for arg in arguments if re.fullmatch(r"-O(?:[0-3sgz]|fast)", arg)]
    require(optimized and optimized[-1] != "-O0", "FORTIFY requires active optimization")
    require(any(arg in arguments for arg in ("-fPIC", "-fpic", "-fPIE", "-fpie")),
            "Position-independent code must be explicitly requested")
    if cxx and libstdcxx:
        require("-D_GLIBCXX_ASSERTIONS" in arguments, "Missing libstdc++ assertions")
        require(not any(arg.startswith("-U_GLIBCXX_ASSERTIONS") for arg in arguments),
                "libstdc++ assertions were undefined")
    if cf_protection:
        options = [arg for arg in arguments if arg.startswith(("-fcf-protection", "-fno-cf-protection"))]
        require(options and options[-1] == "-fcf-protection=full",
                "Linux/x86 requires full compiler control-flow protection")


def check_commands(database, root, native, libstdcxx, cf_protection):
    counts = dict.fromkeys(["project", *native], 0)
    for entry in database:
        arguments = entry.get("arguments") or shlex.split(entry["command"])
        path = (Path(entry["directory"]) / entry["file"]).resolve()
        project = any(path.is_relative_to(root / folder) for folder in ("src", "test"))
        output = entry.get("output") or arguments[arguments.index("-o") + 1]
        matches = [name for name in native if f"CMakeFiles/{name}.dir/" in output]
        require(project or len(matches) == 1, f"Unclassified compile command: {path}")
        name = "project" if project else matches[0]
        require(path.suffix in (".c", ".cc", ".cpp", ".cxx", ".C"), f"Unknown language: {path}")
        try:
            check_flags(arguments, path.suffix != ".c", libstdcxx, cf_protection)
        except ValueError as error:
            raise ValueError(f"{name}: {path}: {error}") from error
        counts[name] += 1
    require(all(counts.values()), f"Missing project/native compile commands: {counts}")
    return counts


def output(command):
    return subprocess.run(command, check=True, capture_output=True, text=True, timeout=30).stdout


def check_elf(path, readelf):
    require(path.is_file(), f"Missing executable: {path}")
    info = output([readelf, "--file-header", "--program-headers", "--dynamic-table", str(path)])
    require(re.search(r"^\s*Type:\s+DYN\b", info, re.MULTILINE)
            and re.search(r"\(FLAGS_1\).*\bPIE\b", info), f"{path}: missing executable PIE")
    require(re.search(r"^\s*GNU_RELRO\s", info, re.MULTILINE), f"{path}: missing RELRO")
    require(re.search(r"\(FLAGS\).*\bBIND_NOW\b|\(FLAGS_1\).*\bNOW\b", info),
            f"{path}: missing immediate binding (NOW)")
    stack = [line.split() for line in info.splitlines() if line.strip().startswith("GNU_STACK ")]
    flags = "".join(stack[0][6:-1]) if len(stack) == 1 else ""
    require(flags and "E" not in flags,
            f"{path}: missing NX stack")
    require("(TEXTREL)" not in info, f"{path}: text relocations are unsupported")


def check_native_objects(native, readelf, cf_protection):
    for name, path in native.items():
        require(path.is_file(), f"Missing native archive: {name}: {path}")
        if cf_protection:
            notes = output([readelf, "--notes", str(path)])
            require(re.search(r"x86 feature:.*\bIBT\b.*\bSHSTK\b", notes),
                    f"{name}: native objects lack full compiler CF properties")


def check_negative_elf_controls(compiler, readelf):
    # Inspect inert controls; never execute a binary with deliberately weakened protection.
    with tempfile.TemporaryDirectory(prefix="blofeld-hardening-") as temporary:
        folder = Path(temporary)
        source = folder / "control.c"
        source.write_text("// SPDX-License-Identifier: GPL-3.0-only\nint main(void) { return 0; }\n")
        base = [compiler, str(source), "-fPIE", "-pie", "-Wl,-z,relro,-z,now,-z,noexecstack"]
        positive = folder / "protected"
        output([*base, "-o", str(positive)])
        check_elf(positive, readelf)
        controls = {
            "PIE": ["-fno-pie", "-no-pie"],
            "RELRO": ["-Wl,-z,norelro"],
            "NOW": ["-Wl,-z,lazy"],
            "NX stack": ["-Wl,-z,execstack"],
        }
        for protection, options in controls.items():
            binary = folder / protection.replace(" ", "-")
            output([*base, *options, "-o", str(binary)])
            try:
                check_elf(binary, readelf)
            except ValueError as error:
                require(protection in str(error), f"Wrong rejection for {protection}: {error}")
            else:
                raise ValueError(f"Accepted ELF control without {protection}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--binary-list", type=Path, required=True)
    parser.add_argument("--readelf", required=True)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--native", action="append", required=True)
    parser.add_argument("--libstdcxx", action="store_true")
    parser.add_argument("--cf-protection", action="store_true")
    args = parser.parse_args()
    try:
        native = {name: Path(path) for name, path in (entry.split("=", 1) for entry in args.native)}
        require(len(native) == len(args.native), "Duplicate native archive entry")
        counts = check_commands(json.loads(args.database.read_text()), args.source.resolve(),
                                native, args.libstdcxx, args.cf_protection)
        check_native_objects(native, args.readelf, args.cf_protection)
        binaries = [Path(line) for line in args.binary_list.read_text().splitlines() if line]
        require(binaries and len(binaries) == len(set(binaries)), "Invalid executable inventory")
        for binary in binaries:
            check_elf(binary, args.readelf)
        check_negative_elf_controls(args.compiler, args.readelf)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Build hardening verification failed: {error}\n")
    print(f"Verified shared compile hardening: {counts}; "
          f"{len(binaries)} PIE/RELRO/NOW/NX-stack executables and four negative ELF controls. "
          "Compiler CF instrumentation does not establish runtime CET or JIT protection.")


if __name__ == "__main__":
    main()
