#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Check native compile flags, built sanitizer hooks and absence of system zlib."""

import argparse
import json
from pathlib import Path
import re
import shlex
import subprocess

DEPENDENCY_SANITIZERS = {
    "address-undefined": {"address", "undefined", "float-divide-by-zero"},
    "thread": {"thread"},
    "none": set(),
}
EXTRA_PROJECT_SANITIZERS = {"unsigned-integer-overflow", "implicit-conversion"}
HOOKS = {"address": "__asan_", "undefined": "__ubsan_", "thread": "__tsan_"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_flags(arguments, mode, project):
    active = set()
    for argument in arguments:
        if argument.startswith("-fsanitize="):
            active.update(argument.split("=", 1)[1].split(","))
        elif argument.startswith("-fno-sanitize="):
            disabled = set(argument.split("=", 1)[1].split(","))
            active = set() if "all" in disabled else active - disabled
        require(not argument.startswith(("-fsanitize-ignorelist=", "-fsanitize-blacklist=")),
                "Sanitizer ignore lists cannot establish full dependency instrumentation")
    expected = DEPENDENCY_SANITIZERS[mode]
    if project and mode == "address-undefined":
        expected = expected | EXTRA_PROJECT_SANITIZERS
    require(expected <= active, f"Missing required sanitizer flags: {sorted(expected - active)}")
    if mode == "none":
        require(not active, f"Unexpected sanitizers in uninstrumented build: {sorted(active)}")
    else:
        require("-fno-sanitize-recover=all" in arguments,
                "Instrumentation must stop on a sanitizer finding")
        require(not any(argument.startswith("-fsanitize-recover=") for argument in arguments),
                "Recovering sanitizer findings cannot establish fail-stop checks")
        require(not ({"address", "thread"} <= active), "ASan and TSan must run separately")


def check_commands(database, root, mode, native):
    counts = dict.fromkeys(["project", *native], 0)
    for entry in database:
        arguments = entry.get("arguments") or shlex.split(entry["command"])
        output = entry.get("output") or arguments[arguments.index("-o") + 1]
        matches = [name for name in native if f"CMakeFiles/{name}.dir/" in output]
        path = Path(entry["file"])
        path = (Path(entry["directory"]) / path).resolve()
        project = any(path.is_relative_to(root / folder) for folder in ("src", "test"))
        if not project and not matches:
            continue
        name = "project" if project else matches[0]
        try:
            check_flags(arguments, mode, project)
        except ValueError as error:
            raise ValueError(f"{name}: {path}: {error}") from error
        counts[name] += 1
    for name, count in counts.items():
        require(count != 0, f"No native compile commands found for {name}")
    return counts


def output(command):
    return subprocess.run(command, capture_output=True, text=True, check=True, timeout=30).stdout


def check_artifacts(native, binary, mode, nm, readelf):
    for name, path in native.items():
        require(path.is_file(), f"Missing built archive for {name}: {path}")
        symbols = output([nm, "--undefined-only", str(path)])
        for sanitizer, hook in HOOKS.items():
            expected = sanitizer in DEPENDENCY_SANITIZERS[mode]
            require((hook in symbols) == expected,
                    f"{name}: actual object hooks disagree with {mode}: {hook}")
    needed = output([readelf, "--dynamic", str(binary)])
    require(not re.search(r"\[libz\.so(?:\.[^]]*)?\]", needed),
            "Executable still depends on an uncontrolled system libz.so")
    defined = output([nm, "--defined-only", str(binary)])
    for symbol in ("inflate", "crc32"):
        require(re.search(rf"\bT {symbol}$", defined, re.MULTILINE),
                f"Executable does not contain the required static zlib implementation: {symbol}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--database", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--mode", choices=DEPENDENCY_SANITIZERS, required=True)
    parser.add_argument("--nm", required=True)
    parser.add_argument("--readelf", required=True)
    parser.add_argument("--native", action="append", required=True, metavar="TARGET=ARCHIVE")
    args = parser.parse_args()
    native = {name: Path(path) for name, path in (entry.split("=", 1) for entry in args.native)}
    try:
        counts = check_commands(json.loads(args.database.read_text()), args.source.resolve(),
                                args.mode, native)
        check_artifacts(native, args.binary, args.mode, args.nm, args.readelf)
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Native dependency verification failed: {error}\n")
    print(f"Verified {args.mode} compile commands and native object hooks: {counts}; static zlib.")


if __name__ == "__main__":
    main()
