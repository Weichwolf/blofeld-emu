#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Use pinned upstream Meson configuration for static CMake client targets."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess


def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace(';', '\\;') + '"'


def recipe(entries, target):
    sources, settings = [], set()
    for entry in entries:
        directory = Path(entry["directory"])
        sources.append((directory / entry["file"]).resolve())
        arguments = shlex.split(entry["command"])[1:]
        includes, definitions = [], []
        cursor = 0
        while cursor < len(arguments):
            argument = arguments[cursor]
            if argument in ("-o", "-c", "-MQ", "-MF"):
                cursor += 2
                continue
            if argument.startswith("-I"):
                includes.append((directory / argument[2:]).resolve())
            elif argument.startswith("-D"):
                definitions.append(argument[2:])
            elif argument not in ("-fdiagnostics-color=always", "-Wall", "-Winvalid-pch",
                                  "-std=gnu11", "-O0", "-g", "-fPIC", "-pthread", "-MD"):
                raise ValueError(f"Unreviewed upstream compiler argument: {argument}")
            cursor += 1
        settings.add((tuple(includes), tuple(definitions)))
    if len(settings) != 1:
        raise ValueError(f"Expected one upstream configuration for {target}")
    includes, definitions = settings.pop()
    lines = [f"add_library({target} STATIC", *(f"  {quote(p)}" for p in sources), ")",
             f"target_include_directories({target} SYSTEM PUBLIC",
             *(f"  {quote(p)}" for p in includes), ")",
             f"target_compile_definitions({target} PRIVATE",
             *(f"  {quote(d)}" for d in definitions), ")",
             f"target_compile_options({target} PRIVATE -pthread)",
             f"set_target_properties({target} PROPERTIES C_STANDARD 11 C_EXTENSIONS ON)"]
    return "\n".join(lines) + "\n"


def configure(args):
    args.build.mkdir(parents=True, exist_ok=True)
    pkgconfig = args.build / "pkgconfig"
    pkgconfig.mkdir(exist_ok=True)
    (pkgconfig / "sndfile.pc").write_text(
        "Name: sndfile\nDescription: pinned instrumented static sndfile\nVersion: 1.2.2\n"
        f"Libs: -L{args.sndfile_build} -lsndfile\n"
        f"Cflags: -I{args.sndfile_source / 'include'}\n")
    env = os.environ.copy()
    env.update(CC=str(args.compiler), PYTHONPATH=str(args.meson),
               PKG_CONFIG_PATH=str(pkgconfig))
    command = [os.sys.executable, "-m", "mesonbuild.mesonmain", "setup",
               str(args.build), str(args.source), "--auto-features=disabled",
               "-Ddaemon=false", "-Dclient=true", "-Dtests=false", "-Dman=false",
               "-Ddoxygen=false", "-Ddatabase=simple", "-Dbashcompletiondir=no",
               "-Dzshcompletiondir=no"]
    if (args.build / "meson-private/coredata.dat").exists():
        command.append("--reconfigure")
    with (args.build / "configure.log").open("w") as log:
        result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                check=False, timeout=120)
    if result.returncode:
        raise ValueError((args.build / "configure.log").read_text()[-5000:])
    entries = json.loads((args.build / "compile_commands.json").read_text())
    output = "# Generated from pinned upstream configuration; no source changes.\n"
    for target, marker, count in (("blofeld_pulse_common", "src/libpulsecommon-17.0.so.p/", 71),
                                  ("blofeld_pulse", "src/pulse/libpulse.so.0.24.3.p/", 25)):
        selected = [entry for entry in entries if entry["output"].startswith(marker)]
        if len(selected) != count:
            raise ValueError(f"Unexpected {target} source count: {len(selected)}")
        if not all((Path(entry["directory"]) / entry["file"]).resolve().is_relative_to(args.source)
                   for entry in selected):
            raise ValueError("Native client source escapes the pinned upstream tree")
        output += recipe(selected, target)
    destination = args.build / "native-targets.cmake"
    if not destination.exists() or destination.read_text() != output:
        destination.write_text(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "build", "meson", "compiler", "sndfile-source", "sndfile-build"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    for name, value in vars(args).items():
        setattr(args, name, value.resolve())
    configure(args)


if __name__ == "__main__":
    main()
