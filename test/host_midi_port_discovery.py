#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Inspect and route only owned virtual application ports using ordinary aconnect."""

import hashlib
import json
import os
from pathlib import Path
import re
import select
import shutil
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def main():
    if shutil.which("aconnect") is None:
        print("SKIP: ordinary aconnect tool unavailable")
        return 77
    binary = Path(sys.argv[1]).resolve()
    destination = Path(sys.argv[2])
    destination.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    environment = os.environ.copy()
    environment["LC_ALL"] = "C"
    child = subprocess.Popen([str(binary), "--discovery"], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, env=environment)
    report = {"status": "running", "pid": child.pid, "binary_sha256": digest,
              "fuzz_execution": False, "physical_traffic": False,
              "whole_goal_complete": False}

    def aconnect(*arguments):
        return subprocess.check_output(["aconnect", *arguments], text=True,
                                       env=environment, timeout=5)

    try:
        require(select.select([child.stdout], [], [], 5)[0],
                "Port did not publish its actual address")
        first = child.stdout.readline().strip()
        if first.startswith("SKIP:"):
            child.communicate(timeout=5)
            require(child.returncode == 77, "Availability refusal must be an explicit skip")
            report.update(status="skipped", reason=first)
            return 77
        match = re.fullmatch(r"MIDI source=(\d+):(\d+) destination=(\d+):(\d+)", first)
        require(match, f"Unexpected address report: {first}")
        source_client, source_port, dest_client, dest_port = map(int, match.groups())
        report["source"] = [source_client, source_port]
        report["destination"] = [dest_client, dest_port]
        before = aconnect("-l")
        for client, name in ((source_client, "source"), (dest_client, "destination")):
            line = next(line for line in before.splitlines()
                        if line.startswith(f"client {client}:"))
            require(f"'Blofeld emulator discovery {name}'" in line,
                    f"Actual application port is absent: {line}")
        source = f"{source_client}:{source_port}"
        target = f"{dest_client}:{dest_port}"
        aconnect(source, target)
        connected = aconnect("-l")
        destination.with_suffix(".connected.txt").write_text(connected)
        child.stdin.write("exchange\n")
        child.stdin.flush()
        require(select.select([child.stdout], [], [], 5)[0],
                "Virtual routing exchange did not finish")
        require(child.stdout.readline() == "exchange passed\n",
                "The owned ports did not exchange literal note bytes")
        aconnect("-d", source, target)
        out, err = child.communicate("retire\n", timeout=5)
        require(child.returncode == 0 and out == "retired\n" and not err,
                f"Retirement failed: {child.returncode}, {out!r}, {err!r}")
        after = aconnect("-l")
        for client, name in ((source_client, "source"), (dest_client, "destination")):
            require(not any(line.startswith(f"client {client}:")
                            and f"'Blofeld emulator discovery {name}'" in line
                            for line in after.splitlines()),
                    "An owned discovery endpoint survived retirement")
        require(hashlib.sha256(binary.read_bytes()).hexdigest() == digest,
                "The tested binary changed during discovery")
        destination.with_suffix(".before.txt").write_text(before)
        destination.with_suffix(".after.txt").write_text(after)
        report.update(status="passed", normal_tool="aconnect -l / connect / disconnect",
                      scope="Owned virtual ALSA discovery, literal note routing and retirement; "
                            "not USB enumeration or instrument boot")
    except BaseException as error:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=5)
        report.update(status="failed", error=repr(error))
        raise
    finally:
        destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))
    return 0


if __name__ == "__main__":
    sys.exit(main())
