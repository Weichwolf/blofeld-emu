#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Send valid MIDI with Debian tools only to the test's owned virtual input."""

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
    for tool in ("aseqsend", "aconnect"):
        if shutil.which(tool) is None:
            print(f"SKIP: required standard MIDI tool absent: {tool}")
            return 77
    binary = Path(sys.argv[1]).resolve()
    report = Path(sys.argv[2])
    report.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    state = {"status": "starting", "binary": str(binary), "binary_sha256": digest,
             "fuzz_execution": False, "physical_traffic": False, "whole_goal_complete": False}
    environment = os.environ.copy()
    environment["LC_ALL"] = "C"
    child = subprocess.Popen([str(binary), "--tool-input"], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, env=environment)
    state["pid"] = child.pid
    try:
        require(select.select([child.stdout], [], [], 10)[0], "No actual input address")
        first = child.stdout.readline()
        if first.startswith("SKIP:"):
            child.communicate(timeout=5)
            require(child.returncode == 77, "Availability refusal must be an explicit skip")
            state.update(status="skipped", reason=first.strip())
            return 77
        match = re.fullmatch(r"MIDI input=(\d+):(\d+)\n", first)
        require(match, f"Unexpected input address report: {first!r}")
        target = f"{match[1]}:{match[2]}"
        state["target"] = target
        before = subprocess.check_output(["aconnect", "-l"], text=True,
                                         env=environment, timeout=5)
        client = next(line for line in before.splitlines()
                      if line.startswith(f"client {match[1]}:"))
        name = "'Blofeld emulator standard tool input'"
        require(name in client, f"Owned input is absent from ordinary discovery: {client}")
        command = ["aseqsend", "-u", "0", "-p", target,
                   "90 3e 63 b0 4a 6f c0 0c f8 f0 7d 01 02 f7"]
        sent = subprocess.run(command, text=True, capture_output=True,
                              env=environment, timeout=10, check=False)
        state["tool"] = {"command": command, "exit_code": sent.returncode,
                         "stdout": sent.stdout, "stderr": sent.stderr}
        require(sent.returncode == 0, f"aseqsend failed: {sent.stderr}")
        out, err = child.communicate("sent\n", timeout=120)
        state.update(exit_code=child.returncode, stdout=out, stderr=err)
        require(child.returncode == 0 and not err and
                out == "Standard MIDI tool/native SCI bytes passed\n",
                f"Native reception failed: {child.returncode}, {out!r}, {err!r}")
        after = subprocess.check_output(["aconnect", "-l"], text=True,
                                        env=environment, timeout=5)
        require(not any(line.startswith(f"client {match[1]}:") and name in line
                        for line in after.splitlines()), "Owned endpoint survived retirement")
        require(hashlib.sha256(binary.read_bytes()).hexdigest() == digest,
                "The tested binary changed during execution")
        state["status"] = "passed"
    except BaseException as error:
        state.update(status="failed", error=repr(error))
        if child.poll() is None:
            child.terminate()
        out, err = child.communicate(timeout=10)
        state.update(native_exit_code=child.returncode, stdout=out, stderr=err)
        raise
    finally:
        report.write_text(json.dumps(state, indent=2) + "\n")
    print("Standard aseqsend -> owned virtual input -> timed wire -> native SCI reads passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
