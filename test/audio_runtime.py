#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Actual worker/board transport and failures against private audio services."""

import json
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

from host_audio import failure_case, require, service, terminate, until
from test_reference_cli import image


def invoke(binary, mode, server, env, extra=()):
    # Only native board supervision scales under TSan; worker I/O deadlines do not.
    result = subprocess.run([str(binary), mode, server, *map(str, extra)], env=env,
                            capture_output=True, text=True,
                            timeout=90 if mode.startswith("board") and
                            env.get("BLOFELD_TEST_SANITIZER") == "thread" else 30)
    require(result.returncode == 0 and not result.stderr,
            f"{mode}: {result.stdout}\n{result.stderr}")
    return result.stdout


def expected_pcm(mode):
    first = struct.pack("<ii", -5913697 * 256, 1193046 * 256)
    if mode.startswith("board"):
        return first + struct.pack("<ii", 1193046 * 256, 1193046 * 256)
    count = {"short": 1, "tail": 1025, "paced": 8192}[mode]
    return first + b"".join(struct.pack("<ii", ((index % 4093) - 2046) * 256,
                                       (2046 - (index % 4093)) * 256)
                            for index in range(1, count))


def monitor_case(binary, mode, directory, extra=()):
    directory.mkdir()
    with service(directory) as (daemon, server, env):
        capture = directory / "monitor.pcm"
        with capture.open("wb") as data, (directory / "monitor.log").open("w") as log:
            recorder = subprocess.Popen(["parec", "--raw", "--format=s32le", "--rate=48000",
                                         "--channels=2", "--device=fixture.monitor",
                                         "--latency-msec=5"], env=env, stdout=data, stderr=log)
            try:
                def recording():
                    result = subprocess.run(["pactl", "--server", server, "list", "short",
                                             "source-outputs"], env=env, text=True,
                                            capture_output=True, timeout=2, check=True)
                    return bool(result.stdout.strip())
                until(recording, daemon, "the runtime monitor")
                output = invoke(binary, mode, server, env, extra)
                time.sleep(0.1)
            finally:
                terminate(recorder)
        recorded = capture.read_bytes()
        expected = expected_pcm(mode)
        position = recorded.find(expected)
        require(position >= 0 and position % 8 == 0 and
                recorded.find(expected, position + 1) < 0 and recorded.count(expected[:8]) == 1,
                f"{mode}: missing, truncated, stale or duplicated PCM prefix; "
                f"captured={len(recorded)}, expected={len(expected)}, position={position}")
        require(not (directory / "monitor.log").read_text(), "Runtime monitor reported a fault")
        (directory / "client.log").write_text(output)
        return {"mode": mode, "frames": len(expected) // 8, "exact_monitor_match": True,
                "client_output": output.strip()}


def board_inputs(directory, generator, index):
    directory.mkdir()
    config, rom, nvm = (directory / name for name in ("board.bfboard", "own-rom.bin", "own.bfnvm"))
    subprocess.run([str(generator), f"esai{index}-refill", str(rom)], check=True,
                   capture_output=True, timeout=20)
    config.write_text(f"""blofeld-reference-board 1
clock 12000000 0 12000000 138240000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 115200 low
operations 400
initial drive B 16 02 00
initial drive E 03 03 00
dsp-input-order input-before-dsp 16
dsp-initial esai {index} unknown unknown low reference-reset-edges
dsp-clock esai {index} hckt 512000 5625 1032 reference-reset-edges
pcm-receive esai {index} tx 0 after 307200 rising 32 24 2 high
""")
    nvm.write_bytes(image(bytes.fromhex("cf2000 3e")))
    return config, rom, nvm


def run(binary, generator, directory):
    results = [monitor_case(binary, mode, directory / mode) for mode in ("short", "tail", "paced")]
    for mode in ("invalid", "empty", "retire", "cancel", "underrun",
                 "blocked-write", "blocked-drain", "closed-underrun"):
        folder = directory / mode
        folder.mkdir()
        with service(folder) as (_, server, env):
            if mode.startswith("blocked-"):
                subprocess.run(["pactl", "--server", server, "suspend-sink", "fixture", "1"],
                               check=True, env=env, capture_output=True, timeout=3)
            output = invoke(binary, mode, server, env)
            if mode.startswith("blocked-"):
                subprocess.run(["pactl", "--server", server, "info"], check=True,
                               env=env, capture_output=True, timeout=3)
            (folder / "client.log").write_text(output)
        results.append({"mode": mode, "passed": True})
    results += [failure_case(binary, mode, directory / mode) for mode in ("death", "stopped")]
    for index in (0, 1):
        inputs = board_inputs(directory / f"board-inputs-{index}", generator, index)
        results.append(monitor_case(binary, "board", directory / f"board-{index}", inputs))
    inputs = board_inputs(directory / "restore-inputs", generator, 1)
    results.append(monitor_case(binary, "board-restore", directory / "board-restore", inputs))
    for mode in ("board-unknown", "board-reject", "board-binding"):
        folder = directory / mode
        folder.mkdir()
        with service(folder) as (_, server, env):
            (folder / "client.log").write_text(invoke(binary, mode, server, env, inputs))
        results.append({"mode": mode, "passed": True})
    invoke(binary, "missing", "unix:" + str(directory / "missing"), os.environ.copy())
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        endpoint = directory / "stalled"
        listener.bind(str(endpoint))
        listener.listen(1)
        invoke(binary, "timeout", "unix:" + str(endpoint), os.environ.copy())
    return {"cases": results, "missing_and_unresponsive_service": "passed",
            "fuzz_execution": False, "physical_audio": False,
            "original_firmware_audio_qualified": False}


def main():
    require(len(sys.argv) == 4, "Expected runtime test, ROM generator and artifact directory")
    for tool in ("pulseaudio", "parec", "pactl"):
        if shutil.which(tool) is None:
            print(f"Audio runtime service integration unavailable: missing {tool}")
            return 77
    binary, generator, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:])
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="blofeld-audio-worker-") as temporary:
        directory = Path(temporary)
        try:
            result = run(binary, generator, directory)
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
        finally:
            # Keep offline generated PCM/configuration and logs, including failures;
            # sockets and daemon private state are never copied or reused.
            destination = artifacts / directory.name
            for file in directory.rglob("*"):
                if file.is_file() and file.suffix in (".log", ".pcm", ".bfboard", ".json"):
                    target = destination / file.relative_to(directory)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(file, target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
