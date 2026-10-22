# SPDX-License-Identifier: GPL-3.0-only
"""Check native DSP PCM through a private PulseAudio server and its real monitor."""

import json
import os
from pathlib import Path
import select
import shutil
import struct
import subprocess
import sys
import time

EXPECTED_EDGES = bytes.fromhex("0000008000ffff7f0001000000ffffff"
                              "000000000000000000ffff7f00000080")


def wait_for(condition, process, seconds=5):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Process exited unexpectedly: {process.args}")
        if condition():
            return
        time.sleep(0.02)
    raise TimeoutError("Private audio server/monitor did not become ready")


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)


def verify_capture(result, directory, board_result):
    samples = (directory / "capture.raw").read_bytes()
    # Independent signed S32LE bytes for the two actual native 24-bit words.
    pair = bytes.fromhex("009fc3a500563412")
    if struct.unpack("<ii", pair) != (-1513906432, 305419776):
        raise AssertionError("Independent sample expectation is inconsistent")
    counts = [int(line.split()[2]) for line in result.stdout.splitlines()
              if line.startswith("PLAYED ")]
    if counts != [4096, 4096] or len(samples) % 8:
        raise AssertionError("Missing both native ESAI routes or complete monitor frames")
    actual = sum(samples[index:index + 8] == pair for index in range(0, len(samples), 8))
    if actual != sum(counts) + 2:
        raise AssertionError(f"Monitor retained {actual} of {sum(counts) + 2} exact native pairs")
    board_rows = [tuple(map(int, line.split()[1:])) for line in board_result.stdout.splitlines()
                  if line.startswith("BOARD ")]
    repeated = bytes.fromhex("0056341200563412")
    if board_rows != [(0, 64), (1, 64)] or sum(
            samples[index:index + 8] == repeated for index in range(0, len(samples), 8)) != 126:
        raise AssertionError("Both actual board runs must retain exactly their 64 original frames")
    if board_result.stdout.splitlines().count("OVERFLOW 256") != 1:
        raise AssertionError("Missing actual native producer overflow and retained 256-frame prefix")
    edges = EXPECTED_EDGES
    if "EDGES 4\n" not in result.stdout or samples.find(edges) < 0:
        raise AssertionError("Missing exact min/max, unit, zero and channel-order boundary frames")
    allowed = {pair, repeated, bytes(8), *(edges[index:index + 8] for index in range(0, len(edges), 8))}
    if any(samples[index:index + 8] not in allowed for index in range(0, len(samples), 8)):
        raise AssertionError("Monitor contains altered, swapped or unexpected PCM")
    for edge in allowed - {pair, repeated, bytes(8)}:
        if sum(samples[index:index + 8] == edge for index in range(0, len(samples), 8)) != 1:
            raise AssertionError("A nonzero precision boundary frame is missing or duplicated")
    return {"passed": True, "native_routes": 2, "exact_frames": sum(counts),
            "coupled_board_frames": 128, "restore_retirements": 2,
            "native_overflow_stopped": True,
            "boundary_frames": 4, "monitor_bytes": len(samples), "sample_rate": 48000,
            "format": "signed 32-bit stereo; original 24-bit precision",
            "fuzz_execution": False, "physical_audio": False,
            "original_firmware_audio": False}


def verify_disconnect(binary, server, environment, daemon, directory):
    client = subprocess.Popen([str(binary), server, server, "--disconnect"], env=environment,
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True)
    try:
        prefix = bytearray()
        deadline = time.monotonic() + 5
        while b"CONNECTED\n" not in prefix and time.monotonic() < deadline:
            ready, _, _ = select.select([client.stdout], [], [], 0.1)
            if ready:
                data = os.read(client.stdout.fileno(), 4096)
                if not data:
                    break
                prefix.extend(data)
        if b"CONNECTED\n" not in prefix:
            raise AssertionError(f"Failure client did not establish playback: {prefix!r}")
        stop(daemon)
        output, errors = client.communicate("x", timeout=5)
        (directory / "disconnect.log").write_text(prefix.decode() + output + errors)
        if client.returncode or output != "DISCONNECTED\n" or errors:
            raise AssertionError(f"Disconnected transport did not reject safely: {output}{errors}")
    finally:
        stop(client)


def private_bus(directory):
    with (directory / "bus.log").open("wb") as log:
        process = subprocess.Popen(["dbus-daemon", "--session", "--nofork", "--print-address=1"],
                                   stdout=subprocess.PIPE, stderr=log, text=True)
    try:
        ready, _, _ = select.select([process.stdout], [], [], 5)
        if not ready:
            raise TimeoutError("Private D-Bus did not publish its address")
        address = process.stdout.readline().strip()
        if not address.startswith("unix:") or process.poll() is not None:
            raise RuntimeError("Invalid private D-Bus address")
        return process, address
    except BaseException:
        stop(process)
        raise


def run(binary, directory, board_binary):
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    directory.chmod(0o700)
    (directory / "verification.json").unlink(missing_ok=True)
    for command in ("pulseaudio", "pactl", "parec", "dbus-daemon"):
        if shutil.which(command) is None:
            print(f"SKIP host audio: {command} absent")
            return 77
    runtime = directory / "runtime"
    runtime.mkdir(exist_ok=True, mode=0o700)
    socket = runtime / "native"
    server = f"unix:{socket}"
    # Separate socket/state/runtime and no default script: no hardware modules,
    # session routing, user's server, or physical audio outputs are touched.
    environment = dict(os.environ, PULSE_RUNTIME_PATH=str(runtime),
                       PULSE_STATE_PATH=str(directory / "state"), PULSE_SERVER=server)
    environment.pop("PULSE_SINK", None)
    configuration = directory / "server.pa"
    configuration.write_text(
        ".fail\n"
        f"load-module module-native-protocol-unix socket={socket} auth-anonymous=1\n"
        "load-module module-null-sink sink_name=blofeld_test format=s32le rate=48000 "
        "channels=2 channel_map=front-left,front-right norewinds=true\n"
        "set-default-sink blofeld_test\n")
    command = ["pulseaudio", "--daemonize=no", "--exit-idle-time=-1", "--disallow-exit",
               "--use-pid-file=no", "--disable-shm", "--log-target=stderr", "-n",
               f"--file={configuration}"]
    daemon = monitor = bus = None
    control = ["pactl", "--server", server]

    def ready():
        return subprocess.run(control + ["info"], env=environment,
                              capture_output=True, timeout=2, check=False).returncode == 0

    def recording():
        result = subprocess.run(control + ["list", "source-outputs", "short"],
                                env=environment, capture_output=True, timeout=2, check=True)
        return bool(result.stdout.strip())

    try:
        bus, address = private_bus(directory)
        environment["DBUS_SESSION_BUS_ADDRESS"] = address
        with (directory / "server.log").open("wb") as log:
            daemon = subprocess.Popen(command, env=environment, stdout=log, stderr=log)
            wait_for(ready, daemon)
            with (directory / "capture.raw").open("wb") as captured, \
                    (directory / "monitor.log").open("wb") as errors:
                monitor = subprocess.Popen(
                    ["parec", "--server", server, "--device=blofeld_test.monitor", "--raw",
                     "--format=s32le", "--rate=48000", "--channels=2", "--latency-msec=5"],
                    env=environment, stdout=captured, stderr=errors)
                wait_for(recording, monitor)
                result = subprocess.run([str(binary), server, f"unix:{runtime / 'absent'}"],
                                        env=environment, capture_output=True, text=True,
                                        timeout=30, check=False)
                (directory / "client.log").write_text(result.stdout + result.stderr)
                if result.returncode or any(marker in result.stderr for marker in (
                        "AddressSanitizer", "LeakSanitizer", "runtime error:")):
                    raise RuntimeError(f"Host audio client failed: {result.stdout}{result.stderr}")
                board_result = subprocess.run([str(board_binary), "--host-audio", server],
                                              env=environment, capture_output=True, text=True,
                                              timeout=60, check=False)
                (directory / "board.log").write_text(board_result.stdout + board_result.stderr)
                if board_result.returncode or any(marker in board_result.stderr for marker in (
                        "AddressSanitizer", "LeakSanitizer", "runtime error:")):
                    raise RuntimeError(f"Coupled board audio failed: {board_result.stdout}{board_result.stderr}")
                wait_for(lambda: EXPECTED_EDGES in (directory / "capture.raw").read_bytes(),
                         monitor)
                wait_for(lambda: (directory / "capture.raw").read_bytes().count(
                    bytes.fromhex("0056341200563412")) >= 126, monitor)
                stop(monitor)
            verification = verify_capture(result, directory, board_result)
            verify_disconnect(binary, server, environment, daemon, directory)
            verification["real_disconnect_rejected"] = True
            (directory / "verification.json").write_text(json.dumps(verification, indent=2) + "\n")
    finally:
        stop(monitor)
        stop(daemon)
        stop(bus)
    print("Verified both native DSP ESAI routes through the actual private audio server; "
          "8192 exact transport frames, 128 coupled-board frames, restore retirement; no physical audio.")
    return 0


if __name__ == "__main__":
    sys.exit(run(*(Path(argument).resolve() for argument in sys.argv[1:4])))
