#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Directed live service tests against an isolated, hardware-free Pulse server."""

import contextlib
import json
import os
from pathlib import Path
import select
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time


def require(condition, message):
    if not condition:
        raise ValueError(message)


def terminate(child):
    if child is not None and child.poll() is None:
        child.send_signal(signal.SIGCONT)
        child.terminate()
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=3)


def until(predicate, child, description):
    deadline = time.monotonic() + 5
    while not predicate():
        require(child.poll() is None, f"Private service exited while waiting for {description}")
        require(time.monotonic() < deadline, f"Timed out waiting for {description}")
        time.sleep(0.005)


@contextlib.contextmanager
def service(directory, extra_config=""):
    runtime = directory / "runtime"
    state = directory / "state"
    runtime.mkdir(mode=0o700)
    state.mkdir(mode=0o700)
    endpoint = directory / "native"
    server = "unix:" + str(endpoint)
    env = os.environ.copy()
    env.update(PULSE_RUNTIME_PATH=str(runtime), PULSE_STATE_PATH=str(state), PULSE_SERVER=server,
               DBUS_SESSION_BUS_ADDRESS="unix:path=" + str(directory / "no-session-bus"))
    config = directory / "server.pa"
    config.write_text(
        ".fail\n"
        f"load-module module-native-protocol-unix socket={endpoint} auth-anonymous=1\n"
        "load-module module-null-sink sink_name=fixture format=s32le rate=48000 channels=2 "
        "channel_map=front-left,front-right norewinds=1\n"
        "set-default-sink fixture\n" + extra_config)
    with (directory / "server.log").open("w") as log:
        child = subprocess.Popen(["pulseaudio", "--daemonize=no", "--use-pid-file=no",
                                  "--high-priority=no", "--realtime=no", "--exit-idle-time=-1",
                                  "--log-target=stderr", "-n", "-F", str(config)],
                                 env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            until(endpoint.exists, child, "the private socket")
            yield child, server, env
        except BaseException:
            log.flush()
            print((directory / "server.log").read_text(), file=sys.stderr)
            raise
        finally:
            terminate(child)


def invoke(binary, mode, server, env):
    result = subprocess.run([str(binary), mode, server], env=env, text=True,
                            capture_output=True, timeout=20)
    require(result.returncode == 0, f"{mode}: {result.stdout}\n{result.stderr}")
    require(not result.stderr.strip(), f"{mode}: unexpected diagnostics: {result.stderr}")
    return result.stdout.strip()


def monitor_case(binary, mode, directory):
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
                until(recording, daemon, "the private monitor recording")
                output = invoke(binary, mode, server, env)
                # Let the monitor's ordinary userspace buffer deliver its final chunk.
                time.sleep(0.1)
            finally:
                terminate(recorder)
        recorded = capture.read_bytes()
        first = struct.pack("<ii", -5913697 * 256, 1193046 * 256)
        rest = struct.pack("<ii", 1193046 * 256, 1193046 * 256)
        expected = first + rest * 255
        offsets = [position for position in range(0, len(recorded) - len(expected) + 1, 8)
                   if recorded[position:position + len(expected)] == expected]
        if not offsets:
            pairs = list(struct.iter_unpack("<ii", recorded[:len(recorded) // 8 * 8]))
            nonzero = [(index, pair) for index, pair in enumerate(pairs) if pair != (0, 0)]
            print(f"{mode}: output={output}, first nonzero samples={nonzero[:12]}, "
                  f"nonzero count={len(nonzero)}", file=sys.stderr)
        require(len(offsets) == 1, f"{mode}: missing or duplicated independent native PCM payload; "
                                  f"capture={len(recorded)} bytes, exact matches={offsets}")
        require((directory / "monitor.log").read_text() == "", "Private monitor reported a fault")
        return {"mode": mode, "matched_frames": 256, "offset_bytes": offsets[0], "status": output}


def failure_case(binary, mode, directory):
    directory.mkdir()
    with service(directory) as (daemon, server, env):
        child = subprocess.Popen([str(binary), mode, server], env=env,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        try:
            prefix = b""
            deadline = time.monotonic() + 15
            while b"READY" not in prefix.splitlines():
                remaining = deadline - time.monotonic()
                require(remaining > 0 and len(prefix) < 16384,
                        f"{mode}: missing client synchronization: {prefix!r}")
                readable, _, _ = select.select([child.stdout], [], [], remaining)
                require(readable, f"{mode}: client did not become ready")
                chunk = os.read(child.stdout.fileno(), 4096)
                require(chunk, f"{mode}: client exited before synchronization: {prefix!r}")
                prefix += chunk
            if mode == "death":
                terminate(daemon)
            else:
                daemon.send_signal(signal.SIGSTOP)
            output, error = child.communicate(b"continue\n", timeout=6)
            require(child.returncode == 0 and not error, f"{mode}: {output}\n{error}")
        finally:
            terminate(child)
        return {"mode": mode, "visible_bounded_failure": True,
                "client_output": prefix.decode().strip()}


def main():
    require(len(sys.argv) == 2, "Expected host audio test binary")
    for program in ("pulseaudio", "parec", "pactl"):
        if shutil.which(program) is None:
            print(f"Host audio service integration unavailable: missing {program}")
            return 77
    binary = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="blofeld-audio-") as temporary:
        directory = Path(temporary)
        results = [monitor_case(binary, mode, directory / mode)
                   for mode in ("primary", "secondary", "default")]
        results += [failure_case(binary, mode, directory / mode)
                    for mode in ("death", "stopped")]
        missing = "unix:" + str(directory / "missing")
        invoke(binary, "missing", missing, os.environ.copy())
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
            endpoint = directory / "stalled"
            listener.bind(str(endpoint))
            listener.listen(1)
            invoke(binary, "timeout", "unix:" + str(endpoint), os.environ.copy())
        print(json.dumps({"cases": results, "missing_and_stalled_server": "passed",
                          "fuzz_execution": False, "physical_audio": False,
                          "original_firmware_audio_qualified": False}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
