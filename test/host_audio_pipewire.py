#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Real alternate-service grants, corked routes, exact PCM and worker failures."""

import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time

import audio_runtime
import host_audio
from host_audio import require, terminate, until
from host_audio_negotiation import actual_input, pactl, response
from pipewire_service import reference_tools, service


def route_case(binary, directory, preference, minimum_request=0):
    directory.mkdir()
    with service(directory, alternate=True, minimum_request=minimum_request) as (daemon, server, env):
        sinks = json.loads(pactl(server, env, "--format=json", "list", "sinks"))
        primary = next(item for item in sinks if item["name"] == "fixture")
        alternate = next(item for item in sinks if item["name"] == "alternate")
        require(alternate["sample_specification"] == "s16le 2ch 44100Hz"
                and alternate["channel_map"] == "front-right,front-left",
                "The alternate sink must actually differ in format, rate and layout")
        child = subprocess.Popen([str(binary), "negotiation", server], env=env,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, text=True)
        reports = []
        try:
            child.stdin.write(f"{preference}\n")
            child.stdin.flush()
            initial = response(child)
            actual_input(server, env, initial)
            require(initial["device"] == primary["index"] and initial["suspended"] is False,
                    "The native client must start on its selected private sink")
            reports.append(initial)
            (directory / "snapshots.json").write_text(json.dumps(reports, indent=2) + "\n")
            if minimum_request:
                require(initial["minimum_request"] >= minimum_request,
                        "The reference service must actually apply its requested floor")
            for target in (alternate, primary):
                pactl(server, env, "move-sink-input", str(initial["index"]), target["name"])
                child.stdin.write(f"observe {target['index']} 0\n")
                child.stdin.flush()
                report = response(child)
                actual_input(server, env, report)
                require(report["requested_frames"] == preference,
                        "Routing must retain the source buffer preference")
                reports.append(report)
                (directory / "snapshots.json").write_text(json.dumps(reports, indent=2) + "\n")
            capture = directory / "monitor.pcm"
            with capture.open("wb") as data, (directory / "monitor.log").open("w") as log:
                recorder = subprocess.Popen(["parec", "--raw", "--format=s32le", "--rate=48000",
                                             "--channels=2", "--device=fixture.monitor",
                                             "--latency-msec=5"], env=env, stdout=data, stderr=log)
                try:
                    until(lambda: bool(pactl(server, env, "list", "short", "source-outputs").strip()),
                          daemon, "the isolated PipeWire monitor")
                    out, err = child.communicate("complete\n", timeout=12)
                    require(child.returncode == 0 and out == "frames=256\n" and not err,
                            f"Native playback failed: {out!r}, {err!r}")
                    time.sleep(0.1)
                finally:
                    terminate(recorder)
            expected = (struct.pack("<ii", -5913697 * 256, 1193046 * 256)
                        + struct.pack("<ii", 1193046 * 256, 1193046 * 256) * 255)
            recorded = capture.read_bytes()
            position = recorded.find(expected)
            require(position >= 0 and position % 8 == 0
                    and recorded.find(expected, position + 1) < 0,
                    "The independent monitor must contain the exact native PCM prefix once")
            require(not (directory / "monitor.log").read_text(), "Monitor reported a failure")
            return {"requested_frames": preference, "snapshots": reports,
                    "server_minimum_request_frames": minimum_request,
                    "exact_native_frames_after_moves": 256, "monitor_bytes": len(recorded),
                    "offset_bytes": position, "suspend_notification_qualified": False}
        finally:
            terminate(child)


def worker_service(directory):
    return service(directory, minimum_request=1024)


def qualify(binary, worker, directory):
    routes = [route_case(binary, directory / str(size), size) for size in (256, 513, 4093)]
    routes.append(route_case(binary, directory / "request-floor", 4093, 1024))
    audio_runtime.service = worker_service
    paced = audio_runtime.monitor_case(worker, "paced", directory / "paced")
    host_audio.service = service
    failures = [host_audio.failure_case(binary, mode, directory / mode)
                for mode in ("death", "stopped")]
    return {"routes": routes, "worker": paced, "service_failures": failures,
            "fuzz_execution": False, "physical_audio": False,
            "original_firmware_audio_qualified": False, "suspend_notification_qualified": False}


def main():
    require(len(sys.argv) == 4, "Expected host audio binary, worker binary and artifacts")
    programs, _ = reference_tools()
    if not all(programs) or any(shutil.which(name) is None for name in ("pactl", "parec")):
        print("SKIP: PipeWire/WirePlumber and Pulse observation tools are required")
        return 77
    binary, worker, artifacts = (Path(value).resolve() for value in sys.argv[1:])
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="blofeld-pipewire-") as temporary:
        directory = Path(temporary)
        try:
            result = qualify(binary, worker, directory)
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
        finally:
            for source in directory.rglob("*"):
                if source.is_file() and source.suffix in (".log", ".pcm", ".json", ".conf", ".txt"):
                    target = artifacts / directory.name / source.relative_to(directory)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
