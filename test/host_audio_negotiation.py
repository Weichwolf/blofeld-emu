#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Observe real private-service grants, sink moves and suspend/resume."""

import json
from pathlib import Path
import select
import shutil
import struct
import subprocess
import sys
import tempfile
import time

from host_audio import require, service, terminate, until


def response(child):
    require(select.select([child.stdout], [], [], 8)[0], "Missing actual stream readback")
    line = child.stdout.readline()
    if not line:
        raise ValueError(f"Audio client exited with {child.poll()}: {child.stderr.read()}")
    return json.loads(line)


def pactl(server, env, *arguments):
    return subprocess.check_output(["pactl", "--server", server, *arguments],
                                   env=env, text=True, timeout=3)


def actual_input(server, env, report):
    inputs = json.loads(pactl(server, env, "--format=json", "list", "sink-inputs"))
    item = next(item for item in inputs if item["index"] == report["index"])
    native = "s32le" if sys.byteorder == "little" else "s32be"
    require(item["sample_specification"] == f"{native} 2ch 48000Hz"
            and item["channel_map"] == "front-left,front-right",
            f"The actual server stream disagrees with source PCM: {item}")
    require(item["sink"] == report["device"] and report["rate"] == 48000
            and report["channels"] == 2 and report["format"] == "s32ne",
            "Public snapshot disagrees with independent server identity/PCM readback")


def case(binary, directory, preference):
    directory.mkdir()
    extra = ("load-module module-null-sink sink_name=alternate format=s16le rate=44100 "
             "channels=2 channel_map=front-right,front-left norewinds=1\n")
    with service(directory, extra) as (daemon, server, env):
        sinks = json.loads(pactl(server, env, "--format=json", "list", "sinks"))
        primary = next(item for item in sinks if item["name"] == "fixture")
        alternate = next(item for item in sinks if item["name"] == "alternate")
        require(alternate["sample_specification"] == "s16le 2ch 44100Hz"
                and alternate["channel_map"] == "front-right,front-left",
                "The alternate sink must actually differ in rate, format and channel map")
        child = subprocess.Popen([str(binary), "negotiation", server], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, env=env)
        reports = []
        try:
            child.stdin.write(f"{preference}\n")
            child.stdin.flush()
            initial = response(child)
            actual_input(server, env, initial)
            require(initial["device"] == primary["index"] and initial["suspended"] is False,
                    "The private session must initially select its configured default sink")
            reports.append(initial)
            (directory / "snapshots.json").write_text(json.dumps(reports, indent=2) + "\n")
            for name, suspended in (("alternate", False), ("alternate", True),
                                    ("alternate", False), ("fixture", False)):
                target = alternate if name == "alternate" else primary
                if reports[-1]["device"] != target["index"]:
                    pactl(server, env, "move-sink-input", str(initial["index"]), name)
                pactl(server, env, "suspend-sink", name, "1" if suspended else "0")
                child.stdin.write(f"observe {target['index']} {int(suspended)}\n")
                child.stdin.flush()
                report = response(child)
                require(report["suspended"] is suspended and report["requested_frames"] == preference,
                        "The snapshot must reflect actual suspension and retain the preference")
                actual_input(server, env, report)
                reports.append(report)
                (directory / "snapshots.json").write_text(json.dumps(reports, indent=2) + "\n")
            capture = directory / "monitor.pcm"
            with capture.open("wb") as data, (directory / "monitor.log").open("w") as log:
                recorder = subprocess.Popen(["parec", "--raw", "--format=s32le", "--rate=48000",
                                             "--channels=2", "--device=fixture.monitor",
                                             "--latency-msec=5"], env=env, stdout=data, stderr=log)
                try:
                    until(lambda: bool(pactl(server, env, "list", "short", "source-outputs").strip()),
                          daemon, "the private monitor")
                    out, err = child.communicate("complete\n", timeout=12)
                    require(child.returncode == 0 and out == "frames=256\n" and not err,
                            f"Post-move playback failed: {out!r}, {err!r}")
                    time.sleep(0.1)
                finally:
                    terminate(recorder)
            expected = (struct.pack("<ii", -5913697 * 256, 1193046 * 256)
                        + struct.pack("<ii", 1193046 * 256, 1193046 * 256) * 255)
            recorded = capture.read_bytes()
            position = recorded.find(expected)
            require(position >= 0 and position % 8 == 0
                    and recorded.find(expected, position + 1) < 0,
                    "Host-only route changes must retain the later exact native PCM prefix")
            require(not (directory / "monitor.log").read_text(), "Private monitor reported a fault")
            return {"requested_frames": preference, "snapshots": reports,
                    "exact_native_frames_after_controls": 256}
        finally:
            terminate(child)


def main():
    for program in ("pulseaudio", "parec", "pactl"):
        if shutil.which(program) is None:
            print(f"SKIP: private audio negotiation unavailable: missing {program}")
            return 77
    artifacts = Path(sys.argv[2]).resolve()
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="blofeld-audio-grants-") as temporary:
        directory = Path(temporary)
        try:
            results = [case(Path(sys.argv[1]).resolve(), directory / str(size), size)
                       for size in (256, 513, 4093)]
            require(all(0 < item["minimum_request"] <= item["target"]
                        for case in results for item in case["snapshots"]),
                    "The service must resolve automatic request granularity to a usable grant")
            result = {"cases": results, "fuzz_execution": False, "physical_audio": False,
                      "original_firmware_audio_qualified": False}
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
        finally:
            for source in directory.rglob("*"):
                if source.is_file() and source.suffix in (".log", ".json", ".pcm", ".pa"):
                    target = artifacts / directory.name / source.relative_to(directory)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
