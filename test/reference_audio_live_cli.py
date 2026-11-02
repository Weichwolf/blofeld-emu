# SPDX-License-Identifier: GPL-3.0-only
"""Ordinary CLI native ESAI -> worker -> private Pulse monitor, without speakers."""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import time
import wave

from audio_runtime import board_inputs
from host_audio import require, service, terminate, until
from reference_midi_session_cli import Lines

FRAMES = 64
EXPECTED = struct.pack("<ii", -5913697 * 256, 1193046 * 256)
EXPECTED += struct.pack("<ii", 1193046 * 256, 1193046 * 256) * (FRAMES - 1)
ZIP_SHA = "095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72"


def monitor(binary, generator, firmware, directory, index, buffer):
    directory.mkdir(parents=True)
    inputs = board_inputs(directory / "inputs", generator, index, FRAMES)
    config, rom, flash = inputs
    before = {p: p.read_bytes() for p in (config, rom, flash, firmware, binary)}
    with service(directory) as (daemon, server, env):
        capture = directory / "monitor.pcm"
        with capture.open("wb") as data, (directory / "monitor.log").open("w") as log:
            recorder = subprocess.Popen(["parec", "--raw", "--format=s32le", "--rate=48000",
                                         "--channels=2", "--device=fixture.monitor",
                                         "--latency-msec=5"], env=env, stdout=data, stderr=log)
            child = None
            try:
                def recording():
                    result = subprocess.check_output(["pactl", "--server", server, "list", "short",
                                                       "source-outputs"], env=env, text=True)
                    return bool(result.strip())
                until(recording, daemon, "the owned CLI monitor")
                command = [str(binary), "--firmware", str(firmware), "--reference-board", str(config),
                           "--dsp-rom", str(rom), "--flash-in", str(flash), "--instructions", "100000",
                           "--audio-live", "--audio-rate", "48000", "--audio-server", server,
                           "--audio-buffer-frames", str(buffer), "--audio-out", str(directory / "out.wav"),
                           "--audio-max-frames", "128"]
                child = subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                output = Lines(child.stdout)
                ready = output.until(lambda line: line.startswith("AUDIO initial"))
                match = re.fullmatch(r"AUDIO initial format=s32-native rate=48000 channels=2 "
                                     r"maximum-frames=(\d+) target-frames=(\d+) prebuffer-frames=(\d+) "
                                     r"minimum-request-frames=(\d+) stream=(\d+)\n", ready)
                require(match and all(int(match[i]) > 0 for i in (1, 2, 4)),
                        "The CLI must publish actual positive negotiated stream dimensions")
                actual = json.loads(subprocess.check_output(
                    ["pactl", "--server", server, "--format=json", "list", "sink-inputs"],
                    env=env, text=True))
                observed = next(item for item in actual if item["index"] == int(match[5]))
                native = "s32le" if sys.byteorder == "little" else "s32be"
                require(observed["sample_specification"] == f"{native} 2ch 48000Hz" and
                        observed["channel_map"] == "front-left,front-right",
                        "Independent service introspection must agree with the source client PCM")
                (directory / "initial-service.json").write_text(json.dumps(observed, indent=2) + "\n")
                tail, error = child.communicate(timeout=120)
                full = bytes(output.seen) + tail
                (directory / "stdout.txt").write_bytes(full)
                (directory / "stderr.txt").write_bytes(error)
                require(child.returncode == 0 and not error, full.decode() + error.decode())
                require(b"AUDIO closed=1 accepted=64 written=64" in full and b"completed=1" in full,
                        "CLI completion must prove every accepted native frame was written")
                time.sleep(0.1)
            finally:
                if child is not None and child.poll() is None:
                    tail, error = child.communicate(timeout=120)
                    (directory / "failed-stdout.txt").write_bytes(bytes(output.seen) + tail)
                    (directory / "failed-stderr.txt").write_bytes(error)
                terminate(recorder)
        payload = capture.read_bytes()
        at = payload.find(EXPECTED)
        require(at >= 0 and at % 8 == 0 and payload.find(EXPECTED, at + 1) < 0 and
                payload.count(EXPECTED[:8]) == 1,
                "The private monitor must contain all64 native frames exactly once")
        require(not (directory / "monitor.log").read_text(), "Private monitor reported a failure")
        with wave.open(str(directory / "out.wav"), "rb") as recorded:
            require((recorded.getnchannels(), recorded.getsampwidth(), recorded.getframerate(),
                     recorded.getnframes()) == (2, 4, 48000, FRAMES) and
                    recorded.readframes(FRAMES + 1) == EXPECTED,
                    "Simultaneous WAV recording and live output must retain identical native samples")
    require(all(p.read_bytes() == value for p, value in before.items()), "An input was changed")
    print(f"Live CLI ESAI{index}:64 exact monitor/WAV frames, actual host grants, buffer preference{buffer}")


def refusals(binary, firmware, generator, directory):
    directory.mkdir(parents=True)
    base = [str(binary), "--firmware", "absent.zip"]
    shared = ["--reference-board", "absent.bfboard", "--dsp-rom", "absent.rom"]
    cases = [(["--audio-live", "--audio-rate", "48000"], "Audio requires"),
             (shared + ["--audio-live"], "Audio requires"),
             (shared + ["--audio-live", "--audio-rate", "48000", "--inspect"], "conflicts"),
             (shared + ["--audio-server", "unused"], "require --audio-live"),
             (shared + ["--audio-live", "--audio-rate", "48000", "--audio-buffer-frames", "255"],
              "256..65536"),
             (shared + ["--audio-live", "--audio-rate", "48000", "--audio-buffer-frames", "65537"],
              "Invalid audio"),
             (shared + ["--audio-live", "--audio-rate", "48000", "--audio-max-frames", "1"],
              "Audio requires")]
    for index, (arguments, expected) in enumerate(cases):
        result = subprocess.run(base + arguments, capture_output=True, text=True, timeout=15)
        require(result.returncode == 1 and expected in result.stderr, result.stdout + result.stderr)
        (directory / f"option-{index}.txt").write_text(result.stdout + result.stderr)
    config, rom, flash = board_inputs(directory / "inputs", generator, 1)
    initial = config.read_text()
    for name, text, expected in (("no-route", re.sub(r"pcm-receive[^\n]+\n", "", initial), "pcm-receive"),
                                 ("replay", initial + "checkpoint 10 2\n", "rejects automatic")):
        config.write_text(text)
        trace = directory / (name + ".trace")
        result = subprocess.run([str(binary), "--firmware", str(firmware), "--reference-board", str(config),
                                 "--dsp-rom", str(rom), "--audio-live", "--audio-rate", "48000",
                                 "--trace", str(trace)], capture_output=True, text=True, timeout=15)
        require(result.returncode == 1 and expected in result.stderr and not trace.exists(),
                result.stdout + result.stderr)
    config.write_text(initial)
    missing = "unix:" + str(directory / "missing-service")
    result = subprocess.run([str(binary), "--firmware", str(firmware), "--reference-board", str(config),
                             "--dsp-rom", str(rom), "--flash-in", str(flash), "--audio-live",
                             "--audio-rate", "48000", "--audio-server", missing],
                            capture_output=True, text=True, timeout=15)
    require(result.returncode == 1 and "Host audio" in result.stderr and "AUDIO closed=1" not in result.stdout,
            "An unavailable service cannot fabricate successful delivery: " + result.stdout + result.stderr)
    (directory / "missing-service.txt").write_text(result.stdout + result.stderr)


def main():
    binary, generator, firmware, directory = (p.resolve() for p in map(Path, sys.argv[1:]))
    for tool in ("pulseaudio", "parec", "pactl"):
        if shutil.which(tool) is None:
            print("SKIP live CLI audio: missing private service tool", tool)
            return 77
    if not firmware.is_file():
        print("SKIP live CLI audio: private original firmware archive absent")
        return 77
    require(hashlib.sha256(firmware.read_bytes()).hexdigest() == ZIP_SHA, "Original archive identity differs")
    refusals(binary, firmware, generator, directory / "refusals")
    for index, buffer in ((0, 513), (1, 4093)):
        monitor(binary, generator, firmware, directory / f"esai{index}", index, buffer)
    return 0


if __name__ == "__main__":
    sys.exit(main())
