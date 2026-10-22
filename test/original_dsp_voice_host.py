#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Match pin-decoded original synthesis to an isolated service's actual monitor."""

import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import time

from dsp_voice_model import signed, waveform_model
from dsp_voice_validation import new_totals, validate_stream
from host_audio import require, service, terminate, until
from original_dsp_voice import original_records
from original_dsp_voice_master import Output
from original_dsp_voice_midi import Midi, SOUND
from original_dsp_voice_pcm import Pcm

ROOT = Path(__file__).resolve().parents[1]


class Capture(Pcm):
    def __init__(self):
        self.payload = bytearray()
        self.blocks = 0

    def __call__(self, expected, text, block):
        super().__call__(expected, text, block)
        for index in range(16):
            self.payload.extend(struct.pack("<ii", signed(expected[2 * index]) * 256,
                                            signed(expected[2 * index + 1]) * 256))
        self.blocks += 1


def expected_payload(text, xdata, ydata, program, midi=False):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    identity = ('normal', '0', '0', '0', '0')
    require(len(cases) == 1 and tuple(cases[0][:5]) == identity,
            "Host handoff must select exactly one declared default voice stream")
    capture = Capture()
    output = Output(program, capture)
    totals = new_totals()
    controller = Midi() if midi else None
    validate_stream(cases[0][5], xdata, ydata, {False: waveform_model(ydata, False)},
                    identity, totals, after_block=output, before_block=controller,
                    sound_base=SOUND if midi else 0x1000)
    require(totals['blocks'] == 8 and capture.blocks == 8 and output.nonzero > 0 and
            len(capture.payload) == 128 * 8,
            "Every original voice/mix/master/DMA/serial block must reach its independent oracle")
    require(controller.calls == 19 if midi else output.nonzero == 256,
            "The complete declared controller or default stream must execute")
    return bytes(capture.payload)


def controller_controls(text, xdata, ydata, program):
    for label, index in (('MIDI_BYTES', -1), ('MIDI_RETURN', -3),
                         ('OUTPUT_BUS', 0), ('PCM_FRAME', -1)):
        def corrupt(match):
            words = match[1].split()
            words[index] = format(int(words[index], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        require(count == 1, 'Missing direct controller/audio negative control')
        try:
            expected_payload(changed, xdata, ydata, program, midi=True)
        except ValueError as error:
            require(('MIDI' if label.startswith('MIDI') else label) in str(error),
                    'Wrong direct controller/audio negative-control failure')
        else:
            raise ValueError('The direct controller/audio oracle accepted corrupted trace data')


def check_monitor(recorded, expected):
    position = recorded.find(expected)
    require(len(recorded) % 8 == 0 and position >= 0 and position % 8 == 0,
            "The monitor is missing a complete aligned original PCM prefix")
    require(not recorded[:position].strip(b'\0') and
            not recorded[position + len(expected):].strip(b'\0'),
            "The private monitor contains extra, stale, duplicated or altered source samples")


def run(binary, firmware, directory, xdata, ydata, program, midi):
    with service(directory) as (daemon, server, env):
        recorded = directory / "monitor.pcm"
        with recorded.open("wb") as samples, (directory / "monitor.log").open("w") as log:
            monitor = subprocess.Popen(["parec", "--raw", "--format=s32le", "--rate=48000",
                                        "--channels=2", "--device=fixture.monitor",
                                        "--latency-msec=5"], env=env, stdout=samples, stderr=log)
            try:
                def ready():
                    result = subprocess.run(["pactl", "--server", server, "list", "short",
                                             "source-outputs"], env=env, capture_output=True,
                                            text=True, timeout=2, check=True)
                    return bool(result.stdout.strip())

                until(ready, daemon, "the original voice monitor")
                trace = directory / "native.trace"
                command = [str(binary), str(firmware), str(trace), server]
                if midi:
                    command.append('midi')
                result = subprocess.run(command, env=env,
                                        capture_output=True, text=True,
                                        timeout=90 if env.get("BLOFELD_TEST_SANITIZER") == "thread" else 30)
                (directory / "native.log").write_text(result.stdout + result.stderr)
                require(result.returncode == 0 and not result.stderr and
                        "original-decoded-host-frames=128" in result.stdout,
                        f"Original native host handoff failed: {result.stdout}{result.stderr}")
                text = trace.read_text()
                expected = expected_payload(text, xdata, ydata, program, midi)
                if midi:
                    controller_controls(text, xdata, ydata, program)
                time.sleep(0.1)
            finally:
                terminate(monitor)
        require(not (directory / "monitor.log").read_text(), "The source monitor reported a fault")
        check_monitor(recorded.read_bytes(), expected)
        corrupted = bytearray(expected)
        corrupted[0] ^= 1
        try:
            check_monitor(bytes(corrupted), expected)
        except ValueError:
            pass
        else:
            raise ValueError("The monitor oracle accepted a corrupted source word")
        return {"passed": True, "original_blocks": 8, "decoded_frames": 128,
                "exact_independent_monitor_match": True, "monitor_negative_controls": 1,
                "original_controller_parser_calls": 19 if midi else 0,
                "controller_trace_negative_controls": 4 if midi else 0,
                "declared_playback_rate": 48000, "physical_sample_clock_qualified": False,
                "fuzz_execution": False, "physical_audio": False,
                "scope": "One declared original voice/mix/master stream through actual finite "
                "DMA, ESAI pins, serial receiver and bounded audio worker/private service. "
                "Optional original CC7/CC1 parser path uses own ring transport and a held "
                "voice. No full boot, MIDI allocation, automatic arbitration or RT/hardware claim."}


def main():
    midi = len(sys.argv) == 5 and sys.argv[4] == 'midi'
    require(len(sys.argv) == 4 or midi,
            "Expected native test, private firmware, artifact directory and optional midi mode")
    binary, firmware, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:4])
    if not firmware.is_file():
        print("SKIP original host voice: private firmware absent")
        return 77
    for command in ("pulseaudio", "parec", "pactl"):
        if shutil.which(command) is None:
            print(f"SKIP original host voice: {command} absent")
            return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    artifacts.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="blofeld-original-host-") as temporary:
        directory = Path(temporary)
        try:
            result = run(binary, firmware, directory, xdata, ydata, program, midi)
            (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
        finally:
            for path in directory.rglob("*"):
                if path.is_file() and path.suffix in (".log", ".trace", ".pcm", ".json"):
                    destination = artifacts / directory.name / path.relative_to(directory)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(path, destination)
    return 0


if __name__ == "__main__":
    sys.exit(main())
