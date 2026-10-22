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
from original_dsp_foreground import row
from original_dsp_repeat import BLOCKS, Pipeline

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


class RepeatCapture(Pipeline):
    def __init__(self, program):
        super().__init__(False, False, program)
        self.payload = bytearray()

    def after(self, voice, text, check):
        super().after(voice, text, check)
        stages = [[int(v, 16) for v in values.split()]
                  for values in re.findall(r'^FOREGROUND_STAGE ([^\n]+)$', text, re.M)]
        expected = [[0x198, 1], *[[0xc53, 2]] * (4 if self.block == 0 else 1),
                    [0xc5d, 2], [0x2407, 2], [0x27b, 2], [0x2a20, 2],
                    [0x298, 2], [0x251, 2], [0x298, 2], [0x1cc1, 2],
                    [0x436, 2], [0x4df, 3], [0x22d, 1], [0x246, 1]]
        require(stages == expected, 'FOREGROUND_STAGE repeating parent order/frame differs')
        for raw in self.halves[self.block % 2]:
            self.payload.extend(struct.pack('<i', signed(raw) * 256))


def expected_repeat(stream, xdata, ydata, program, identity):
    require(row('REPEAT_AUTOMATIC_REJECTED', stream) == [1],
            'Automatic active-DMA timing rejection must remain strict')
    require(row('FOREGROUND_INPUT', stream) == [0xb0, 7, 0],
            'FOREGROUND_INPUT repeating source bytes differ')
    require([ydata[0x1e25 + index] & 255 for index in (129, 145)] == [0, 0],
            'Changed original factory FX Mix0/0 profile')
    capture = RepeatCapture(program)
    totals = new_totals()
    validate_stream(stream, xdata, ydata, {False: waveform_model(ydata, False)},
                    identity, totals, block_count=BLOCKS, before_block=capture.before,
                    after_block=capture.after, sound_base=SOUND)
    require(totals['blocks'] == BLOCKS and capture.nonzero > 0 and
            len(capture.payload) == 96 * 8,
            'Incomplete original repeating parent/DMA/serial source')
    return bytes(capture.payload)


def expected_payload(text, xdata, ydata, program, midi=False, stream=False):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    identity = ('normal', '0', '127' if stream else '0', '0', '0')
    require(len(cases) == 1 and tuple(cases[0][:5]) == identity,
            "Host handoff must select exactly one declared default voice stream")
    if stream:
        return expected_repeat(cases[0][5], xdata, ydata, program, identity)
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


def trace_controls(text, xdata, ydata, program, stream):
    labels = (('REPEAT_MASTER', -1), ('REPEAT_MIX', -1),
              ('REPEAT_DMA', -1), ('REPEAT_FRAME', -1)) if stream else (
              ('MIDI_BYTES', -1), ('MIDI_RETURN', -3), ('OUTPUT_BUS', 0), ('PCM_FRAME', -1))
    for label, index in labels:
        def corrupt(match):
            words = match[1].split()
            words[index] = format(int(words[index], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        require(count == 1, 'Missing direct controller/audio negative control')
        try:
            expected_payload(changed, xdata, ydata, program, midi=not stream, stream=stream)
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


def run(binary, firmware, directory, xdata, ydata, program, midi, stream):
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
                if stream:
                    command.append('stream')
                elif midi:
                    command.append('midi')
                threaded = env.get('BLOFELD_TEST_SANITIZER') == 'thread'
                timeout = (240 if threaded else 90) if stream else (90 if threaded else 30)
                result = subprocess.run(command, env=env,
                                        capture_output=True, text=True,
                                        timeout=timeout)
                (directory / "native.log").write_text(result.stdout + result.stderr)
                require(result.returncode == 0 and not result.stderr and
                        f"original-decoded-host-frames={96 if stream else 128}" in result.stdout,
                        f"Original native host handoff failed: {result.stdout}{result.stderr}")
                text = trace.read_text()
                expected = expected_payload(text, xdata, ydata, program, midi, stream)
                if midi or stream:
                    trace_controls(text, xdata, ydata, program, stream)
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
        return {"passed": True, "original_blocks": 6 if stream else 8,
                "decoded_frames": 96 if stream else 128,
                "exact_independent_monitor_match": True, "monitor_negative_controls": 1,
                "original_controller_parser_calls": (len(re.findall(
                    r"^FOREGROUND_STAGE c53 2$", text, re.M)) if stream else 19 if midi else 0),
                "controller_trace_negative_controls": 4 if midi else 0,
                "repeating_trace_negative_controls": 4 if stream else 0,
                "original_foreground_iterations": 6 if stream else 0,
                "declared_playback_rate": 48000, "physical_sample_clock_qualified": False,
                "fuzz_execution": False, "physical_audio": False,
                "scope": "One declared original voice/mix/master stream through actual "
                "DMA, ESAI pins, serial receiver and bounded audio worker/private service. "
                "Optional original repeating foreground uses untimed native access tickets "
                "and paused-core serial phases. Original CC7/CC1 uses own ring input and a held "
                "voice. No full boot, MIDI allocation, automatic arbitration or RT/hardware claim."}


def main():
    midi = len(sys.argv) == 5 and sys.argv[4] == 'midi'
    stream = len(sys.argv) == 5 and sys.argv[4] == 'stream'
    require(len(sys.argv) == 4 or midi or stream,
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
            result = run(binary, firmware, directory, xdata, ydata, program, midi, stream)
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
