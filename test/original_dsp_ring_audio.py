# SPDX-License-Identifier: GPL-3.0-only
"""Compare a direct original foreground/ring source with a private host audio monitor."""

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
from original_dsp_foreground import row
from original_dsp_idle_midi import BLOCKS as MIDI_BLOCKS, IdlePipeline
from original_dsp_idle_midi import negative_controls as midi_negative_controls
from original_dsp_repeat import BLOCKS, Pipeline, SOUND, negative_controls
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def expected_payload(text, xdata, ydata, program, midi=False):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    identity = ('normal', '0', '127', '0', '0')
    require(len(cases) == 1 and tuple(cases[0][:5]) == identity,
            'Direct audio must select one complete declared original ring stream')

    class Capture(IdlePipeline if midi else Pipeline):
        def __init__(self):
            super().__init__(False, False, program)
            self.payload = bytearray()

        def serial(self, text, check):
            super().serial(text, check)
            for raw in self.halves[self.block % 2]:
                self.payload.extend(struct.pack('<i', signed(raw) * 256))

    capture = Capture()
    stream = cases[0][5]
    require(row('REPEAT_AUTOMATIC_REJECTED', stream) == [1] and
            row('FOREGROUND_INPUT', stream) == [0xb0, 7, 0],
            'Direct audio must retain the DMA timing refusal and actual controller input')
    require([ydata[0x1e25 + index] & 255 for index in (129, 145)] == [0, 0],
            'Changed original factory FX Mix0/0 profile')
    totals = new_totals()
    blocks = MIDI_BLOCKS if midi else BLOCKS
    validate_stream(stream, xdata, ydata, {False: waveform_model(ydata, False)}, identity,
                    totals, block_count=blocks, before_block=capture.before,
                    after_block=capture.after, sound_base=SOUND)
    require(totals['blocks'] == blocks and totals['dependent_blocks'] == blocks - 1 and
            capture.nonzero > 0 and len(capture.payload) == blocks * 16 * 8,
            'Every native original ring block must reach the independent audio oracle')
    return bytes(capture.payload)


def check_monitor(recorded, expected):
    position = recorded.find(expected)
    require(len(recorded) % 8 == 0 and position >= 0 and position % 8 == 0,
            'Private monitor lacks the complete aligned original ring audio prefix')
    require(not recorded[:position].strip(b'\0') and
            not recorded[position + len(expected):].strip(b'\0'),
            'Private monitor has extra, stale, duplicated or changed source samples')
    return position


def playback(binary, firmware, trace, directory, xdata, ydata, program, midi=False):
    blocks = MIDI_BLOCKS if midi else BLOCKS
    frames = blocks * 16
    with service(directory) as (daemon, server, env):
        recorded = directory / 'monitor.pcm'
        with recorded.open('wb') as samples, (directory / 'monitor.log').open('w') as log:
            monitor = subprocess.Popen(['parec', '--raw', '--format=s32le', '--rate=48000',
                                        '--channels=2', '--device=fixture.monitor',
                                        '--latency-msec=5'], env=env, stdout=samples, stderr=log)
            try:
                def ready():
                    outputs = subprocess.run(['pactl', '--server', server, 'list', 'short',
                                               'source-outputs'], env=env, capture_output=True,
                                              text=True, timeout=2, check=True)
                    return bool(outputs.stdout.strip())

                until(ready, daemon, 'the direct original ring monitor')
                command = [str(binary), str(firmware), str(trace), server, *(['midi'] if midi else [])]
                result = subprocess.run(command, env=env,
                                        capture_output=True, text=True, check=False,
                                        timeout=240 if env.get('BLOFELD_TEST_SANITIZER') == 'thread' else 90)
                diagnostics = result.stdout + result.stderr
                (directory / 'client.log').write_text(diagnostics)
                require(result.returncode == 0 and not result.stderr and
                        not any(marker in diagnostics for marker in
                                ('AddressSanitizer', 'LeakSanitizer', 'ThreadSanitizer', 'runtime error:')),
                        'Direct original ring audio failed: ' + diagnostics)
                require(re.findall(r'^direct-ring-frames=(\d+) playback-rate=48000 '
                                   r'physical-clock=unqualified$', result.stdout, re.M) == [str(frames)],
                        f'The actual worker must accept, write and finish all {frames} decoded frames')
                time.sleep(0.1)
            finally:
                terminate(monitor)
        require(not (directory / 'monitor.log').read_text(), 'Private monitor failed')
        text = trace.read_text()
        def validator(text, xdata, ydata, program):
            return expected_payload(text, xdata, ydata, program, midi=midi)

        expected = validator(text, xdata, ydata, program)
        controls = midi_negative_controls if midi else negative_controls
        controls(text, xdata, ydata, program, validator=validator)
        position = check_monitor(recorded.read_bytes(), expected)
        return {'passed': True, 'original_foreground_blocks': blocks, 'accepted_frames': frames,
                'exact_host_frames': frames, 'oracle_negative_controls': 9 if midi else 4,
                'idle_midi': midi,
                'monitor_offset_bytes': position, 'reference_sample_rate': 48000,
                'reference_timebase': 24576000, 'fuzz_execution': False, 'physical_audio': False,
                'scope': 'Direct original controller/foreground/repeating DMA/ESAI frames '
                'to AudioRuntime, verified at a private no-rewind monitor. Native source '
                'remains untimed, Effects Off and predeclared held entry/RAM. Source is '
                'shorter than host prebuffer and starts on EOF; no realtime, physical '
                'clock, full boot, live host MIDI or NoteOn-allocation qualification.'}


def main():
    midi = sys.argv[4:] == ['midi']
    require(len(sys.argv) == 4 or midi, 'Expected binary, firmware, artifacts and optional midi')
    binary, firmware, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:4])
    artifacts.mkdir(parents=True, exist_ok=True)
    report = artifacts / 'result.json'
    report.unlink(missing_ok=True)
    for tool in ('pulseaudio', 'parec', 'pactl'):
        if shutil.which(tool) is None:
            print('SKIP direct original ring audio: missing ' + tool)
            return 77
    if not firmware.is_file():
        print('SKIP direct original ring audio: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    with tempfile.TemporaryDirectory(prefix='bf-direct-ring-') as temporary:
        directory = Path(temporary)
        try:
            result = playback(binary, firmware, artifacts / 'native.trace', directory,
                              xdata, ydata, program, midi=midi)
            report.write_text(json.dumps(result, indent=2) + '\n')
            print(f"Verified {result['exact_host_frames']} direct original ring frames at the private host audio monitor; "
                  'full boot, realtime playback and live host MIDI remain open.')
        finally:
            for file in directory.rglob('*'):
                if file.is_file() and file.suffix in ('.log', '.pcm', '.pa'):
                    shutil.copyfile(file, artifacts / file.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
