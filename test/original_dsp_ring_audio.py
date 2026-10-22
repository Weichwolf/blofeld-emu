# SPDX-License-Identifier: GPL-3.0-only
"""Verify all original ring contexts through distinct source-owned native workers."""

import itertools
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
from original_dsp_repeat import BLOCKS, Pipeline, SOUND, negative_controls
from original_dsp_voice import original_records
from original_dsp_voice_audio import compare

ROOT = Path(__file__).resolve().parents[1]
IDENTITIES = tuple((kind, mode, '127', str(int(kind == 'sync')), routing)
                   for kind, mode, routing in itertools.product(('normal', 'sync'),
                                                               ('0', '1'), ('0', '1')))


def expected_payload(text, xdata, ydata, program, identity):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    require(len(cases) == 1 and tuple(cases[0][:5]) == identity,
            'Direct audio must select exactly the requested original ring context')
    varied, twos = identity[3] == '1', identity[1] == '1'

    class Capture(Pipeline):
        def __init__(self):
            super().__init__(varied, twos, program)
            self.payload = bytearray()

        def serial(self, text, check):
            super().serial(text, check)
            for raw in self.halves[self.block % 2]:
                self.payload.extend(struct.pack('<i', signed(raw) * 256))

    capture = Capture()
    stream = cases[0][5]
    inputs = [0xb0, 7, 127, 1, 0, 7, 127] if varied else [0xb0, 7, 0]
    require(row('REPEAT_AUTOMATIC_REJECTED', stream) == [1] and
            row('FOREGROUND_INPUT', stream) == inputs,
            'Direct audio must retain the timing refusal and actual controller input')
    require([ydata[0x1e25 + index] & 255 for index in (129, 145)] == [0, 0],
            'Changed original factory FX Mix0/0 profile')
    totals = new_totals()
    validate_stream(stream, xdata, ydata, {twos: waveform_model(ydata, twos)}, identity,
                    totals, block_count=BLOCKS, before_block=capture.before,
                    after_block=capture.after, sound_base=SOUND)
    require(totals['blocks'] == 6 and totals['dependent_blocks'] == 5 and
            capture.nonzero > 0 and len(capture.payload) == 96 * 8,
            'Every original ring block must reach the independent audio oracle')
    return bytes(capture.payload)


def render(binary, firmware, trace, server, env, directory, index, xdata, ydata, program):
    result = subprocess.run([str(binary), str(firmware), str(trace), server, str(index)],
                            env=env, capture_output=True, text=True, check=False,
                            timeout=240 if env.get('BLOFELD_TEST_SANITIZER') == 'thread' else 90)
    diagnostics = result.stdout + result.stderr
    (directory / f'client-{index}.log').write_text(diagnostics)
    require(result.returncode == 0 and not result.stderr and
            not any(marker in diagnostics for marker in
                    ('AddressSanitizer', 'LeakSanitizer', 'ThreadSanitizer', 'runtime error:')),
            'Direct original ring audio failed: ' + diagnostics)
    require(re.findall(r'^direct-ring-frames=(\d+) playback-rate=48000 '
                       r'physical-clock=unqualified source-reuse-rejected=1$',
                       result.stdout, re.M) == ['96'],
            'Every worker must retain one source and accept/write/finish 96 decoded frames')
    text = trace.read_text()
    identity = IDENTITIES[index]
    expected = expected_payload(text, xdata, ydata, program, identity)

    def validate(changed, xdata, ydata, program):
        return expected_payload(changed, xdata, ydata, program, identity)

    negative_controls(text, xdata, ydata, program, validator=validate)
    return expected


def monitor_controls(recorded, expected):
    for index, stream in enumerate(expected):
        changed = bytearray(stream)
        changed[-8] ^= 1
        corrupted = list(expected)
        corrupted[index] = bytes(changed)
        try:
            compare(recorded, corrupted)
        except ValueError:
            pass
        else:
            raise ValueError('Private monitor accepted a corrupted original ring stream')


def playback(binary, firmware, artifacts, directory, xdata, ydata, program):
    with service(directory) as (daemon, server, env):
        recorded = directory / 'monitor.pcm'
        expected = []
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

                until(ready, daemon, 'the original ring matrix monitor')
                for index in range(len(IDENTITIES)):
                    expected.append(render(binary, firmware, artifacts / f'case-{index}.trace',
                                           server, env, directory, index, xdata, ydata, program))
                time.sleep(0.1)
            finally:
                terminate(monitor)
        require(not (directory / 'monitor.log').read_text(), 'Private monitor failed')
        actual = recorded.read_bytes()
        offsets = compare(actual, expected)
        monitor_controls(actual, expected)
        return {'passed': True, 'original_foreground_blocks': 48, 'contexts': IDENTITIES,
                'host_streams': 8, 'accepted_frames': 768, 'exact_host_frames': 768,
                'rejected_foreign_sources': 8, 'oracle_negative_controls': 32,
                'monitor_negative_controls': 8, 'monitor_offsets_bytes': offsets,
                'reference_sample_rate': 48000, 'reference_timebase': 24576000,
                'fuzz_execution': False, 'physical_audio': False,
                'scope': 'Eight independently initialized original foreground/repeating DMA/'
                'ESAI sources, each delivering 96 real decoded frames directly to its own '
                'source-owned AudioRuntime and exact private monitor. Explicit untimed '
                'admissions, paused-core phases, Effects Off and declared RAM/held entry. '
                'Short corked sources start on EOF. No calibrated time, sustained RT, '
                'full boot, normal USB-MIDI, original NoteOn or hardware equivalence claim.'}


def main():
    binary, firmware, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:])
    artifacts.mkdir(parents=True, exist_ok=True)
    report = artifacts / 'result.json'
    report.unlink(missing_ok=True)
    for tool in ('pulseaudio', 'parec', 'pactl'):
        if shutil.which(tool) is None:
            print('SKIP original ring audio matrix: missing ' + tool)
            return 77
    if not firmware.is_file():
        print('SKIP original ring audio matrix: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    with tempfile.TemporaryDirectory(prefix='bf-ring-matrix-') as temporary:
        directory = Path(temporary)
        try:
            result = playback(binary, firmware, artifacts, directory, xdata, ydata, program)
            report.write_text(json.dumps(result, indent=2) + '\n')
            print('Verified eight source-owned workers, 768 original ring frames and '
                  'all source/payload controls; full boot/MIDI/realtime remain open.')
        finally:
            for file in directory.rglob('*'):
                if file.is_file() and file.suffix in ('.log', '.pcm', '.pa'):
                    shutil.copyfile(file, artifacts / file.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
