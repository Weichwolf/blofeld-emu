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
from original_dsp_idle_midi import (BLOCKS as MIDI_BLOCKS, IdlePipeline,
                                   negative_controls as midi_negative_controls)
from original_dsp_repeat import BLOCKS, Pipeline, SOUND, negative_controls
from original_dsp_release_ring import (BLOCKS as RELEASE_BLOCKS,
                                       expected_payload as release_payload,
                                       negative_controls as release_negative_controls)
from original_dsp_voice import original_records
from original_dsp_voice_audio import compare

ROOT = Path(__file__).resolve().parents[1]
IDENTITIES = tuple((kind, mode, '127', str(int(kind == 'sync')), routing)
                   for kind, mode, routing in itertools.product(('normal', 'sync'),
                                                               ('0', '1'), ('0', '1')))


def expected_payload(text, xdata, ydata, program, identity, midi=False, release=False):
    if release:
        require(identity == IDENTITIES[0], 'Direct release oracle covers only the first context')
        return release_payload(text, xdata, ydata, program)
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    require(len(cases) == 1 and tuple(cases[0][:5]) == identity,
            'Direct audio must select exactly the requested original ring context')
    varied, twos = identity[3] == '1', identity[1] == '1'

    base = IdlePipeline if midi else Pipeline

    class Capture(base):
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
    blocks = MIDI_BLOCKS if midi else BLOCKS
    validate_stream(stream, xdata, ydata, {twos: waveform_model(ydata, twos)}, identity,
                    totals, block_count=blocks, before_block=capture.before,
                    after_block=capture.after, sound_base=SOUND)
    require(totals['blocks'] == blocks and totals['dependent_blocks'] == blocks - 1 and
            capture.nonzero > 0 and len(capture.payload) == blocks * 16 * 8,
            'Every original ring block must reach the independent audio oracle')
    if midi:
        require(capture.idle_polls == 24, 'Every MIDI source must execute 24 real idle polls')
    return bytes(capture.payload)


def render(binary, firmware, trace, server, env, directory, index, xdata, ydata, program,
           midi, release=False):
    result = subprocess.run([str(binary), str(firmware), str(trace), server, str(index),
                             *(['release'] if release else ['midi'] if midi else [])],
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
                       result.stdout, re.M) ==
            [str((RELEASE_BLOCKS if release else MIDI_BLOCKS if midi else BLOCKS) * 16)],
            'Every worker must retain one source and accept/write/finish all decoded frames')
    text = trace.read_text()
    identity = IDENTITIES[index]
    expected = expected_payload(text, xdata, ydata, program, identity, midi=midi, release=release)

    def validate(changed, xdata, ydata, program):
        return expected_payload(changed, xdata, ydata, program, identity, midi=midi, release=release)

    controls = release_negative_controls if release else midi_negative_controls if midi else negative_controls
    controls(text, xdata, ydata, program, validator=validate)
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


def playback(binary, firmware, artifacts, directory, xdata, ydata, program, midi, release=False):
    blocks = RELEASE_BLOCKS if release else MIDI_BLOCKS if midi else BLOCKS
    count = 1 if release else len(IDENTITIES)
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
                for index in range(count):
                    expected.append(render(binary, firmware, artifacts / f'case-{index}.trace',
                                           server, env, directory, index, xdata, ydata, program,
                                           midi, release))
                time.sleep(0.1)
            finally:
                terminate(monitor)
        require(not (directory / 'monitor.log').read_text(), 'Private monitor failed')
        actual = recorded.read_bytes()
        offsets = compare(actual, expected)
        monitor_controls(actual, expected)
        return {'passed': True, 'original_foreground_blocks': count * blocks,
                'contexts': IDENTITIES[:count],
                'host_streams': count, 'accepted_frames': count * blocks * 16,
                'exact_host_frames': count * blocks * 16,
                'rejected_foreign_sources': count,
                'oracle_negative_controls': count * (10 if release else 9 if midi else 4),
                'idle_midi': midi, 'idle_polls': count * 24 if midi else 0,
                'release_midi': release,
                'monitor_negative_controls': count, 'monitor_offsets_bytes': offsets,
                'reference_sample_rate': 48000, 'reference_timebase': 24576000,
                'fuzz_execution': False, 'physical_audio': False,
                'scope': f'{count} independently initialized original foreground/repeating DMA/'
                'ESAI sources, each delivering real decoded frames directly to its own '
                'source-owned AudioRuntime and exact private monitor. Explicit untimed '
                'admissions, paused-core phases, Effects Off and declared RAM/held entry. '
                'Short corked sources start on EOF. No calibrated time, sustained RT, '
                'full boot, normal USB-MIDI, original NoteOn or hardware equivalence claim.'}


def main():
    midi = sys.argv[4:] == ['midi']
    release = sys.argv[4:] == ['release']
    require(len(sys.argv) == 4 or midi or release,
            'Expected binary, firmware, artifacts and optional midi or release')
    binary, firmware, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:4])
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
            result = playback(binary, firmware, artifacts, directory, xdata, ydata, program,
                              midi, release)
            report.write_text(json.dumps(result, indent=2) + '\n')
            print(f"Verified {result['host_streams']} source-owned workers, "
                  f"{result['exact_host_frames']} original ring frames and "
                  'all source/payload controls; full boot/MIDI/realtime remain open.')
        finally:
            for file in directory.rglob('*'):
                if file.is_file() and file.suffix in ('.log', '.pcm', '.pa'):
                    shutil.copyfile(file, artifacts / file.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
