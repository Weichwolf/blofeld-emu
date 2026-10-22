# SPDX-License-Identifier: GPL-3.0-only
"""Deliver actual original pin-decoded streams through the native audio worker."""

import json
import os
from pathlib import Path
import re
import select
import shutil
import struct
import subprocess
import sys
import tempfile
import time

from host_audio import require, service, terminate, until
from original_dsp_voice import original_records
from original_dsp_voice_master import validate as validate_master
from original_dsp_voice_pcm import Pcm, validate as validate_pcm

ROOT = Path(__file__).resolve().parents[1]


def expected_streams(text, xdata, ydata, program, reference):
    streams = []

    class Expected(Pcm):
        def __init__(self):
            self.samples = bytearray()
            streams.append(self.samples)

        def __call__(self, expected, block, index):
            super().__call__(expected, block, index)
            for raw in expected:
                signed = raw if raw < 1 << 23 else raw - (1 << 24)
                self.samples.extend(struct.pack('<i', signed * 256))

    validate_master(text, xdata, ydata, program, reference, consumer=Expected)
    require(len(streams) == 32 and all(len(stream) == 128 * 8 for stream in streams),
            'Incomplete independent original audio matrix')
    return list(map(bytes, streams))


def ready(child):
    prefix = b''
    timeout = 480 if os.environ.get('BLOFELD_TEST_SANITIZER') == 'thread' else 280
    deadline = time.monotonic() + timeout
    while b'AUDIO_READY' not in prefix.splitlines():
        remaining = deadline - time.monotonic()
        require(remaining > 0 and len(prefix) < 16384, 'Original native audio render did not finish')
        readable, _, _ = select.select([child.stdout], [], [], min(remaining, 1))
        require(child.poll() is None, f'Original native renderer exited early: {prefix!r}')
        if readable:
            chunk = os.read(child.stdout.fileno(), 4096)
            require(chunk, 'Original renderer closed output before readiness')
            prefix += chunk
    return prefix.decode()


def compare(recorded, expected):
    require(len(recorded) % 8 == 0, 'Truncated native service stereo container')
    cursor, offsets = 0, []
    for stream in expected:
        position = recorded.find(stream, cursor)
        require(position >= 0 and position % 8 == 0,
                f'Missing/truncated original service stream after byte {cursor}')
        require(not any(recorded[cursor:position]), 'Unexpected duplicated/stale service samples')
        offsets.append(position)
        cursor = position + len(stream)
    require(not any(recorded[cursor:]), 'Unexpected samples after complete original delivery')
    return offsets


def playback(binary, firmware, trace, directory, xdata, ydata, program, reference):
    with service(directory) as (daemon, server, env):
        child = subprocess.Popen([str(binary), str(firmware), str(trace), server], env=env,
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE)
        recorder = None
        try:
            prefix = ready(child)
            text = trace.read_text()
            native = validate_pcm(text, xdata, ydata, program, reference)
            expected = expected_streams(text, xdata, ydata, program, reference)
            capture = directory / 'monitor.pcm'
            with capture.open('wb') as data, (directory / 'monitor.log').open('w') as log:
                recorder = subprocess.Popen(['parec', '--raw', '--format=s32le', '--rate=48000',
                                             '--channels=2', '--device=fixture.monitor',
                                             '--latency-msec=5'], env=env, stdout=data, stderr=log)

                def recording():
                    result = subprocess.run(['pactl', '--server', server, 'list', 'short',
                                             'source-outputs'], env=env, text=True,
                                            capture_output=True, timeout=2, check=True)
                    return bool(result.stdout.strip())

                until(recording, daemon, 'the original audio monitor')
                output, error = child.communicate(b'play\n', timeout=120)
                diagnostics = error.decode()
                (directory / 'client.log').write_text(prefix + output.decode() + diagnostics)
                require(child.returncode == 0 and not any(marker in diagnostics for marker in
                        ('AddressSanitizer', 'LeakSanitizer', 'ThreadSanitizer', 'runtime error:')),
                        f'Original audio worker failed: {diagnostics}')
                time.sleep(0.1)
                terminate(recorder)
                recorder = None
            rows = re.findall(r'^AUDIO_STREAM (\d+) (\d+) (\d+) (\d+)$', output.decode(), re.M)
            require([tuple(map(int, row[:3])) for row in rows] ==
                    [(index, 128, 128) for index in range(32)], 'Incomplete native worker delivery')
            require(not (directory / 'monitor.log').read_text(), 'Private audio monitor failed')
            offsets = compare(capture.read_bytes(), expected)
            return {'passed': True, **native, 'host_streams': len(offsets),
                    'exact_host_frames': 4096, 'monitor_offsets_bytes': offsets,
                    'reference_sample_rate': 48000, 'reference_timebase': 24576000,
                    'fuzz_execution': False, 'physical_audio': False,
                    'scope': 'Offline original synthesis/DMA/ESAI captures through AudioRuntime '
                    'to a private no-rewind service monitor. Declared reference clock; '
                    'full boot, live MIDI, realtime production and physical rate remain open.'}
        finally:
            terminate(recorder)
            terminate(child)
            remainder = child.stderr.read() if not child.stderr.closed else b''
            if remainder:
                (directory / 'native-failure.log').write_bytes(remainder)


def main():
    binary, firmware, artifacts = map(lambda value: Path(value).resolve(), sys.argv[1:])
    artifacts.mkdir(parents=True, exist_ok=True)
    report = artifacts / 'result.json'
    report.unlink(missing_ok=True)
    for tool in ('pulseaudio', 'parec', 'pactl'):
        if shutil.which(tool) is None:
            print(f'SKIP original voice audio: missing {tool}')
            return 77
    if not firmware.is_file():
        print('SKIP original voice audio: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    with tempfile.TemporaryDirectory(prefix='bf-voice-') as temporary:
        directory = Path(temporary)
        try:
            result = playback(binary, firmware, artifacts / 'native.trace', directory,
                              xdata, ydata, program, reference)
            report.write_text(json.dumps(result, indent=2) + '\n')
            print('Verified 4096 original pin-decoded frames through the native audio worker '
                  'and exact private service monitor; live boot/MIDI/realtime remain open.')
        finally:
            for file in directory.rglob('*'):
                if file.is_file() and file.suffix in ('.log', '.pcm', '.pa'):
                    shutil.copyfile(file, artifacts / file.name)
    return 0


if __name__ == '__main__':
    sys.exit(main())
