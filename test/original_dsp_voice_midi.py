# SPDX-License-Identifier: GPL-3.0-only
"""Verify original byte-parser controller changes through native voice/PCM output."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import RESET_LEVEL, waveform_model
from dsp_voice_validation import new_totals, validate_coverage, validate_stream
from original_dsp_voice import original_records
from original_dsp_voice_master import Output
from original_dsp_voice_pcm import Pcm

ROOT = Path(__file__).resolve().parents[1]
SOUND = 0x936
VOLUMES = (127, 0, 1, 64, 96, 127, 32, 0)


def messages(block):
    value = VOLUMES[block]
    if block == 2:
        chunks = [(0xb0,), (7,), (value,)]
    elif block == 1:
        chunks = [(7, value)]
    elif block == 3:
        chunks = [(0xb1, 7, 127), (0xb0, 7, value)]
    else:
        chunks = [(0xb0, 7, value)]
    return [*chunks, (1, 0)]


class Midi:
    def __init__(self):
        self.cursor = 0x2f80
        self.status = self.first = self.second = 0
        self.waiting_first = True
        self.destination = self.target = 0
        self.volume, self.wheel = RESET_LEVEL, 0
        self.calls = 0

    def deliver(self, chunk):
        for byte in chunk:
            self.cursor = 0x2f80 + ((self.cursor - 0x2f80 + 1) % 64)
            if byte >= 128:
                self.status, self.waiting_first = byte, True
            elif self.waiting_first:
                self.first, self.waiting_first = byte, False
            else:
                self.second, self.waiting_first = byte, True
                if self.status != 0xb0:
                    continue
                destination = SOUND + (404 if self.first == 7 else 383)
                if self.first not in (1, 7):
                    raise ValueError('Unexpected controller in independent input fixture')
                # Separate MIDI fields and Q23 square; no observed native word is an input.
                target = 512 * byte * byte if self.first == 7 else byte * 65536
                if self.destination and self.destination != destination:
                    if self.destination == SOUND + 404:
                        self.volume = self.target
                    else:
                        self.wheel = self.target
                self.destination, self.target = destination, target
        self.calls += 1
        return [self.cursor, self.cursor, self.status << 16, self.first << 16,
                self.second << 16, self.destination, self.target, 0x80,
                self.volume, self.wheel, 0]

    def __call__(self, voice, text, block):
        rows = re.findall(r'^MIDI_(BYTES|RETURN) ([^\n]+)$', text, re.M)
        expected = []
        for chunk in messages(block):
            expected.extend((('BYTES', list(chunk)), ('RETURN', self.deliver(chunk))))
        actual = [(label, [int(word, 16) for word in row.split()]) for label, row in rows]
        if actual != expected:
            raise ValueError(f'MIDI byte/state mismatch in block {block}: {actual} != {expected}')
        if self.volume != 512 * VOLUMES[block] ** 2 or self.wheel != 0:
            raise ValueError('Independent controller destination switching differs')
        voice.controller = self.volume


def validate(text, xdata, ydata, program, reference):
    tables = {mode: waveform_model(ydata, mode) for mode in (False, True)}
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    expected = list(itertools.product(('normal', 'sync'), ('0', '1'), ('0', '127'),
                                     ('0', '1'), ('0', '1')))
    if [tuple(row[:5]) for row in cases] != expected:
        raise ValueError('Incomplete controller/voice/PCM matrix')
    totals = new_totals()
    totals['parser_returns'] = 0
    for *identity, stream in cases:
        midi, output = Midi(), Output(program, Pcm())
        validate_stream(stream, xdata, ydata, tables, tuple(identity), totals,
                        before_block=midi, after_block=output, sound_base=SOUND)
        if output.blocks != 8 or not output.nonzero:
            raise ValueError('Controller stream must retain and render all eight blocks')
        totals['parser_returns'] += midi.calls
    validate_coverage(text, reference['coverage'])
    if totals['blocks'] != 256 or totals['parser_returns'] != 608:
        raise ValueError('Incomplete original parser/voice returns')
    return totals


def negative_controls(text, xdata, ydata, program, reference):
    for label, index in (('MIDI_BYTES', -1), ('MIDI_RETURN', -3),
                         ('OUTPUT_BUS', 0), ('PCM_FRAME', -1)):
        def corrupt(match):
            words = match[1].split()
            words[index] = format(int(words[index], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing controller/PCM oracle negative control')
        try:
            validate(changed, xdata, ydata, program, reference)
        except ValueError as error:
            expected = 'MIDI' if label.startswith('MIDI') else label
            if expected not in str(error):
                raise ValueError('Wrong controller/PCM negative-control failure') from error
        else:
            raise ValueError('Independent oracle accepted corrupted controller/PCM data')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original MIDI voice PCM: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=280, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in
                               ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError(f'Original MIDI/voice native failure: {diagnostics}')
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program, reference)
    negative_controls(text, xdata, ydata, program, reference)
    report.write_text(json.dumps({'passed': True, 'streams': 32, **totals,
                                  'decoded_frames': 4096, 'oracle_negative_controls': 4,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Original primary MIDI parser, CC7/CC1 destination '
                                  'switching and declared held voices through master, finite '
                                  'DMA and ESAI pins. Own transport/setup, paused-core serial '
                                  'clock. No note allocation, smoothing scheduler, full boot, '
                                  'host MIDI, live playback or hardware equivalence claim.'},
                                 indent=2) + '\n')
    print('Verified original controller bytes through 256 voice blocks and 4096 PCM frames; '
          'full boot, MIDI note allocation and live host routing remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
