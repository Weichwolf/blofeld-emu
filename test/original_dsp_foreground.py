# SPDX-License-Identifier: GPL-3.0-only
"""Check original foreground routing, held-voice arithmetic and scheduled smoothing."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import FULL_MASK, RESET_LEVEL, W, round_word, waveform_model
from dsp_voice_validation import new_totals, validate_stream
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
SOUND = 0x936


def row(label, text):
    values = re.findall('^' + label + r' ([^\n]+)$', text, re.M)
    if len(values) != 1:
        raise ValueError('Missing/duplicate ' + label)
    return [int(value, 16) for value in values[0].split()]


class Foreground:
    def __init__(self, varied, twos):
        self.varied, self.twos = varied, twos
        self.target = 512 * 127 ** 2 if varied else 0
        self.previous = self.target if varied else RESET_LEVEL

    def before(self, voice, _text, _block):
        # The real parent mixes/clears the old voice bus before this first voice.
        voice.bus = [0] * 32
        voice.controller = self.previous
        voice.stack_base = 2

    def after(self, voice, text, check):
        byte_count = 7 if self.varied else 3
        stages = [[int(v, 16) for v in values.split()]
                  for values in re.findall(r'^FOREGROUND_STAGE ([^\n]+)$', text, re.M)]
        expected = [[0x198, 1], *[[0xc53, 2]] * (byte_count + 1),
                    [0xc5d, 2], [0x2407, 2], [0x27b, 2], [0x2a20, 2],
                    [0x298, 2], [0x251, 2], [0x298, 2], [0x1cc1, 2],
                    [0x436, 2], [0x4df, 3], [0x22d, 1], [0x246, 1]]
        if stages != expected:
            raise ValueError('FOREGROUND_STAGE call order or native frame depth differs')
        # Original Q23 one-pole coefficient is 63/64, followed by native MACR.
        full = (63 * self.previous + self.target) * (W // 64)
        smoothed = round_word(full, self.twos) // W
        delta = smoothed - self.target
        settled = abs(delta) <= 250
        current = self.target if settled else smoothed
        seed = (voice.seed >> 1) ^ (0x840000 if voice.seed & 1 else 0)
        wanted = [0x1000, seed, 0 if settled else 0x80, SOUND + 404, self.target,
                  0, 0x4801, 0x4800, 1, current,
                  0x2f80 + byte_count, 0x2f80 + byte_count, 32, 0x660]
        check('FOREGROUND_RAM', wanted, text)
        state = row('FOREGROUND_END', text)
        expected_registers = [0x198, 1, delta * W & FULL_MASK, 250 * W,
                              self.previous * W + self.target,
                              0x840000 * W + 0x7e0000, SOUND + 404]
        if (state[:7] != expected_registers or len(state) != 8
                or state[7] & ((1 << 21) | 0xc00) != (int(self.twos) << 21)):
            raise ValueError('FOREGROUND_END full registers or mode differ')
        check('FOREGROUND_PREVIOUS_OUTPUT', [0] * 32, text)


def validate(text, xdata, ydata):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    expected = list(itertools.product(('normal', 'sync'), ('0', '1'), ('0', '127'),
                                     ('0', '1'), ('0', '1')))
    if [tuple(case[:5]) for case in cases] != expected:
        raise ValueError('Incomplete original foreground matrix')
    tables = {mode: waveform_model(ydata, mode) for mode in (False, True)}
    totals = new_totals()
    for *identity, stream in cases:
        foreground = Foreground(identity[3] == '1', identity[1] == '1')
        expected_bytes = [0xb0, 7, 127, 1, 0, 7, 127] if foreground.varied else [0xb0, 7, 0]
        if row('FOREGROUND_INPUT', stream) != expected_bytes:
            raise ValueError('FOREGROUND_INPUT bytes differ')
        validate_stream(stream, xdata, ydata, tables, tuple(identity), totals, block_count=1,
                        before_block=foreground.before, after_block=foreground.after,
                        sound_base=SOUND)
    if totals['blocks'] != 32 or totals['dependent_blocks']:
        raise ValueError('Wrong foreground voice scope')
    return totals


def negative_controls(text, xdata, ydata):
    labels = ('FOREGROUND_STAGE', 'FOREGROUND_RAM', 'FOREGROUND_END', 'OUTPUT_BUS')
    for label in labels:
        def corrupt(match):
            words = match[1].split()
            words[0] = format(int(words[0], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing foreground oracle negative control')
        try:
            validate(changed, xdata, ydata)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong foreground negative-control failure') from error
        else:
            raise ValueError('Independent foreground oracle accepted corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original DSP foreground: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    xdata, ydata = original_records(firmware, reference)
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
        raise ValueError('Original foreground native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, xdata, ydata)
    negative_controls(text, xdata, ydata)
    report.write_text(json.dumps({'passed': True, 'iterations': 32, **totals,
                                  'active_transitions': 16, 'settled_transitions': 16,
                                  'oracle_negative_controls': 4, 'fuzz_execution': False,
                                  'physical_traffic': False,
                                  'scope': 'One real original foreground iteration per declared '
                                  'Effects Off context. Original MIDI, voice and scheduler '
                                  'smoothing; no replaced callee/return. Disabled DMA with '
                                  'declared source/previous phase; no physical timing, '
                                  'continuous DMA, boot, live playback or NoteOn allocation.'},
                                 indent=2) + '\n')
    print('Verified 32 original DSP foreground iterations with original held voices and '
          'scheduler smoothing. Continuous DMA, boot and live MIDI remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
