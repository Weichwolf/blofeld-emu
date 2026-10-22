# SPDX-License-Identifier: GPL-3.0-only
"""Independent original foreground/master pipeline with a continuously enabled DMA ring."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import FULL_MASK, RESET_LEVEL, W, WORD_MASK, qword, round_word, signed, waveform_model
from dsp_voice_validation import new_totals, validate_stream
from original_dsp_foreground import SOUND, row
from original_dsp_voice import original_records
from original_dsp_voice_master import Output

ROOT = Path(__file__).resolve().parents[1]
BLOCKS = 6


class Pipeline:
    def __init__(self, varied, twos, program):
        self.varied, self.twos = varied, twos
        self.target = 512 * 127 ** 2 if varied else 0
        self.current = self.target if varied else RESET_LEVEL
        self.pending = True
        self.cursor = 0x2f80 + (7 if varied else 3)
        self.master = Output(program)
        self.mix = [0] * 32
        self.halves = [[0] * 32, [0] * 32]
        self.block = 0
        self.nonzero = 0

    def filter(self):
        result = []
        master = self.master
        for index, sample in enumerate(self.mix):
            lane = index % 2
            full = (2 * master.current * sample
                    - 2 * master.previous * master.history[lane + 2]
                    + 2 * master.feedback * master.history[lane])
            value = qword(full)
            result.append(value & WORD_MASK)
            master.history[lane], master.history[lane + 2] = value, sample
        return result

    def before(self, voice, text, block):
        self.block = block
        output = self.filter()
        self.halves[1 - block % 2] = output
        if row('REPEAT_MASTER', text) != output:
            raise ValueError('REPEAT_MASTER differs from the previous original mix/history')
        if row('REPEAT_HISTORY', text) != [v & WORD_MASK for v in self.master.history]:
            raise ValueError('REPEAT_HISTORY differs from independent master recurrence')
        # Original factory Mix0/0 selects the dry path. Its coefficient is
        # limited to Q23 7fffff, rather than an invented exact unity.
        old = voice.bus if block else [0] * 32
        self.mix = [qword(2 * 0x7fffff * signed(value)) for value in old]
        if row('REPEAT_MIX', text) != [v & WORD_MASK for v in self.mix]:
            raise ValueError('REPEAT_MIX differs from original copy/mix/output accumulation')
        if row('REPEAT_CLEARED', text) != [0] * 32 or row('REPEAT_GAINS', text) != [0] * 2:
            raise ValueError('REPEAT_CLEARED/GAINS differ from the declared Effects Off profile')
        voice.bus = [0] * 32
        voice.controller, voice.stack_base = self.current, 2
        voice.tick = block * 0x1000

    def scheduler(self, voice, text, check):
        block = self.block
        previous = self.current
        if self.pending:
            smoothed = round_word((63 * previous + self.target) * (W // 64), self.twos) // W
            delta = smoothed - self.target
            settled = abs(delta) <= 250
            self.current = self.target if settled else smoothed
            state = row('FOREGROUND_END', text)
            expected = [0x198, 1, delta * W & FULL_MASK, 250 * W,
                        previous * W + self.target, 0x840000 * W + 0x7e0000, SOUND + 404]
            if state[:7] != expected:
                raise ValueError('FOREGROUND_END smoothing registers differ')
            self.pending = not settled
        else:
            state = row('FOREGROUND_END', text)
            if (state[:4] != [0x198, 1, (block + 1) * 0x4800 * W, 0]
                    or state[4] & WORD_MASK or state[5] != 0x840000 * W + 0x7fffff
                    or state[6] != 0x6d4):
                raise ValueError('FOREGROUND_END idle scheduler registers differ')
        if len(state) != 8 or state[7] & ((1 << 21) | 0xc00) != int(self.twos) << 21:
            raise ValueError('FOREGROUND_END mode differs')
        seed = (voice.seed >> 1) ^ (0x840000 if voice.seed & 1 else 0)
        check('FOREGROUND_RAM', [(block + 1) * 0x1000, seed, 0x80 if self.pending else 0,
                                SOUND + 404, self.target, 0, 0x7fffff,
                                (block + 1) * 0x4800, 0, self.current,
                                self.cursor, self.cursor,
                                (1 - block % 2) * 32, 0x660 + block % 2 * 32], text)
        check('FOREGROUND_PREVIOUS_OUTPUT', self.halves[1], text)
        voice.seed = seed

    def serial(self, text, check):
        block = self.block
        expected = self.halves[block % 2]
        transfers = [[i, 0x660 + (block % 2) * 32 + i, 0xffff80, sample]
                     for i, sample in enumerate(expected)]
        actual = [[int(v, 16) for v in values.split()]
                  for values in re.findall(r'^REPEAT_DMA ([^\n]+)$', text, re.M)]
        if actual != transfers:
            raise ValueError('REPEAT_DMA ring addresses or previous-half payload differ')
        frames = [[i, 267 + (block * 16 + i) * 512,
                   267 + (block * 16 + i) * 512 + 504, 8, *expected[2 * i:2 * i + 2]]
                  for i in range(16)]
        actual = [[int(v, 16) for v in values.split()]
                  for values in re.findall(r'^REPEAT_FRAME ([^\n]+)$', text, re.M)]
        if actual != frames:
            raise ValueError('REPEAT_FRAME differs from original pipeline/edge/slot equations')
        restores = re.findall(r'^REPEAT_RESTORE (\d+)$', text, re.M)
        if restores != (['1'] if block == 0 else []):
            raise ValueError('REPEAT_RESTORE missing or duplicated')
        self.nonzero += sum(v != 0 for v in expected)

    def after(self, voice, text, check):
        self.scheduler(voice, text, check)
        self.serial(text, check)


def validate(text, xdata, ydata, program):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    expected = [(kind, mode, '127', str(int(kind == 'sync')), routing)
                for kind, mode, routing in itertools.product(('normal', 'sync'), ('0', '1'), ('0', '1'))]
    if [tuple(case[:5]) for case in cases] != expected:
        raise ValueError('Incomplete original repeating foreground matrix')
    if [ydata[0x1e25 + index] & 255 for index in (129, 145)] != [0, 0]:
        raise ValueError('Changed declared factory FX Mix0/0 profile')
    tables = {mode: waveform_model(ydata, mode) for mode in (False, True)}
    totals = new_totals()
    totals['nonzero_serial_words'] = 0
    for *identity, stream in cases:
        pipeline = Pipeline(identity[3] == '1', identity[1] == '1', program)
        if row('REPEAT_AUTOMATIC_REJECTED', stream) != [1]:
            raise ValueError('Automatic DMA timing guard was not retained')
        expected_bytes = [0xb0, 7, 127, 1, 0, 7, 127] if pipeline.varied else [0xb0, 7, 0]
        if row('FOREGROUND_INPUT', stream) != expected_bytes:
            raise ValueError('Original stream controller bytes differ')
        validate_stream(stream, xdata, ydata, tables, tuple(identity), totals, block_count=BLOCKS,
                        before_block=pipeline.before, after_block=pipeline.after, sound_base=SOUND)
        if not pipeline.nonzero:
            raise ValueError('Repeating original stream must decode nonzero original samples')
        totals['nonzero_serial_words'] += pipeline.nonzero
    if totals['blocks'] != 48 or totals['dependent_blocks'] != 40:
        raise ValueError('Incomplete persistent original stream histories')
    return totals


def negative_controls(text, xdata, ydata, program, validator=validate):
    for label in ('REPEAT_MASTER', 'REPEAT_MIX', 'REPEAT_DMA', 'REPEAT_FRAME'):
        def corrupt(match):
            words = match[1].split()
            words[-1] = format(int(words[-1], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing repeating oracle negative control')
        try:
            validator(changed, xdata, ydata, program)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong repeating negative-control failure') from error
        else:
            raise ValueError('Independent repeating oracle accepted corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original repeating foreground: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=800, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in
                               ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Original repeating native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program)
    negative_controls(text, xdata, ydata, program)
    report.write_text(json.dumps({'passed': True, 'streams': 8, **totals,
                                  'dma_words': 1536, 'decoded_frames': 768,
                                  'held_word_restores': 8, 'oracle_negative_controls': 4,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Original persistent foreground and enabled repeating '
                                  'DMA/ESAI ring. Untimed native access admissions alternate '
                                  'with paused-core serial/grant phases. Effects Off, declared '
                                  'initial RAM/phase and non-due part clock. No physical '
                                  'arbitration/clock, full boot, live host or NoteOn claim.'},
                                 indent=2) + '\n')
    print('Verified 48 consecutive original foreground blocks and 768 repeating ring PCM frames; '
          'physical DMA timing, boot and live host MIDI remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
