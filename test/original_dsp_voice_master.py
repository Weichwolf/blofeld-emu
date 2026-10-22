# SPDX-License-Identifier: GPL-3.0-only
"""Check native original voices, copy/mix consumption and retained master histories."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import FULL_MASK, W, WORD_MASK, qword, signed, waveform_model
from dsp_voice_validation import new_totals, validate_coverage, validate_stream
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


class Output:
    def __init__(self, program, consume=None):
        self.current = qword(2 * (512 * 127 ** 2) * signed(program[0x27d]))
        self.previous = qword(2 * (512 * 127 ** 2) * signed(program[0x27f]))
        self.feedback = signed(program[0x285])
        self.history = [0] * 4
        self.halves = [[0] * 32, [0] * 32]
        self.blocks = 0
        self.nonzero = 0
        self.consume = consume

    def __call__(self, voice, text, check):
        check('MASTER_COPY', voice.bus, text)
        mixed = [qword(2 * ((W // 2) - 1) * signed(value)) for value in voice.bus]
        check('MASTER_MIX', [value & WORD_MASK for value in mixed], text)
        check('MASTER_CLEARED', [0] * 32, text)
        check('MASTER_SEND', [0] * 32, text)
        phase = (self.blocks % 2) * 32
        if re.findall(r'^MASTER_PHASE (\d+)$', text, re.M) != [str(phase)]:
            raise ValueError('Own phase alternation differs')
        output = []
        for index, sample in enumerate(mixed):
            lane = index % 2
            full = (2 * self.current * sample - 2 * self.previous * self.history[lane + 2]
                    + 2 * self.feedback * self.history[lane])
            value = qword(full)
            output.append(value & WORD_MASK)
            self.history[lane], self.history[lane + 2] = value, sample
        check('MASTER_OUTPUT', output, text)
        check('MASTER_OTHER', self.halves[1 - phase // 32], text)
        check('MASTER_HISTORY', [value & WORD_MASK for value in self.history], text)
        # The original final prefetch is X:$640, actually zeroed by its mix send.
        next_full = 2 * self.feedback * self.history[0]
        check('MASTER_RETURN', [next_full & FULL_MASK, full & FULL_MASK, 1,
                               0x641, 0x660 + phase, 0x47, 0x680 + phase], text)
        if self.consume is not None:
            self.consume(output, text, self.blocks)
        self.halves[phase // 32] = output
        self.nonzero += sum(value != 0 for value in output)
        self.blocks += 1
        # The original mix kernel consumed these words; never reset the native voice.
        voice.bus = [0] * 32


def validate(text, xdata, ydata, program, reference, consumer=None):
    tables = {mode: waveform_model(ydata, mode) for mode in (False, True)}
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    expected = list(itertools.product(('normal', 'sync'), ('0', '1'), ('0', '127'),
                                     ('0', '1'), ('0', '1')))
    if [tuple(row[:5]) for row in cases] != expected:
        raise ValueError('Incomplete consecutive voice/master stream matrix')
    totals = new_totals()
    totals['master_blocks'] = totals['nonzero_output_words'] = 0
    for *identity, stream in cases:
        output = Output(program, consumer() if consumer is not None else None)
        validate_stream(stream, xdata, ydata, tables, tuple(identity), totals,
                        after_block=output)
        if output.blocks != 8 or output.nonzero == 0:
            raise ValueError('Original master output must consume every nonempty voice stream')
        totals['master_blocks'] += output.blocks
        totals['nonzero_output_words'] += output.nonzero
    if totals['blocks'] != 256 or totals['dependent_blocks'] != 224:
        raise ValueError('Missing persistent voice/master histories')
    validate_coverage(text, reference['coverage'])
    return totals


def negative_controls(text, xdata, ydata, program, reference):
    labels = ('MASTER_COPY', 'MASTER_MIX', 'MASTER_OUTPUT', 'MASTER_HISTORY')
    for label in labels:
        def corrupt(match):
            return match[1] + format(int(match[2], 16) ^ 1, 'x')
        changed, count = re.subn(r'^(' + label + r' )([0-9a-f]+)', corrupt,
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing native word for oracle negative control')
        try:
            validate(changed, xdata, ydata, program, reference)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong independent negative-control failure') from error
        else:
            raise ValueError('Independent validation accepted a corrupted native word')
    return len(labels)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original voice/master: private firmware absent')
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
        raise ValueError(f'Original voice/master native failure: {diagnostics}')
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program, reference)
    controls = negative_controls(text, xdata, ydata, program, reference)
    report.write_text(json.dumps({'passed': True, 'streams': 32, **totals,
                                  'oracle_negative_controls': controls,
                                  'master_output_words': totals['master_blocks'] * 32,
                                  'fuzz_execution': False, 'physical_audio': False,
                                  'scope': 'Own caller composition of unchanged original voice, '
                                  'copy, replacement mix and master routines. Original histories '
                                  'persist; no boot, scheduler, DMA, PCM or hardware claim.'},
                                 indent=2) + '\n')
    print('Verified 256 native original voice/copy/mix/master blocks and 8192 independent '
          'master words; full boot, MIDI allocation and serial audio remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
