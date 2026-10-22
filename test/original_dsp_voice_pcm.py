# SPDX-License-Identifier: GPL-3.0-only
"""Compare actual DMA/ESAI-decoded original voice samples with independent models."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_dsp_voice_master import validate as validate_master

ROOT = Path(__file__).resolve().parents[1]


class Pcm:
    def __call__(self, expected, text, block):
        def rows(label):
            return [list(map(lambda value: int(value, 16), row.split()))
                    for row in re.findall('^' + label + r' ([^\n]+)$', text, re.M)]

        start = 0x660 + (block % 2) * 32
        transfers = [[index, start + index, 0xffff80, sample]
                     for index, sample in enumerate(expected)]
        if rows('PCM_DMA') != transfers:
            raise ValueError('PCM_DMA differs from independent master words/linear addresses')
        frames = []
        # TCCR=$640301: divide HCKT by4, each HCKT period has two supplied edges.
        # A real zero prime precedes data. Active-low FS locks the receiver to
        # the next slot; these are reference edge ordinals, not measured time.
        bit_period = 8
        first_data = 267
        for index in range(16):
            first = first_data + (block * 16 + index) * 64 * bit_period
            last = first + 63 * bit_period
            frames.append([index, first, last, bit_period, *expected[2 * index:2 * index + 2]])
        if rows('PCM_FRAME') != frames:
            raise ValueError('PCM_FRAME differs from independent payload, slot order or clock')
        if rows('PCM_RESTORE') != ([[1]] if block == 0 else []):
            raise ValueError('Missing held-DMA-word/peripheral receiver restore')


def validate(text, xdata, ydata, program, reference):
    totals = validate_master(text, xdata, ydata, program, reference, consumer=Pcm)
    counts = {label: len(re.findall('^' + label + ' ', text, re.M))
              for label in ('PCM_DMA', 'PCM_FRAME', 'PCM_RESTORE')}
    if counts != {'PCM_DMA': 8192, 'PCM_FRAME': 4096, 'PCM_RESTORE': 32}:
        raise ValueError('Incomplete original voice/DMA/ESAI matrix')
    return totals | {'dma_words': counts['PCM_DMA'], 'decoded_frames': counts['PCM_FRAME'],
                     'held_word_restores': counts['PCM_RESTORE']}


def negative_controls(text, xdata, ydata, program, reference):
    for label in ('PCM_DMA', 'PCM_FRAME'):
        def corrupt(match):
            words = match[1].split()
            words[-1] = format(int(words[-1], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing serial oracle negative-control word')
        try:
            validate(changed, xdata, ydata, program, reference)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong serial oracle negative-control failure') from error
        else:
            raise ValueError('Serial oracle accepted a corrupted native word')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print('SKIP original voice PCM: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=280, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in
                               ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError(f'Original voice PCM native failure: {diagnostics}')
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program, reference)
    negative_controls(text, xdata, ydata, program, reference)
    report.write_text(json.dumps({'passed': True, 'streams': 32, **totals,
                                  'serial_negative_controls': 2, 'fuzz_execution': False,
                                  'physical_audio': False,
                                  'scope': 'Unchanged original voice/copy/mix/master to finite DMA '
                                  'and ESAI_1 pins/PCM. Own native setup, paused core, explicit '
                                  'HCKT/grants and receiver polarity. No original boot, live '
                                  'MIDI, automatic DMA scheduling or physical audio claim.'},
                                 indent=2) + '\n')
    print('Verified 8192 actual DMA words and 4096 pin-decoded original voice frames; '
          '32 held-word restores pass. Whole boot and physical timing remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
