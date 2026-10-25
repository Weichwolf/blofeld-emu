# SPDX-License-Identifier: GPL-3.0-only
"""Original persistent foreground feeds both original DMA-controlled pin outputs."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_audio_dma import bmc_word
from original_dsp_foreground import row
from original_dsp_repeat import Pipeline, negative_controls
from original_dsp_repeat import validate as validate_repeat
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


class DualPipeline(Pipeline):
    def __init__(self, varied, twos, program):
        super().__init__(varied, twos, program)
        self.dax_words = [0, 0]  # Real XADR zero priming precedes all rendered data.

    def serial(self, text, check):
        super().serial(text, check)
        expected = self.halves[self.block % 2]
        actual = [[int(word, 16) for word in line.split()]
                  for line in re.findall(r'^DUAL_DAX_DMA ([^\n]+)$', text, re.M)]
        transfers = [[i, 0x660 + (self.block % 2) * 32 + i, 0xffffd2, value]
                     for i, value in enumerate(expected)]
        if actual != transfers:
            raise ValueError('DUAL_DAX_DMA differs from the independent previous-mix ring')
        self.dax_words.extend(expected)
        if self.block != 5:
            if any(label in text for label in ('DUAL_BMC', 'DUAL_SETUP', 'DUAL_FINAL')):
                raise ValueError('Premature DAX source finalization')
            return
        if row('DUAL_SETUP', text) != [4100, 0xaab604, 0xa8b600]:
            raise ValueError('DUAL_SETUP original control/REP/NOP/BSET/RTS scope differs')
        if row('DUAL_FINAL', text) != [192, 97, 12416, 49663]:
            raise ValueError('DUAL_FINAL did not retain every final DAX subframe/unit')
        streams = re.findall(r'^DUAL_BMC ([01]+)$', text, re.M)
        if len(streams) != 1 or len(streams[0]) != 12416:
            raise ValueError('DUAL_BMC missing/duplicated/truncated pin stream')
        for index, value in enumerate(self.dax_words):
            prefix = '00011011' if index % 2 else ('00011101' if index else '00010111')
            try:
                bmc_word(streams[0][index * 64:(index + 1) * 64], value, prefix)
            except ValueError as error:
                raise ValueError('DUAL_BMC original waveform/metadata/parity differs') from error


def validate(text, xdata, ydata, program):
    return validate_repeat(text, xdata, ydata, program, pipeline_type=DualPipeline)


def dual_controls(text, xdata, ydata, program):
    for label in ('DUAL_DAX_DMA', 'DUAL_SETUP', 'DUAL_FINAL', 'DUAL_BMC'):
        def corrupt(match):
            fields = match[1].split()
            if label == 'DUAL_BMC':
                bits = list(fields[0])
                bits[8] = '1' if bits[8] == '0' else '0'
                fields[0] = ''.join(bits)
            else:
                fields[-1] = format(int(fields[-1], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(fields)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing directed dual-output oracle control')
        try:
            validate(changed, xdata, ydata, program)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong dual-output negative-control failure') from error
        else:
            raise ValueError('Dual-output oracle accepted a fixed corrupted trace')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not binary.is_file():
        raise ValueError('Original dual-output native executable missing')
    if not firmware.is_file():
        print('SKIP original dual output: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    program = {}
    xdata, ydata = original_records(firmware, reference, program)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    trace.with_suffix('.json').unlink(missing_ok=True)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    run = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                         capture_output=True, text=True, timeout=800, check=False)
    output = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Original voice streams executed; independent numerical verification is separate.\n'
    if run.returncode or output != allowed:
        raise ValueError('Original foreground dual-output execution failed: ' + output)
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program)
    negative_controls(text, xdata, ydata, program, validator=validate)
    dual_controls(text, xdata, ydata, program)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != sha for path, sha in frozen.items()):
        raise ValueError('A native dual-output input changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(
        passed=True, streams=8, **totals, dma_words_per_output=1536, pcm_frames=768,
        dax_frames=776, original_suffix_instructions_per_stream=4100,
        held_word_restores=8, fixed_oracle_corruptions=8,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, physical_traffic=False, fuzz_execution=False,
        automatic_core_dma_timing=False,
        scope='Original persistent foreground/render/mix/master fills the actual repeating '
              'DMA0/3 ring. Original enable/REP/NOP/BSET/RTS suffix with own preloads. '
              'Untimed core admissions alternate with paused-core HCKT/ACI/grant phases; '
              'final ACI-only edges complete the actual DAX tail. Effects Off, held entry, '
              'declared caller/RAM/clock/pin conditions. No normal boot, NoteOn allocation, '
              'physical clocks/arbitration, live host audio or hardware equivalence.'), indent=2) + '\n')
    print('Verified48 original foreground blocks and both DMA-controlled serial outputs.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
