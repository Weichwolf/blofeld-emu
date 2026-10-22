# SPDX-License-Identifier: GPL-3.0-only
"""Track original idle MIDI parsing through later foreground, DMA and serial samples."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import W, waveform_model
from dsp_voice_validation import new_totals, validate_stream
from original_dsp_foreground import row
from original_dsp_repeat import Pipeline, SOUND, negative_controls as ring_negative_controls
from original_dsp_voice import original_records
from original_dsp_voice_midi import Midi

ROOT = Path(__file__).resolve().parents[1]
BLOCKS = 10


def chunks(block):
    if block == 0:
        return [(7, 64)]
    if block == 1:
        return [(7,), (0,)]
    if block == 2:
        return [(0xb1, 7, 127), (0xb0, 7, 1)]
    if block == 3:
        return [(7, 127, 7, 32)]
    if block == 4:
        return [tuple(byte for value in range(24) for byte in (7, value))]
    return [()]


class IdlePipeline(Pipeline):
    def __init__(self, varied, twos, program):
        super().__init__(varied, twos, program)
        self.midi = Midi()
        self.midi.deliver((0xb0, 7, 127, 1, 0, 7, 127) if varied else (0xb0, 7, 0))
        self.idle_polls = 0

    def idle_loop(self, loop, chunk, ordinal, ram, check):
        stages = [[int(value, 16) for value in values.split()]
                  for values in re.findall(r'^IDLE_STAGE ([^\n]+)$', loop, re.M)]
        expected = [[0x198, 1], *[[0xc53, 2]] * (len(chunk) + 1), [0xc5d, 2], [0x2407, 2]]
        if stages != expected:
            raise ValueError('IDLE_STAGE differs from the actual equal-half DMA gate')
        state = row('IDLE_END', loop)
        if (len(state) != 5 or state[:4] != [0x198, 1, ram[12] * W, 0]
                or state[4] & ((1 << 21) | 0xc00 | 4) != (int(self.twos) << 21) | 4):
            raise ValueError('IDLE_END compare, zero flag or native frame differs')
        midi = self.midi.deliver(chunk)
        midi[-1] = 1  # The real foreground retains its own native caller frame.
        check('IDLE_MIDI', midi, loop)
        expected_ram = list(ram)
        expected_ram[2:5] = [midi[7], midi[5], midi[6]]
        expected_ram[9:12] = [midi[8], midi[0], midi[1]]
        check('IDLE_RAM', expected_ram, loop)
        if row('IDLE_BEGIN', loop) != [ordinal]:
            raise ValueError('IDLE_BEGIN order differs')
        self.idle_polls += 1

    def after(self, voice, text, check):
        self.scheduler(voice, text, check)
        self.midi.volume, self.midi.pending = self.current, self.pending
        parts = re.findall(r'^IDLE_BYTES([^\n]*)\n([\s\S]*?)(?=^IDLE_BYTES|\Z)', text, re.M)
        expected = chunks(self.block)
        if [tuple(int(value, 16) for value in raw.split()) for raw, _ in parts] != expected:
            raise ValueError('IDLE_BYTES differs from the independent transport schedule')
        ram = row('FOREGROUND_RAM', text)
        ordinal = 0
        for (_, part), chunk in zip(parts, expected, strict=True):
            loops = re.findall(r'(^IDLE_BEGIN [^\n]+\n[\s\S]*?)(?=^IDLE_BEGIN |\Z)', part, re.M)
            if len(loops) != 2:
                raise ValueError('IDLE_BEGIN must include both delivery and empty polls')
            for index, loop in enumerate(loops):
                self.idle_loop(loop, chunk if index == 0 else (), ordinal, ram, check)
                ordinal += 1
        self.target, self.pending, self.cursor = self.midi.target, self.midi.pending, self.midi.cursor
        self.serial(text, check)


def validate(text, xdata, ydata, program):
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                       r'(?=CASE |Entry-streams visited PCs:)', text)
    expected = [(kind, mode, '127', str(int(kind == 'sync')), routing)
                for kind, mode, routing in itertools.product(('normal', 'sync'), ('0', '1'), ('0', '1'))]
    if [tuple(case[:5]) for case in cases] != expected:
        raise ValueError('Incomplete idle MIDI/foreground/ring matrix')
    if [ydata[0x1e25 + index] & 255 for index in (129, 145)] != [0, 0]:
        raise ValueError('Changed declared original FX Mix0/0 profile')
    totals = new_totals()
    totals['idle_polls'] = totals['nonzero_serial_words'] = 0
    for *identity, stream in cases:
        pipeline = IdlePipeline(identity[3] == '1', identity[1] == '1', program)
        wanted = [0xb0, 7, 127, 1, 0, 7, 127] if pipeline.varied else [0xb0, 7, 0]
        if row('FOREGROUND_INPUT', stream) != wanted or row('REPEAT_AUTOMATIC_REJECTED', stream) != [1]:
            raise ValueError('Initial original MIDI or automatic DMA refusal differs')
        validate_stream(stream, xdata, ydata, {mode: waveform_model(ydata, mode) for mode in (False, True)},
                        tuple(identity), totals, block_count=BLOCKS, before_block=pipeline.before,
                        after_block=pipeline.after, sound_base=SOUND)
        if pipeline.cursor != 0x2f80 + (5 if pipeline.varied else 1) or not pipeline.nonzero:
            raise ValueError('Idle input must wrap its ring and retain nonzero original output')
        totals['idle_polls'] += pipeline.idle_polls
        totals['nonzero_serial_words'] += pipeline.nonzero
    if (totals['blocks'], totals['dependent_blocks'], totals['idle_polls']) != (80, 72, 192):
        raise ValueError('Incomplete idle MIDI and persistent original voice histories')
    return totals


def negative_controls(text, xdata, ydata, program, validator=validate):
    for label in ('IDLE_BYTES', 'IDLE_STAGE', 'IDLE_RAM', 'IDLE_END', 'IDLE_MIDI'):
        def corrupt(match):
            words = match[1].split()
            index = 2 if label == 'IDLE_END' else -1
            words[index] = format(int(words[index], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing idle MIDI negative control')
        try:
            validator(changed, xdata, ydata, program)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong idle MIDI negative-control failure') from error
        else:
            raise ValueError('Independent idle MIDI oracle accepted corruption')
    ring_negative_controls(text, xdata, ydata, program, validator=validator)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP idle original MIDI ring: private firmware absent')
        return 77
    program = {}
    xdata, ydata = original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()), program)
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
        raise ValueError('Original idle MIDI native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program)
    negative_controls(text, xdata, ydata, program)
    report.write_text(json.dumps({'passed': True, 'streams': 8, **totals, 'dma_words': 2560,
                                  'decoded_frames': 1280, 'input_ring_wraps': 8,
                                  'oracle_negative_controls': 9, 'fuzz_execution': False,
                                  'physical_traffic': False,
                                  'scope': 'Raw CC bytes enter the original ring while the '
                                  'foreground waits at the equal-half gate. Original parser, '
                                  'smoothing, voice, master and repeating DMA retain state. '
                                  'Untimed interleave and declared held entry/RAM; no full '
                                  'boot, physical timing, live host MIDI or NoteOn claim.'}, indent=2) + '\n')
    print('Verified 192 idle MIDI polls, 80 original foreground blocks and 1280 ring PCM frames.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
