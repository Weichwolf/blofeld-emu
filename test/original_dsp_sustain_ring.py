# SPDX-License-Identifier: GPL-3.0-only
"""Original sustained release through foreground, retirement and real serial audio."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import W
from dsp_voice_validation import new_totals
from original_dsp_foreground import row
from original_dsp_release_ring import ReleasePipeline, cases, validate_context
from original_dsp_release_ring import negative_controls as release_controls
from original_dsp_repeat import SOUND
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
BLOCKS = 12
RELEASE_BLOCK = 4


class SustainPipeline(ReleasePipeline):
    def __init__(self, varied, twos, program, serial):
        super().__init__(varied, twos, program, serial)
        self.scalar, self.pedal, self.latch = 127 << 16, 0, 0
        self.status, self.first, self.second = 0xb0, 7, 127 if varied else 0
        self.waiting_first = True
        self.idle_polls = 0

    def entry_state(self, voice):
        state = super().entry_state(voice)
        state[2] = 0  # Actual deferred callbacks use the pedal-off controller's velocity.
        return state

    def sustain_state(self):
        return [self.scalar, self.pedal, self.latch, *([0] * 7)]

    def chunks(self):
        if self.block == 0:
            return ((0xb0, 64, 127 if self.serial_route else 1),)
        if self.block == 2:
            return ((0x90, 0, 0) if self.serial_route else (0x80, 0, 64),)
        if self.block == RELEASE_BLOCK:
            return ((0xb0, 64), (0,))
        return ()

    def deliver(self, chunk, voice):
        for byte in chunk:
            self.cursor = 0x2f80 + ((self.cursor - 0x2f80 + 1) % 64)
            if byte >= 128:
                self.status, self.waiting_first = byte, True
            elif self.waiting_first:
                self.first, self.waiting_first = byte, False
            else:
                self.second, self.waiting_first = byte, True
                if self.status == 0xb0 and self.first == 64:
                    self.scalar, self.pedal = self.second << 16, int(self.second != 0)
                    if not self.pedal:
                        # The scan and single-part dispatch share the saved-status word.
                        # Each of eight groups saves its start; the caller restores it.
                        saved_status = self.status
                        for group in range(8):
                            saved_status = group * 16
                            if group == 0 and self.latch:
                                self.latch = 0
                                self.first = 0
                                voice.request_release()
                                self.sent = True
                        self.status = saved_status
                elif self.status in (0x80, 0x90) and self.first == 0:
                    if not self.pedal:
                        raise ValueError('IDLE_MIDI note must remain sustained before pedal release')
                    self.latch = 1
                else:
                    raise ValueError('IDLE_MIDI unexpected independent sustain input')

    def idle_loop(self, text, chunk, ordinal, ram, voice, check):
        if row('IDLE_BEGIN', text) != [ordinal]:
            raise ValueError('IDLE_BEGIN sustain polls out of order')
        stages = [[int(value, 16) for value in raw.split()]
                  for raw in re.findall(r'^IDLE_STAGE ([^\n]+)$', text, re.M)]
        expected = [[0x198, 1], *[[0xc53, 2]] * (len(chunk) + 1), [0xc5d, 2], [0x2407, 2]]
        if stages != expected:
            raise ValueError('IDLE_STAGE sustain must retain the equal-half wait gate')
        self.deliver(chunk, voice)
        check('IDLE_MIDI', [self.cursor, self.cursor, self.status << 16, self.first << 16,
                            self.second << 16, SOUND + 404, self.target,
                            0x80 if self.pending else 0, self.current, 0, 1], text)
        check('IDLE_RAM', ram[:10] + [self.cursor, self.cursor] + ram[12:], text)
        state = row('IDLE_END', text)
        if (len(state) != 5 or state[:4] != [0x198, 1, ram[12] * W, 0]
                or state[4] & ((1 << 21) | 0xc00 | 4) != (int(self.twos) << 21) | 4):
            raise ValueError('IDLE_END sustain must preserve phase, caller frame and modes')
        self.idle_polls += 1

    def after_input(self, voice, text, check):
        check('SUSTAIN_AFTER_BODY', self.sustain_state(), text)
        chunks = self.chunks()
        parts = re.findall(r'^IDLE_BYTES([^\n]*)\n([\s\S]*?)(?=^IDLE_BYTES|\Z)', text, re.M)
        if [tuple(int(value, 16) for value in raw.split()) for raw, _ in parts] != list(chunks):
            raise ValueError('IDLE_BYTES differs from pedal/deferred-note/fragmented-off input')
        ordinal = 0
        ram = row('FOREGROUND_RAM', text)
        for chunk, (_, part) in zip(chunks, parts, strict=True):
            loops = re.findall(r'(^IDLE_BEGIN [^\n]+\n[\s\S]*?)(?=^IDLE_BEGIN |\Z)', part, re.M)
            if len(loops) != 2:
                raise ValueError('IDLE_BEGIN sustain needs actual delivery and empty polling')
            for index, loop in enumerate(loops):
                self.idle_loop(loop, chunk if index == 0 else (), ordinal, ram, voice, check)
                ordinal += 1
        if chunks:
            check('SUSTAIN_AFTER_INPUT', self.sustain_state(), text)
            check('RELEASE_AFTER_INPUT', self.entry_state(voice), text)
        elif re.search(r'^(IDLE_|RELEASE_AFTER_INPUT|SUSTAIN_AFTER_INPUT)', text, re.M):
            raise ValueError('IDLE_BYTES unexpected sustain input outside its own wait')


def validate(text, xdata, ydata, program):
    streams = cases(text)
    expected = [(kind, mode, '127', str(int(kind == 'sync')), routing)
                for kind, mode, routing in itertools.product(('normal', 'sync'), ('0', '1'), ('0', '1'))]
    if [tuple(item[:5]) for item in streams] != expected:
        raise ValueError('Incomplete original sustain/ring matrix')
    totals = new_totals()
    for *identity, stream in streams:
        pipeline = validate_context(stream, xdata, ydata, program, tuple(identity), totals,
                                    pipeline_type=SustainPipeline, block_count=BLOCKS)
        if pipeline.idle_polls != 8:
            raise ValueError('IDLE_BEGIN incomplete sustained foreground input')
    if (totals['blocks'], totals['dependent_blocks']) != (64, 56):
        raise ValueError('Incomplete original sustained voice histories')
    return totals


def expected_payload(text, xdata, ydata, program):
    streams = cases(text)
    identity = ('normal', '0', '127', '0', '0')
    if len(streams) != 1 or tuple(streams[0][:5]) != identity:
        raise ValueError('Direct sustain audio must select one complete original stream')
    pipeline = validate_context(streams[0][5], xdata, ydata, program, identity, new_totals(),
                                pipeline_type=SustainPipeline, block_count=BLOCKS)
    if pipeline.idle_polls != 8 or len(pipeline.payload) != BLOCKS * 16 * 8:
        raise ValueError('Direct sustain audio lost input polling or predicted serial frames')
    return bytes(pipeline.payload)


def negative_controls(text, xdata, ydata, program, validator=validate):
    for label in ('SUSTAIN_AFTER_BODY', 'SUSTAIN_AFTER_INPUT'):
        def corrupt(match):
            words = match[1].split()
            words[-1] = format(int(words[-1], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing sustain/ring oracle control')
        try:
            validator(changed, xdata, ydata, program)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong sustain/ring oracle-control failure') from error
        else:
            raise ValueError('Independent sustain/ring oracle accepted corruption')
    release_controls(text, xdata, ydata, program, validator=validator)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original sustain ring: private firmware absent')
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
                               ('AddressSanitizer', 'LeakSanitizer', 'ThreadSanitizer', 'runtime error:')):
        raise ValueError('Original sustain/ring native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program)
    negative_controls(text, xdata, ydata, program)
    report.write_text(json.dumps({'passed': True, 'streams': 8, **totals, 'foreground_blocks': 96,
                                  'empty_blocks': 32, 'retirements': 8, 'decoded_frames': 1536,
                                  'idle_polls': 64, 'oracle_negative_controls': 12,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Raw CC64 and release during the original DMA wait, '
                                  'deferred native callbacks, full voice/collection and enabled '
                                  'DMA/ESAI output. Declared held entry/RAM and untimed interleave; '
                                  'no full boot, realtime, live host MIDI or allocation claim.'}, indent=2) + '\n')
    print('Verified sustained original MIDI through 96 foreground bodies and 1536 PCM frames.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
