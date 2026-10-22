# SPDX-License-Identifier: GPL-3.0-only
"""Original MIDI release, rendering, collection and subsequent repeating audio halves."""

import itertools
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

from dsp_voice_model import FULL_MASK, H, W, WORD_MASK, Voice, qword, signed, waveform_model
from dsp_voice_validation import new_totals, validate_block
from original_dsp_foreground import row
from original_dsp_repeat import Pipeline, SOUND, negative_controls as ring_controls
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
BLOCKS = 10
REQUEST_BLOCK, REMOVAL_BLOCK = 2, 5
RELEASE, RETIRE, IDLE = 0x160f, 0x1609, 0x160e


class ReleaseVoice(Voice):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.released = False

    def request_release(self):
        if any(self.sound[base] & 7 for base in (196, 208, 220, 232)):
            raise ValueError('Release scope requires the original ADSR modes')
        self.released = True
        for at in range(0, 24, 6):
            self.envelopes[at] = RELEASE

    def finish_controls(self):
        if not self.released:
            return super().finish_controls()
        self.advance_lfos()
        last = old = coefficient = address = 0
        for index, base in enumerate((196, 208, 220, 232)):
            at = index * 6
            callback, raw = self.envelopes[at:at + 2]
            if callback == RELEASE:
                old = signed(raw)
                address = 0x16c8 - self.sound[base + 9]
                coefficient = signed(self.data[address])
                last = old * W - 2 * coefficient * old
                self.envelopes[at + 1] = qword(last) & WORD_MASK
                if last <= 0:
                    self.envelopes[at] = RETIRE
            elif callback == RETIRE:
                self.envelopes[at:at + 2] = [IDLE, 0]
            elif callback != IDLE:
                raise ValueError('Unexpected independent released envelope callback')
        if self.envelopes[18] != RELEASE:
            raise ValueError('Factory fourth release must remain active in this finite window')
        self.block += 1
        return [0x16c8 * W, last & FULL_MASK, 0xc54b, address, 0xc538, 0xc54a,
                0xc500, RELEASE, old & WORD_MASK, 0x16c8, coefficient & WORD_MASK]


class ReleasePipeline(Pipeline):
    def __init__(self, varied, twos, program, serial):
        super().__init__(varied, twos, program)
        self.serial_route = serial
        self.retired = self.sent = False
        self.active = True
        self.payload = bytearray()
        self.empty_blocks = 0

    def before(self, voice, text, block):
        self.active = not self.retired
        super().before(voice, text, block)

    def entry_state(self, voice):
        velocity = 0 if self.serial_route or not self.sent else 64 << 16
        flags = (0x80 if self.pending else 0) | self.extra_flags
        return [0, 0, velocity, 0 if self.retired else 0xc500, 0, flags, 0,
                *voice.envelopes, *([0] * 24)]

    def input(self, voice, text, check):
        status, velocity = (0x90, 0) if self.serial_route else (0x80, 64)
        if row('IDLE_BYTES', text) != [status, 0, velocity]:
            raise ValueError('IDLE_BYTES differs from the declared original release')
        self.cursor += 3
        loops = re.findall(r'^IDLE_BEGIN ([^\n]+)\n([\s\S]*?)(?=^IDLE_BEGIN |\Z)', text, re.M)
        if [int(ordinal, 16) for ordinal, _ in loops] != [0, 1]:
            raise ValueError('IDLE_BEGIN release and subsequent empty poll must execute')
        ram = row('FOREGROUND_RAM', text)
        expected_ram = ram[:10] + [self.cursor, self.cursor] + ram[12:]
        for index, (_, loop) in enumerate(loops):
            stages = [[int(value, 16) for value in raw.split()]
                      for raw in re.findall(r'^IDLE_STAGE ([^\n]+)$', loop, re.M)]
            if stages != [[0x198, 1], *[[0xc53, 2]] * (4 if index == 0 else 1), [0xc5d, 2], [0x2407, 2]]:
                raise ValueError('IDLE_STAGE release must remain behind the equal-half gate')
            check('IDLE_MIDI', [self.cursor, self.cursor, status << 16, 0, velocity << 16,
                                SOUND + 404, self.target, 0x80 if self.pending else 0,
                                self.current, 0, 1], loop)
            check('IDLE_RAM', expected_ram, loop)
            state = row('IDLE_END', loop)
            if (len(state) != 5 or state[:4] != [0x198, 1, ram[12] * W, 0]
                    or state[4] & ((1 << 21) | 0xc00 | 4) != (int(self.twos) << 21) | 4):
                raise ValueError('IDLE_END release phase/zero flag/frame differs')
        voice.request_release()
        self.sent = True
        check('RELEASE_AFTER_INPUT', self.entry_state(voice), text)

    def after(self, voice, text, check):
        removed = self.active and voice.envelopes[6] == IDLE
        if removed:
            self.retired, self.extra_flags, self.service_return = True, 2, 0x6ec
        else:
            self.extra_flags = 0
            self.service_return = 0x6d4 if self.active else 0x6d3
        self.scheduler(voice, text, check)
        check('RELEASE_RENDERED', [int(self.active)], text)
        check('RELEASE_AFTER_BODY', self.entry_state(voice), text)
        self.after_input(voice, text, check)
        self.serial(text, check)

    def after_input(self, voice, text, check):
        if self.block == REQUEST_BLOCK:
            self.input(voice, text, check)
        elif re.search(r'^(IDLE_|RELEASE_AFTER_INPUT)', text, re.M):
            raise ValueError('Unexpected release input outside its original wait')

    def serial(self, text, check):
        super().serial(text, check)
        for raw in self.halves[self.block % 2]:
            self.payload.extend(struct.pack('<i', signed(raw) * 256))


def validate_context(stream, xdata, ydata, program, identity, totals,
                     pipeline_type=ReleasePipeline, block_count=BLOCKS):
    if [ydata[0x1e25 + field] & 255 for field in (129, 145)] != [0, 0]:
        raise ValueError('Changed original factory Effects Off mix profile')
    kind, mode, color, varied, routing = identity
    high, twos, varied, serial = kind == 'sync', mode == '1', varied == '1', routing == '1'
    voice = ReleaseVoice(xdata, ydata, waveform_model(ydata, twos), high, twos, int(color),
                         varied, serial, sound_base=SOUND)
    pipeline = pipeline_type(varied, twos, program, serial)

    def check(label, expected, text):
        if row(label, text) != expected:
            raise ValueError(f'{label} differs in original release context {identity}, block {pipeline.block}')

    prefix = stream.split('BLOCK ')[0]
    check('CACHE', voice.gains, prefix)
    check('PARAM_CACHE', [word & WORD_MASK for word in voice.parameters], prefix)
    check('SETUP', [1, int(color) * 65536, 0, 1392, 1, 1392, 1], prefix)
    check('REPEAT_AUTOMATIC_REJECTED', [1], prefix)
    check('FOREGROUND_INPUT', [0xb0, 7, 127, 1, 0, 7, 127] if varied else [0xb0, 7, 0], prefix)
    blocks = re.findall(r'^BLOCK (\d+)\n([\s\S]*?)(?=^BLOCK |\Z)', stream, re.M)
    if [int(number) for number, _ in blocks] != list(range(block_count)):
        raise ValueError('Incomplete original release foreground stream')
    for raw, text in blocks:
        block = int(raw)
        pipeline.before(voice, text, block)
        if pipeline.active:
            validate_block(voice, raw, text, xdata, ydata, check, twos, color, high,
                           varied, serial, totals, identity)
        else:
            forbidden = ('WHOLE_RETURN', 'OUTPUT_BUS', 'NOISE_INPUT', 'COEFFICIENTS', 'OUTPUT_DISPATCH')
            if any(re.search('^' + label + ' ', text, re.M) for label in forbidden):
                raise ValueError('Removed entry must not execute another voice')
            check('CALLBACKS_BEFORE', [xdata[0xc500] & (H - 1), 4856, 4856, 4820, 1736], text)
            check('HISTORY_INIT', voice.history_words(), text)
            pipeline.empty_blocks += 1
        pipeline.after(voice, text, check)
    if (pipeline.empty_blocks != 4 or not pipeline.retired
            or voice.block != block_count - 4 or not pipeline.nonzero):
        raise ValueError('Original release must retire and retain four subsequent empty blocks')
    return pipeline


def cases(text):
    return re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)'
                      r'(?=CASE |Entry-streams visited PCs:)', text)


def validate(text, xdata, ydata, program):
    streams = cases(text)
    expected = [(kind, mode, '127', str(int(kind == 'sync')), routing)
                for kind, mode, routing in itertools.product(('normal', 'sync'), ('0', '1'), ('0', '1'))]
    if [tuple(item[:5]) for item in streams] != expected:
        raise ValueError('Incomplete original release/ring matrix')
    totals = new_totals()
    for *identity, stream in streams:
        validate_context(stream, xdata, ydata, program, tuple(identity), totals)
    if (totals['blocks'], totals['dependent_blocks']) != (48, 40):
        raise ValueError('Incomplete original release voice histories')
    return totals


def expected_payload(text, xdata, ydata, program):
    streams = cases(text)
    identity = ('normal', '0', '127', '0', '0')
    if len(streams) != 1 or tuple(streams[0][:5]) != identity:
        raise ValueError('Direct release audio must select one complete original stream')
    pipeline = validate_context(streams[0][5], xdata, ydata, program, identity, new_totals())
    if len(pipeline.payload) != BLOCKS * 16 * 8:
        raise ValueError('Direct original release must retain every predicted serial frame')
    return bytes(pipeline.payload)


def negative_controls(text, xdata, ydata, program, validator=validate):
    for label in ('RELEASE_AFTER_BODY', 'RELEASE_AFTER_INPUT', 'RELEASE_RENDERED',
                  'IDLE_MIDI', 'IDLE_STAGE', 'IDLE_BYTES'):
        def corrupt(match):
            words = match[1].split()
            at = 7 if label == 'RELEASE_AFTER_INPUT' else -1
            words[at] = format(int(words[at], 16) ^ 1, 'x')
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing original release/ring negative control')
        try:
            validator(changed, xdata, ydata, program)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong original release/ring negative-control failure') from error
        else:
            raise ValueError('Independent release/ring oracle accepted corruption')
    ring_controls(text, xdata, ydata, program, validator=validator)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original release ring: private firmware absent')
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
        raise ValueError('Original release/ring native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, xdata, ydata, program)
    negative_controls(text, xdata, ydata, program)
    report.write_text(json.dumps({'passed': True, 'streams': 8, **totals, 'foreground_blocks': 80,
                                  'empty_blocks': 32, 'retirements': 8, 'decoded_frames': 1280,
                                  'oracle_negative_controls': 10, 'fuzz_execution': False,
                                  'physical_traffic': False,
                                  'scope': 'Raw original MIDI release during DMA wait, subsequent '
                                  'native envelopes/collection and enabled DMA/ESAI output. '
                                  'Declared held entry/RAM and untimed reference interleave; '
                                  'no full boot, realtime, live host MIDI or allocation claim.'}, indent=2) + '\n')
    print('Verified original MIDI release through 80 foreground blocks, eight retirements and 1280 PCM frames.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
