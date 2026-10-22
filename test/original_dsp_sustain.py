# SPDX-License-Identifier: GPL-3.0-only
"""Independent CC64 latch, deferred release and held-entry retirement expectations."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_note_release import CHANNELS, DECAY, IDLE, Model, NOTES, RELEASE, RETIRE
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
CHORD = (127, 112, 65, 64, 63, 16, 15, 0, 64)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def matrix():
    for modes in range(16):
        if modes == 0:
            for level, note in itertools.product((0, 1, 63, 64, 127), range(128)):
                yield modes, level, 0, note, 0, 0, 0, 0
        for level in (1, 127):
            for channel, zero, delivery in itertools.product((0, 15), (0, 1), (1, 2)):
                yield modes, level, channel, 64, zero, delivery, 0, 0
            yield modes, level, 15, 64, 0, 0, 1, 0
            yield modes, level, 0, 64, 0, 0, 0, 1


class Sustain(Model):
    def __init__(self, identity, coefficient):
        modes, self.level, _channel, self.note, self.note_zero, self.delivery, filtered, self.chord = identity
        super().__init__(modes, self.note, True, filtered, coefficient)
        self.filtered = bool(filtered)
        self.scalar, self.pedal = 127 << 16, 0
        self.latches = [0] * 8
        self.count = [0] * 13
        self.control_count = [0] * 3
        self.calls, self.pedal_rows, self.entry_rows = [], [], []
        self.deferred = self.drained = self.immediate = self.matches_total = 0

    def state(self, stage):
        self.pedal_rows.append([stage, self.scalar, self.pedal, *self.latches])
        self.entry_rows.append(self.row(stage))

    def invoke(self, note, status, velocity, stage):
        self.calls.append([stage, note << 16, status << 16, velocity << 16,
                           self.scalar, self.pedal, *self.latches])
        self.matches = [slot for slot in self.order if NOTES[slot] == note and CHANNELS[slot] == 0]
        self.zero = velocity == 0
        self.release()
        self.control_count[2] += 1
        self.count[6] += 1
        self.count[7] += len(self.matches)
        self.matches_total += len(self.matches)

    def controller(self, value, stage):
        if not self.filtered:
            self.control_count[0] += 1
            self.scalar, self.pedal = value << 16, int(value != 0)
            if value == 0:
                self.control_count[1] += 1
                # Clear a complete packed word before dispatching any of its bits.
                for group, word in enumerate(self.latches):
                    self.latches[group] = 0
                    for bit in range(16):
                        if word & (1 << bit):
                            self.invoke(group * 16 + bit, 0x80, 0, stage)
                            self.drained += 1
        self.state(stage)

    def note_event(self, note):
        if not self.filtered:
            self.count[0] += 1
            if self.pedal:
                self.deferred += 1
            else:
                self.count[1] += self.note_zero
                self.invoke(note, 0x90 if self.note_zero else 0x80,
                            0 if self.note_zero else 64, 3)
                self.immediate += 1
            self.latches[note // 16] |= 1 << (note % 16)
        self.state(3)

    def sequence(self):
        self.state(0)
        self.controller(self.level, 1)
        for note in CHORD if self.chord else (self.note,):
            if self.delivery == 1:
                self.state(2)
                self.state(2)
            elif self.delivery == 2:
                self.note_event(127)
            self.note_event(note)
        self.controller(0, 4)
        self.controller(0, 5)
        for stage in range(6, 9):
            for callback in self.callbacks[0]:
                if callback == RELEASE:
                    self.count[8] += 1
                elif callback == RETIRE:
                    self.count[9] += 1
                elif callback == DECAY:
                    self.count[12] += 1
                else:
                    require(callback == IDLE, 'Unexpected independent sustain envelope')
            self.envelopes()
            self.state(stage)
        for stage in (9, 10):
            before = self.removals
            self.collect()
            self.count[10] += 1
            self.count[11] += self.removals - before
            self.state(stage)


def rows(label, body, base=16):
    return [[int(value, base) for value in raw.split()]
            for raw in re.findall(r'^' + label + r' ([^\n]+)$', body, re.M)]


def same(label, observed, expected, identity):
    if observed != expected:
        for at, (actual, wanted) in enumerate(zip(observed, expected)):
            if actual != wanted:
                raise ValueError(f'{label} {identity}, row {at}: {actual} != {wanted}')
        raise ValueError(f'{label} {identity}: missing or extra rows')


def validate(text, coefficient):
    cases = re.findall(r'^SUSTAIN_CASE ([^\n]+)\n([\s\S]*?)(?=^SUSTAIN_CASE |\Z)', text, re.M)
    identities = [tuple(map(int, raw.split())) for raw, _ in cases]
    require(identities == list(matrix()), 'SUSTAIN_CASE incomplete note/pedal/mode/delivery matrix')
    totals = dict(cases=0, deferred_note_events=0, drained_latches=0, immediate_releases=0,
                  matching_entries=0, retired_entries=0, controller_calls=0)
    for identity, (_, body) in zip(identities, cases, strict=True):
        model = Sustain(identity, coefficient)
        model.sequence()
        for label, expected in (('SUSTAIN_STATE', model.pedal_rows),
                                ('RELEASE_STATE', model.entry_rows),
                                ('SUSTAIN_CALL', model.calls)):
            same(label, rows(label, body), expected, identity)
        same('RELEASE_COUNTS', rows('RELEASE_COUNTS', body, 10), [model.count], identity)
        same('SUSTAIN_COUNTS', rows('SUSTAIN_COUNTS', body, 10), [model.control_count], identity)
        totals['cases'] += 1
        totals['deferred_note_events'] += model.deferred
        totals['drained_latches'] += model.drained
        totals['immediate_releases'] += model.immediate
        totals['matching_entries'] += model.matches_total
        totals['retired_entries'] += model.removals
        totals['controller_calls'] += model.control_count[0]
    return totals


def negative_controls(text, coefficient):
    for label, index in (('SUSTAIN_CASE', 1), ('SUSTAIN_STATE', 1), ('SUSTAIN_STATE', 3),
                         ('SUSTAIN_CALL', 2), ('SUSTAIN_CALL', -1), ('RELEASE_STATE', 1),
                         ('RELEASE_COUNTS', -1), ('SUSTAIN_COUNTS', 1)):
        def corrupt(match):
            words = match[1].split()
            base = 10 if label in ('SUSTAIN_CASE', 'RELEASE_COUNTS', 'SUSTAIN_COUNTS') else 16
            value = int(words[index], base) ^ 1
            words[index] = format(value, 'x') if base == 16 else str(value)
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        require(count == 1, 'Missing sustain oracle control')
        try:
            validate(changed, coefficient)
        except ValueError as error:
            require(label in str(error), 'Wrong sustain oracle control failure')
        else:
            raise ValueError('Independent sustain oracle accepted corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original CC64: private firmware absent')
        return 77
    _, ydata = original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    coefficient = ydata[0x16c8]
    require(coefficient == 0x7fffff, 'Changed original zero-rate coefficient')
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve()), 'sustain'],
                            capture_output=True, text=True, timeout=800, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    require(result.returncode == 0 and not result.stderr and
            not any(marker in diagnostics for marker in
                    ('AddressSanitizer', 'LeakSanitizer', 'ThreadSanitizer', 'runtime error:')),
            'Original CC64 native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, coefficient)
    negative_controls(text, coefficient)
    report.write_text(json.dumps({'passed': True, **totals, 'oracle_negative_controls': 8,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Original byte parser/CC64, packed latches, actual deferred '
                                  'release callbacks, three envelope calls for the first declared '
                                  'held entry and real collection/owner cleanup. Declared slot '
                                  'headers/ADSR state and zero converted controls; no positive '
                                  'allocation, boot, audio scheduling or hardware claim.'}, indent=2) + '\n')
    print('Verified original CC64 and deferred note release:', totals)
    return 0


if __name__ == '__main__':
    sys.exit(main())
