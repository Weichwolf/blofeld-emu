# SPDX-License-Identifier: GPL-3.0-only
"""Independent note matching, envelope retirement and original active-list collection."""

import itertools
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
BASE, STRIDE = 0xc500, 0x106
NOTES, CHANNELS = (64, 65, 64, 64), (0, 0, 1, 0)
DECAY, RELEASE, RETIRE, IDLE = 0x1639, 0x160f, 0x1609, 0x160e
MASK, W = 0xffffff, 1 << 24


class Model:
    def __init__(self, modes, note, zero, filtered, coefficient):
        self.modes, self.note, self.zero, self.filtered = modes, note, zero, filtered
        self.coefficient = coefficient
        self.flags = 0
        self.order = list(range(4))
        self.owners = [i % 4 for i in range(24)]
        self.mask = MASK
        self.callbacks = [[DECAY] * 4 for _ in range(4)]
        self.values = [[0x400000 + slot * 0x10000 + envelope * 0x1000
                        for envelope in range(4)] for slot in range(4)]
        self.velocities = [0] * 4
        self.matches = [] if filtered else [i for i in range(4) if NOTES[i] == note and CHANNELS[i] == 0]
        self.removals = 0

    def row(self, stage):
        result = [stage, self.flags, *[BASE + i * STRIDE for i in self.order]]
        result.extend([0] * (5 - len(self.order)))
        result.extend([self.mask, *[0 if i is None else BASE + i * STRIDE for i in self.owners]])
        for slot in range(4):
            result.append(self.velocities[slot])
            for callback, value in zip(self.callbacks[slot], self.values[slot], strict=True):
                result.extend((callback, value))
        return result

    def release(self):
        for slot in self.matches:
            self.velocities[slot] = 0 if self.zero else 64 << 16
            for envelope in range(4):
                if not self.modes & (1 << envelope):
                    self.callbacks[slot][envelope] = RELEASE
        for slot in reversed(self.matches):
            at = self.order.index(slot)
            while at + 1 < len(self.order) and self.callbacks[self.order[at + 1]][1] > RELEASE:
                self.order[at], self.order[at + 1] = self.order[at + 1], self.order[at]
                at += 1

    def envelopes(self):
        for envelope in range(4):
            callback, value = self.callbacks[0][envelope], self.values[0][envelope]
            if callback in (RELEASE, DECAY):
                full = value * W - 2 * self.coefficient * value
                self.values[0][envelope] = max(-(1 << 23), min((1 << 23) - 1, full // W)) & MASK
                if callback == RELEASE and full <= 0:
                    self.callbacks[0][envelope] = RETIRE
            elif callback == RETIRE:
                self.flags |= 2
                self.values[0][envelope], self.callbacks[0][envelope] = 0, IDLE
            elif callback != IDLE:
                raise ValueError('Unsupported independent envelope fixture')

    def collect(self):
        # A successful removal tail-returns through owner cleanup. Only a later
        # scan reaching the terminator clears the collection flag.
        removed = set(next(([slot] for slot in self.order if self.callbacks[slot][1] == IDLE), []))
        self.order = [slot for slot in self.order if slot not in removed]
        for index, owner in enumerate(self.owners):
            if owner in removed:
                self.owners[index] = None
                self.mask &= ~(1 << (23 - index))
        if not removed:
            self.flags &= ~2
        self.removals += len(removed)

    def counts(self):
        released = (4 - self.modes.bit_count()) if 0 in self.matches else 0
        return [int(not self.filtered), int(self.zero and not self.filtered), 0, 0, 0, 0,
                int(not self.filtered), len(self.matches), 2 * released, released, 2,
                self.removals, 3 * (4 - released)]


def matrix():
    result = []
    for modes in range(16):
        result.extend((modes, channel, note, zero, delivery, 0)
                      for channel, note, zero, delivery in
                      itertools.product((0, 15), (64, 65, 67), (0, 1), range(3)))
        result.append((modes, 15, 64, 0, 0, 1))
    return result


def validate(text, coefficient):
    cases = re.findall(r'^RELEASE_CASE ([^\n]+)\n([\s\S]*?)(?=^RELEASE_CASE |\Z)', text, re.M)
    identities = [tuple(map(int, raw.split())) for raw, _ in cases]
    if identities != matrix():
        raise ValueError('RELEASE_CASE incomplete hold-mode/note/channel/delivery matrix')
    totals = dict(cases=0, matching_entries=0, retired_entries=0, release_callbacks=0)
    for (modes, _channel, note, zero, delivery, filtered), (_, body) in zip(identities, cases, strict=True):
        model = Model(modes, note, zero, filtered, coefficient)
        expected = [model.row(0)] * (3 if delivery == 1 else 2 if delivery == 2 else 1)
        model.release()
        expected.append(model.row(1))
        for stage in range(2, 5):
            model.envelopes()
            expected.append(model.row(stage))
        model.collect()
        expected.append(model.row(5))
        model.collect()
        expected.append(model.row(6))
        actual = [[int(value, 16) for value in raw.split()]
                  for raw in re.findall(r'^RELEASE_STATE ([^\n]+)$', body, re.M)]
        if actual != expected:
            for at, (observed, wanted) in enumerate(zip(actual, expected)):
                if observed != wanted:
                    raise ValueError(f'RELEASE_STATE {modes,note,zero,delivery,filtered}, row {at}: '
                                     f'{observed} != {wanted}')
            raise ValueError('RELEASE_STATE missing or extra complete states')
        actual_counts = re.findall(r'^RELEASE_COUNTS ([^\n]+)$', body, re.M)
        if len(actual_counts) != 1 or list(map(int, actual_counts[0].split())) != model.counts():
            raise ValueError('RELEASE_COUNTS differs from actual original calls/returns')
        totals['cases'] += 1
        totals['matching_entries'] += len(model.matches)
        totals['retired_entries'] += model.removals
        totals['release_callbacks'] += model.counts()[8]
    return totals


def negative_controls(text, coefficient):
    for label, index in (('RELEASE_CASE', 2), ('RELEASE_COUNTS', -1),
                         ('RELEASE_STATE', -1), ('RELEASE_STATE', 7)):
        def corrupt(match):
            words = match[1].split()
            base = 16 if label == 'RELEASE_STATE' else 10
            value = int(words[index], base) ^ 1
            words[index] = format(value, 'x') if base == 16 else str(value)
            return label + ' ' + ' '.join(words)
        changed, count = re.subn('^' + label + r' ([^\n]+)$', corrupt, text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing release negative control')
        try:
            validate(changed, coefficient)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong release negative-control failure') from error
        else:
            raise ValueError('Independent release oracle accepted corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original MIDI release: private firmware absent')
        return 77
    _, ydata = original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    coefficient = ydata[0x16c8]
    if coefficient != 0x7fffff:
        raise ValueError('Changed original zero-rate coefficient')
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve()), 'release'],
                            capture_output=True, text=True, timeout=800, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in
                               ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Original MIDI release native failure: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, coefficient)
    negative_controls(text, coefficient)
    report.write_text(json.dumps({'passed': True, **totals, 'oracle_negative_controls': 4,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Original byte parser/release/prioritization, three '
                                  'envelope calls for the first declared held entry, original '
                                  'collection and mask-owner cleanup. Declared slot headers, '
                                  'ADSR state and zero converted controls; no positive-note '
                                  'allocation, boot, scheduled audio or hardware claim.'}, indent=2) + '\n')
    print('Verified original MIDI release, held-envelope retirement and active-list cleanup:', totals)
    return 0


if __name__ == '__main__':
    sys.exit(main())
