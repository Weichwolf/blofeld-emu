# SPDX-License-Identifier: GPL-3.0-only
"""Original HC12 reset/SYNC with explicit reference circuitry and clock history."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_keyboard_receiver import reference_files
from original_dsp_voice import ROOT, original_records

CASES = ((4000000, 1, 1, 1), (4000000, 3, 1, 1), (3000000, 1, 1, 1),
         (6000000, 1, 1, 1), (4000000, 1, 0, 1), (4000000, 1, 1, 0))


def rows(path):
    result = []
    for line in path.read_text().splitlines():
        data, _, error = line.partition(' error=')
        tag, *items = data.split()
        values = {key: int(value) for key, value in (item.split('=') for item in items)}
        result.append((tag, values, error))
    return result


def validate(path, index):
    rate, phase, pullup, connected = CASES[index]
    data = rows(path)
    if data[0] != ('CASE', dict(index=index, rate=rate, phase=phase, pullup=pullup, connected=connected), ''):
        raise ValueError('Changed explicit reset/SYNC profile')
    if data[1] != ('CLOCK', dict(common=120000000, bus=30000000,
                               **{'timer-divisor': 4, 'timer-start': 100, 'reset-cycles': 5}), ''):
        raise ValueError('Changed nominal PHY/TIM reference clock')
    if data[-1] != ('CHECK', dict(helper=1, nvm=1, **{'target-instructions': 0}), ''):
        raise ValueError('Missing unchanged helper/NVM/held-target qualification')
    releases, assertions, targets, stamps, ends, faults, timer_reads = [], [], [], {}, [], [], []
    latest = 0
    for tag, row, error in data[2:-1]:
        if 'time' in row:
            if row['time'] < latest:
                raise ValueError('Backward native observation time')
            latest = row['time']
        if tag == 'ASSERT':
            assertions.append(row)
        elif tag == 'RELEASE':
            if not assertions or row['time'] <= assertions[0]['time'] or row['period'] != 120000000 // rate:
                raise ValueError('Invalid actual reset release/clock epoch')
            releases.append(row)
        elif tag == 'HOST':
            if row['address'] not in (1, 3) or row['drive'] not in (0, 1, 2):
                raise ValueError('Invalid original PB1 host write')
        elif tag == 'TARGET':
            if not releases or (row['time'] - releases[0]['time'] - phase) % (120000000 // rate):
                raise ValueError('Target drive changed outside an actual supplied clock edge')
            targets.append(row)
        elif tag == 'TIMER':
            if row['address'] != 0x44 or row['size'] != 2 or row['time'] % 4:
                raise ValueError('Changed native aligned TCNT word-read cycle')
            timer_reads.append(row)
        elif tag == 'STAMP':
            if row['pc'] in stamps:
                raise ValueError('Duplicate original timestamp/difference')
            if row['pc'] in (0x13ef, 0x13f7):
                sample = timer_reads[-1] if timer_reads else None
                if (sample is None or sample['pc'] != row['pc']
                        or row['value'] != (((sample['time'] - 100) // 16) & 65535)):
                    raise ValueError('Original LDD timestamp lost its native word read')
                row['read_time'] = sample['time']
            stamps[row['pc']] = row
        elif tag == 'END':
            ends.append(row)
        elif tag == 'FAULT':
            if index != 4 or row['pc'] != 0x13b0 or 'Unknown MCU GPIO input at 0x0008 mask=0x80' not in error:
                raise ValueError('Changed original reset-pin failure boundary')
            faults.append(row)
        else:
            raise ValueError('Unexpected reset/SYNC trace event')
    if len(assertions) != 1 or len(releases) != pullup:
        raise ValueError('Missing actual reset assertion/release')
    if len(timer_reads) != (2 if connected and pullup else 0):
        raise ValueError('Missing or duplicate actual TCNT operand cycles')
    if index == 4:
        if len(faults) != 1 or ends or targets or stamps:
            raise ValueError('Unknown reset release cannot become SYNC success')
        return dict(fault=True, target_instructions=0)
    return validate_end(ends, faults, targets, stamps, index, rate, connected)


def validate_end(ends, faults, targets, stamps, index, rate, connected):
    if len(ends) != 1 or faults:
        raise ValueError('Missing complete native reset/SYNC return')
    end = ends[0]
    expected_result = 9 if index < 2 else 0
    if (end['rate'] != rate or end['result'] != expected_result
            or end['time'] % 4 or end['steps'] <= 0
            or end['commands'] != int(expected_result == 9) or end['syncs'] != 2 * connected):
        raise ValueError('Changed original reference-clock acceptance/return')
    if not connected:
        if targets or stamps or end['difference'] != 0:
            raise ValueError('Disconnected target cannot supply a response')
        return dict(disconnected=True, result=0, native_steps=end['steps'])
    if set(stamps) != {0x13ef, 0x13f7, 0x13f9} or len(targets) != 6:
        raise ValueError('Missing both observed target SYNC pulses or original measurements')
    period = 120000000 // rate
    for number in range(2):
        low, high, release = targets[number * 3:number * 3 + 3]
        if ([low['drive'], high['drive'], release['drive']] != [1, 2, 0]
                or [low['syncs'], high['syncs'], release['syncs']] != [number, number, number + 1]
                or high['time'] - low['time'] != 128 * period
                or release['time'] - high['time'] != period):
            raise ValueError('SYNC low/speedup pulse differs from independent clock counts')
    low, high = stamps[0x13ef]['value'], stamps[0x13f7]['value']
    delta = (high - low) & 65535
    if (stamps[0x13f9]['value'] != delta or end['difference'] != delta
            or end['low'] != low or end['high'] != high
            or (200 <= delta <= 300) != bool(expected_result)):
        raise ValueError('Original modular timestamp acceptance differs')
    return dict(result=expected_result, difference=delta, native_steps=end['steps'],
                low_sample_latency_ticks=stamps[0x13ef]['read_time'] - targets[3]['time'],
                high_sample_latency_ticks=stamps[0x13f7]['read_time'] - targets[4]['time'])


def corruptions(path):
    original = path.read_text().splitlines(keepends=True)
    cases = []
    for tag, field in [('TIMER', 'time'), ('TARGET', 'time'), ('STAMP', 'value')]:
        changed, done = [], False
        for line in original:
            if not done and line.startswith(tag + ' '):
                parts = line.split()
                for n, part in enumerate(parts):
                    if part.startswith(field + '='):
                        increment = 16 if tag == 'TIMER' else 1
                        parts[n] = field + '=' + str(int(part.split('=')[1]) + increment)
                line, done = ' '.join(parts) + '\n', True
            changed.append(line)
        cases.append(changed)
    cases.append([line for line in original if not line.startswith('RELEASE ')])
    altered = path.with_name(path.name + '.oracle-negative')
    for changed in cases:
        altered.write_text(''.join(changed))
        try:
            validate(altered, 0)
        except ValueError:
            continue
        raise ValueError('Fixed reset/SYNC oracle corruption accepted')
    altered.unlink()
    return len(cases)


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original Keyboard reset/SYNC: pinned original archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, _ = reference_files(keyboard, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, helper, word)}
    run = subprocess.run([str(p.resolve()) for p in (binary, firmware, helper, trace)],
                         capture_output=True, text=True, check=False, timeout=270)
    trace.with_suffix('.log').write_text(run.stdout + run.stderr)
    if run.returncode or run.stderr or run.stdout.strip() != 'Original HC12 reset/SYNC reference clocks, GPIO and polled timer verified.':
        raise ValueError('Original reset/SYNC failed: ' + run.stdout + run.stderr)
    results = [validate(Path(str(trace) + '.' + str(n)), n) for n in range(len(CASES))]
    negative = corruptions(Path(str(trace) + '.0'))
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original reset/SYNC inputs changed')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, cases=results,
        fixed_oracle_corruptions=negative, HCS08_instructions=0, physical_timing_qualified=False,
        complete_update=False, full_boot=False, physical_traffic=False, fuzz_execution=False,
        scope='Own caller and declared source-derived locked nominal PHY/TIM state; reference '
        'passive RESET/BKGD circuit, weak other E inputs and candidate active-BDC initialization. '
        'Native target reset/ICG startup, prior target history, physical continuity and full '
        'application/SMF/update/recovery remain unqualified.'), indent=2) + '\n')
    print('Verified original HC12 reset/SYNC, independent timer/pulse expectations and strict failure profiles.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
