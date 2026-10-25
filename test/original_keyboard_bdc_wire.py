# SPDX-License-Identifier: GPL-3.0-only
"""Unchanged HC12 BDC byte routines through GPIO and a clocked target wire."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_keyboard_receiver import reference_files
from original_dsp_voice import ROOT, original_records


def validate(path):
    phase, pairs, pins, rx, steps, total_steps, cycles = -1, 0, 0, 0, 0, 0, 0
    total, total_pins, ended = 0, 0, False
    bit_start = 0
    phase_open = False
    with path.open() as stream:
        for line in stream:
            tag, *fields = line.split()
            row = {key: int(value) for key, value in (field.split('=') for field in fields)}
            if ended:
                raise ValueError('Trailing data after native BDC wire completion')
            if tag not in ('PHASE', 'END') and not phase_open:
                raise ValueError('Native BDC event outside an active reference phase')
            if tag == 'PHASE':
                if phase_open:
                    raise ValueError('Missing native BDC phase completion')
                phase += 1
                if row != dict(index=phase, ratio=8, offset=phase * 2 + 1) or phase >= 8:
                    raise ValueError('Changed reference phase/clock contract')
                pairs, pins, rx, steps, cycles = 0, 0, 0, 0, 0
                phase_open = True
            elif tag == 'PIN':
                if (row['cycles'] <= cycles or row['address'] not in (1, 3)
                        or row['drive'] not in (0, 1, 2) or not 0x142f <= row['pc'] < 0x14db):
                    raise ValueError('Invalid original BDC GPIO write/time/entry')
                value = pairs if phase in (0, 4) else (0, 255, 0x55, 0xaa)[pairs]
                position = pins % 248
                if position < 224:
                    frame = (0xc0, 2, 0, value, 0xe0, 2, 1)
                    bit, slot = divmod(position, 4)
                    data = 2 if frame[bit // 8] & (128 >> (bit % 8)) else 0
                    address = (1, 3, 1, 1)[slot]
                    expected_value = (0, 255, data, 2)[slot]
                    drive = (0 if position == 0 and pairs else 1, 1, 2 if data else 1, 2)[slot]
                    offset = (0, 4, 43, 88)[slot]
                else:
                    bit, slot = divmod(position - 224, 3)
                    address = (1, 3, 3)[slot]
                    expected_value = (0, 255, 253)[slot]
                    drive = (1 if bit == 0 else 0, 1, 0)[slot]
                    offset = (0, 4, 38)[slot]
                if slot == 0:
                    bit_start = row['cycles']
                if (row['address'], row['value'], row['drive'], row['cycles'] - bit_start) != (
                        address, expected_value, drive, offset):
                    raise ValueError('Original GPIO latch/direction/data/timing waveform changed')
                cycles = row['cycles']
                pins += 1
            elif tag == 'RX':
                value = pairs if phase in (0, 4) else (0, 255, 0x55, 0xaa)[pairs]
                frame = (0xc0, 2, 0, value, 0xe0, 2, 1)
                index = rx % 7
                if row['index'] != (index if index < 4 else index - 4) or row['value'] != frame[index]:
                    raise ValueError('Original serial bytes differ from independent command frame')
                rx += 1
            elif tag == 'PAIR':
                value = pairs if phase in (0, 4) else (0, 255, 0x55, 0xaa)[pairs]
                pairs += 1
                if (row != dict(value=value, reply=value ^ 255, commands=pairs * 2,
                                rx=pairs * 7, tx=pairs, steps=row['steps'])
                        or row['steps'] <= steps or pins != pairs * 248 or rx != pairs * 7):
                    raise ValueError('Native target byte/reply/command/GPIO counts changed')
                steps = row['steps']
            elif tag == 'PHASE_END':
                expected = 256 if phase in (0, 4) else 4
                if (pairs != expected or row != dict(index=phase, cases=pairs, commands=pairs * 2,
                        rx=pairs * 7, tx=pairs, steps=steps, cycles=row['cycles'],
                        **{'pin-writes': pins}) or row['cycles'] <= cycles):
                    raise ValueError('Incomplete original wire phase')
                total += pairs
                total_steps += steps
                total_pins += pins
                phase_open = False
            elif tag == 'END':
                if (phase_open or phase != 7 or row != dict(phases=8, cases=536, commands=1072, rx=3752,
                        tx=536, steps=total_steps, **{'pin-writes': 132928}) or total != 536
                        or total_pins != 132928):
                    raise ValueError('Incomplete original wire matrix')
                ended = True
            else:
                raise ValueError('Unknown original wire trace row')
    if not ended:
        raise ValueError('Missing original wire completion')
    return dict(phases=8, byte_pairs=536, commands=1072, received_bytes=3752,
                transmitted_bytes=536, native_HC12_instructions=total_steps,
                actual_GPIO_writes=132928)


def check_oracle(trace):
    rows = trace.read_text().splitlines(keepends=True)
    phase_end = next(row for row in rows if row.startswith('PHASE_END index=1 '))
    first_pin = next(index for index, row in enumerate(rows) if row.startswith('PIN '))
    variants = [
        [row for row in rows if not row.startswith('END ')],
        [row for row in rows if not row.startswith('PHASE_END index=1 ')],
        [row.replace('value=0 ', 'value=1 ', 1) if index == first_pin else row
         for index, row in enumerate(rows)],
        [part for row in rows if not row.startswith('PHASE_END index=2 ')
         for part in ([row, row] if row == phase_end else [row])],
    ]
    changed = trace.with_name(trace.name + '.oracle-negative')
    for rows in variants:
        changed.write_text(''.join(rows))
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Fixed native BDC trace corruption accepted')
    changed.unlink()
    return len(variants)


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original Keyboard BDC wire: pinned original archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, _ = reference_files(keyboard, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, helper, word)}
    result = subprocess.run([str(path.resolve()) for path in (binary, firmware, helper, trace)],
                            capture_output=True, text=True, timeout=330, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    allowed = 'Original HC12 BDC GPIO byte helpers and actual target memory/replies verified.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original Keyboard wire failed: ' + result.stdout + result.stderr)
    counts = validate(trace)
    corruptions = check_oracle(trace)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != sha for path, sha in frozen.items()):
        raise ValueError('Original wire inputs changed during native execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **counts,
        fixed_oracle_corruptions=corruptions,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, physical_timing_qualified=False,
        reference_host_cycles_per_BDC=8, half_host_cycle_phase_offsets=list(range(1, 16, 2)),
        original_reset_SYNC_executed=False, HCS08_instructions_executed=0,
        hardware_equivalence=False, physical_traffic=False, fuzz_execution=False,
        scope='Original HC12 byte helpers, explicit post-SYNC delays9/12, active-reset BDC mode, '
        'eight host bus cycles per BDC reference clock and eight phases. Actual GPIO sends/reads '
        'reach target memory; target replies are independently seeded. No physical clock/wiring, '
        'reset/SYNC caller, full helper, target execution or whole update qualification.'), indent=2) + '\n')
    print('Verified original HC12 GPIO BDC byte helpers and actual target replies across eight phases.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
