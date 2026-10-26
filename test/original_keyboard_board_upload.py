# SPDX-License-Identifier: GPL-3.0-only
"""Independently check the production-board reset-known upload trace."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_keyboard_receiver import reference_files
from original_hcs08 import reference_inputs
from original_dsp_voice import ROOT, original_records


def validate(path, helper, records):
    expected_cases = ((0, 0), (4, 0), (0, 1))
    cases, replays, host_steps = 0, 0, 0
    stage, cursor, cpu = '', 0, 0
    row_case = None
    ended = False
    case_open, go_seen, case_replays = False, False, 0
    with path.open() as stream:
        for line in stream:
            tag, *fields = line.split()
            row = {key: int(value) for key, value in (field.split('=') for field in fields)}
            if ended:
                raise ValueError('Data after original target completion')
            if tag == 'CASE':
                if case_open or cases >= 3 or (row['phase'], row['secure']) != expected_cases[cases]:
                    raise ValueError('Changed target case/entry assumptions')
                row_case, stage, cursor, cpu = row, 'prepare', 0, 0
                case_open, go_seen, case_replays = True, False, 0
            elif tag == 'RX':
                if stage == 'prepare':
                    frame = [0xe0, 255, 190]
                    for address in range(0x80, 0x124):
                        value = 0 if address < 0xa0 else helper[838 + address - 0xa0]
                        frame.extend((0xc0, address >> 8, address & 255, value))
                    expected = frame[cursor]
                    index = cursor
                elif stage == 'buffer':
                    index = cursor % 4
                    byte = cursor // 4
                    address = 0x200 + byte
                    expected = (0xc0, address >> 8, address & 255, records[byte + 2])[index]
                else:
                    frame = (0xc0, 0, 0x80, records[0], 0xc0, 0, 0x81, 0,
                             0x4b, 0, 0xa0, 0x4f, 4, 0x7c, 8)
                    expected, index = frame[cursor], cursor
                if row['index'] != index or row['value'] != expected:
                    raise ValueError('Native target control/data bytes differ from independent original stream')
                cursor += 1
            elif tag == 'PREP':
                secure = row_case['secure']
                if (stage != 'prepare' or cursor != 659 or row != dict(commands=165, rx=659, tx=1,
                        known=0 if secure else 164, trim=0 if secure else 0xa5, steps=row['steps'])):
                    raise ValueError('Original complete RAM upload/trim/native frame changed')
                stage, cursor = 'buffer', 0
            elif tag == 'BUFFER':
                if stage != 'buffer' or cursor != 2048 or row != dict(commands=677, rx=2707, tx=1, steps=row['steps']):
                    raise ValueError('Original native page-buffer delivery incomplete')
                stage, cursor = 'start', 0
            elif tag == 'GO':
                if not case_open or go_seen or stage != 'start' or cursor != 15 or row['pc'] != 0xa0 or row['sp'] != 0x47c or row['hx'] != 0 or row['a-known'] != 0 or row['hx-known'] != 0xff00 or row['ccr-known'] != 0x68 or row['ccr'] != 0x68:
                    raise ValueError('Actual PC/SP/GO must preserve native reset knowledge')
                go_seen = True
            elif tag == 'CPU':
                cycles = (2, 4, 6, 10, 12, 16, 18, 22)
                pcs = (0xa2, 0xa3, 0xa5, 0xa8, 0xaa, 0xad, 0xaf, 0xb2)
                accumulators = (0, 0, 0x1e, 0x1e, 2, 2, 255, 255)
                if not case_open or not go_seen or row_case['secure'] or cpu >= 8 or row != dict(n=cpu + 1, periods=cycles[cpu],
                        pc=pcs[cpu], hx=0x78, sp=0x47c if cpu == 0 else 0x77,
                        a=accumulators[cpu], ccr=0x68 if cpu < 6 else 0x6c,
                        **{'a-known': 0 if cpu < 2 else 255, 'hx-known': 65535, 'ccr-known': 0xee}):
                    raise ValueError('Independent native HCS08 prologue states/cycle budgets changed')
                cpu += 1
            elif tag == 'FAULT':
                secure = row_case['secure']
                if (not case_open or not go_seen or stage != 'start' or cursor != 15 or row != dict(secure=secure,
                        **{'target-pc': 0xa0 if secure else 0xb2}, retired=0 if secure else 8,
                        executing=0 if secure else 26, commands=682, rx=2722, tx=1, steps=row['steps'])
                        or cpu != (0 if secure else 8)):
                    raise ValueError('Native strict target/host failure no longer has its full causal prefix')
                stage = 'failed'
            elif tag == 'REPLAY':
                if not case_open or stage != 'failed' or case_replays >= 2 or row['index'] != case_replays or row['bytes'] <= 0 or row['identical'] != 1:
                    raise ValueError('Missing complete fresh joint target replay')
                case_replays += 1
                replays += 1
            elif tag == 'CASE_END':
                secure = row_case['secure']
                if not case_open or case_replays != 2 or stage != 'failed' or row != dict(phase=row_case['phase'], secure=secure,
                        **{'host-steps': row['host-steps'], 'target-retired': 0 if secure else 8,
                           'target-executing': 0 if secure else 26}, commands=682, rx=2722, tx=1):
                    raise ValueError('Changed complete target case')
                host_steps += row['host-steps']
                cases += 1
                case_open, stage = False, 'closed'
            elif tag == 'END':
                if case_open or row != {'cases': 3, 'replays': 6, 'RAM-uploads': 3, 'launched': 3, 'NVM-transfers': 0} or cases != 3 or replays != 6:
                    raise ValueError('Incomplete original target matrix')
                ended = True
            elif tag not in ('PIN', 'TICK', 'HOST'):
                raise ValueError('Unknown original target trace row')
    if not ended:
        raise ValueError('Missing original target completion')
    return dict(cases=cases, trusted_joint_suffix_replays=replays,
                native_HC12_steps=host_steps, baseline_HCS08_retired=16,
                original_RAM_uploads=3, accepted_RAM_uploads=2, secured_uploads_ignored=1,
                target_commands=2046, received_bytes=8166, transmitted_bytes=3,
                strict_idle_FSTAT_cases=2, secure_unknown_RAM_fetch_cases=1, NVM_transfers=0)


def check_oracle(path, helper, records):
    original = path.read_text().splitlines(keepends=True)
    changed, moved = [], []
    for line in original:
        if line.startswith('REPLAY ') and len(moved) < 2:
            moved.append(line)
        else:
            changed.append(line)
    last = max(n for n, line in enumerate(changed) if line.startswith('CASE_END '))
    changed[last:last] = moved
    duplicate = list(original)
    first = next(n for n, line in enumerate(duplicate) if line.startswith('CASE '))
    duplicate.insert(first, duplicate[first])
    omitted, altered, done = [], [], False
    for line in original:
        if not line.startswith('GO '):
            omitted.append(line)
        if line.startswith('CPU ') and not done:
            parts = line.split()
            parts = ['periods=' + str(int(p.split('=')[1]) + 1)
                     if p.startswith('periods=') else p for p in parts]
            line, done = ' '.join(parts) + '\n', True
        altered.append(line)
    knowledge = []
    for field, expected, wrong in (('a-known', '0', '255'), ('hx-known', '65535', '255'),
                                   ('ccr-known', '238', '255'), ('sp', '119', '1143')):
        candidate = list(original)
        index = next(n for n, line in enumerate(candidate)
                     if line.startswith('CPU ') and f'{field}={expected}' in line.split())
        candidate[index] = candidate[index].replace(f'{field}={expected}', f'{field}={wrong}')
        knowledge.append(candidate)
    negative = path.with_name(path.name + '.oracle-negative')
    for lines in (changed, duplicate, omitted, altered, *knowledge):
        negative.write_text(''.join(lines))
        try:
            validate(negative, helper, records)
        except ValueError:
            continue
        raise ValueError('Fixed native target oracle corruption accepted')
    negative.unlink()
    return 8


def main():
    if len(sys.argv) != 7:
        raise ValueError('Expected binary,main/Keyboard archives,both native specifications,trace')
    binary, firmware, keyboard, hcs08, hc12, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original Keyboard target: pinned original archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, _ = reference_files(keyboard, trace)
    programmer, records = reference_inputs(keyboard)
    if helper.read_bytes()[838:970] != programmer:
        raise ValueError('Changed embedded original programmer identity')
    pages = trace.with_suffix('.records.bin')
    pages.write_bytes(records)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, firmware, keyboard, hcs08, hc12, helper, word, pages)}
    result = subprocess.run([str(p.resolve()) for p in (binary, firmware, helper, pages, trace)],
                            capture_output=True, text=True, timeout=550, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    allowed = 'Original HC12 upload/GO reaches production HCS08 reset-known strict frontiers through GPIO.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original Keyboard production-board upload failed: ' + result.stdout + result.stderr)
    totals = validate(trace, helper.read_bytes(), records)
    negative = check_oracle(trace, helper.read_bytes(), records)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original target inputs changed during native execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **totals,
        fixed_oracle_corruptions=negative,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, full_caller_history=False,
        physical_timing_qualified=False, active_background_BUSCLK_qualified=False,
        original_reset_SYNC_executed=False, full_DSP_transport=False,
        hardware_equivalence=False, physical_traffic=False, fuzz_execution=False,
        input_sha256={str(p): pin for p, pin in frozen.items()},
        scope='Original HC12 post-display prepare/complete RAM upload, original byte buffer helpers '
        'and contiguous pointer/PC/SP/GO/start-poll code through actual GPIO/BDC into native HCS08. '
        'Production target uses native reset A/X/CCR knowledge;H/TXS produces SP0077. '
        'Explicit post-SYNC/context/clock assumptions; documented six-form unstretched instruction '
        'budgets commit at retirement under a named reference contract. No physical reset, full helper, '
        'bus subcycle/arbitration, peripheral-clock or whole update/recovery equivalence.'), indent=2) + '\n')
    print('Verified production-board original upload/GO, reset knowledge and strict coupled frontiers.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
