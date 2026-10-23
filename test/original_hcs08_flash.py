# SPDX-License-Identifier: GPL-3.0-only
"""Original page algorithm under explicit candidate/reference FLASH contracts."""

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def collect(path):
    selected = {'STRICT', 'CASE', 'PAGE', 'PAGE_END', 'REPLAY', 'VECTOR_CUT',
                'PROFILE_END', 'PROTECTED', 'PROTECTED_END'}
    rows, steps, accesses = [], 0, 0
    with path.open() as stream:
        for line in stream:
            tag, *pairs = line.split()
            if tag == 'STEP':
                steps += 1
            elif tag == 'ACCESS':
                accesses += 1
            elif tag in selected:
                rows.append((tag, {key: int(value) for key, value in (x.split('=') for x in pairs)}))
            else:
                raise ValueError('Unknown original programmer trace row ' + tag)
    return dict(rows=rows, steps=steps, accesses=accesses)


def validate(actual, records):
    rows = actual['rows']
    expected_strict = [('STRICT', dict(seed=seed, replay=replay, pc=0xb2, sp=seed | 0x77,
                                     retired=8, transfers=0))
                       for seed in (0, 0x1200) for replay in (0, 1)]
    if rows[:4] != expected_strict:
        raise ValueError('Original strict idle-launch frontier changed')
    cursor, instructions, replay_count = 4, 16, 0
    for seed, delay in ((0, 5), (0x1200, 97)):
        if rows[cursor] != ('CASE', dict(seed=seed, delay=delay)):
            raise ValueError('Missing reference profile')
        cursor += 1
        retired = 0
        for index in range(7):
            address = int.from_bytes(records[index * 514:index * 514 + 2], 'big')
            if rows[cursor] != ('PAGE', dict(index=index, address=address)):
                raise ValueError('Original supplied page order/address changed')
            cursor += 1
            if index == 0 and delay == 97:
                for cut in (0, 1):
                    for repetition in (0, 1):
                        tag, replay = rows[cursor]
                        if (tag != 'REPLAY' or replay['cut'] != cut or replay['index'] != repetition
                                or replay['bytes'] <= 0 or replay['retired'] <= 0 or replay['identical'] != 1):
                            raise ValueError('Missing original joint native suffix replay')
                        cursor += 1
                        replay_count += 1
            if index == 6 and delay == 97:
                if rows[cursor] != ('VECTOR_CUT', dict(address=65535, unknown=1, marker=114)):
                    raise ValueError('Pending final vector byte must remain unknown after reference interruption')
                cursor += 1
            tag, end = rows[cursor]
            if (tag != 'PAGE_END' or end['retired'] <= retired or not 0 < end['cuts'] <= end['retired']):
                raise ValueError('Invalid native page retirement/power-cut accounting')
            retired = end['retired']
            expected = dict(index=index, retired=retired, sp=(seed | 0x77) if index == 0 else 0x477,
                            transfers=(index + 1) * 513, completed=(index + 1) * 513,
                            **{'pending-at-bgnd': int(delay == 97)}, cuts=end['cuts'])
            if end != expected:
                raise ValueError('Original page command counts, stack or final pending state changed')
            cursor += 1
        instructions += retired
        if rows[cursor] != ('PROFILE_END', dict(seed=seed, delay=delay, pages=7, bytes=3584, retained=1)):
            raise ValueError('Incomplete original page-programming profile')
        cursor += 1
    if rows[cursor] != ('PROTECTED', {}):
        raise ValueError('Missing protected-page execution')
    tag, end = rows[cursor + 1]
    if (tag != 'PROTECTED_END' or end != dict(status=224, marker=114, transfers=0, nvm=1,
                                             retired=end.get('retired')) or end['retired'] <= 0):
        raise ValueError('Original protected-page BGND must not imply programmed contents')
    instructions += end['retired']
    if len(rows) != cursor + 2 or actual['steps'] != instructions or actual['accesses'] <= 0:
        raise ValueError('Complete native trace accounting changed')
    return dict(original_instructions=instructions, reference_profiles=2,
                programmed_pages=14, actual_erase_commands=14, actual_program_commands=7168,
                strict_frontier_replays=4, trusted_native_suffix_replays=replay_count,
                protected_page_marked_without_programming=1, pending_vector_loss_cases=1,
                native_data_accesses=actual['accesses'])


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 FLASH: pinned local Keyboard archive absent')
        return 77
    programmer, records = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    code, pages = trace.with_suffix('.programmer.bin'), trace.with_suffix('.pages.bin')
    code.write_bytes(programmer)
    pages.write_bytes(records)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, code, pages)}
    run = subprocess.run([str(p.resolve()) for p in (binary, code, pages, trace)],
                         capture_output=True, text=True, timeout=240, check=False)
    trace.with_suffix('.log').write_text(run.stdout + run.stderr)
    allowed = 'Original HCS08 page programmer, strict frontier and explicit reference FLASH paths verified.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original programmer execution failed: ' + run.stdout + run.stderr)
    actual = collect(trace)
    totals = validate(actual, records)
    for tag, key in [('STRICT', 'pc'), ('PAGE_END', 'sp'), ('PAGE_END', 'transfers'), ('VECTOR_CUT', 'unknown')]:
        changed = copy.deepcopy(actual)
        next(row for name, row in changed['rows'] if name == tag)[key] += 1
        try:
            validate(changed, records)
        except ValueError:
            continue
        raise ValueError('Fixed original programmer oracle corruption accepted')
    expected = bytearray([0xa5] * 8192)
    expected[0xffbd - 0xe000], expected[0xffbf - 0xe000] = 0xff, 0xfe
    for index in range(7):
        record = records[index * 514:(index + 1) * 514]
        address = int.from_bytes(record[:2], 'big') - 0xe000
        expected[address:address + 512] = record[2:]
    for suffix in ('.fast.bin', '.slow.bin'):
        if Path(str(trace) + suffix).read_bytes() != expected:
            raise ValueError('Actual final NVM differs from independently overlaid supplied pages and own preimage')
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original programmer inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **totals,
        fixed_oracle_corruptions=4, final_image_sha256=hashlib.sha256(expected).hexdigest(),
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        idle_launch_hardware_qualified=False, physical_timing_qualified=False,
        full_boot=False, complete_update=False, BDC_link=False, hardware_equivalence=False,
        physical_traffic=False, fuzz_execution=False,
        scope='Candidate GT8A peripheral with explicit reference ignore-empty and external instruction-boundary '
        'milestones. Default idle launch remains a visible failure. Own complete preimage and declared '
        'debug/RAM entries; no BDC/HC12 transfer, calibrated clock/voltage, physical corruption bounds or '
        'whole-update recovery claim.'), indent=2) + '\n')
    print('Verified original HCS08 FLASH algorithm under explicit reference contracts and strict default frontier.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
