# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original programmer entry after architectural reset and real BDC commands."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs

SEEDS = ((0xa5, 0x3322, 0x4567, 255, 0xa5), (0x5a, 0x44bb, 0x2456, 0xfc, 0x5a))
DIAGNOSTIC = ('HCS08 execution pc=0x00b2: HCS08 FLASH register write address=0x1825 '
              'pc=0x00b2 ccr=0x006c agent=CPU: unqualified idle FCBEF launch without a staged command')


def registers(pc, hx, sp, a, known, ccr, pc_known=1):
    return dict(pc=pc, **{'pc-known': pc_known}, hx=hx, sp=sp, a=a, known=known, ccr=ccr)


def expected_rows():
    rows = []
    for index, (a, hx, sp, protection, trim) in enumerate(SEEDS):
        rows.append(('CASE', dict(index=index, a=a, hx=hx, sp=sp, protection=protection, trim=trim)))
        rows.append(('RESET', dict(registers(0, hx & 255, 255, a, 0x68, 0x68, 0), status=0xc8, **{'ram-known': 0})))
        rows.append(('TRIM', dict(value=trim, commands=3)))
        rows.append(('UPLOAD', dict(bytes=164, known=164, commands=167, **{'pc-known': 0})))
        rows.append(('START', dict(registers(0xa0, hx & 255, 0x47c, a, 0x68, 0x68), commands=171, status=0x88)))
        pcs = (0xa2, 0xa3, 0xa5, 0xa8, 0xaa, 0xad, 0xaf, 0xb2)
        periods = (2, 4, 6, 10, 12, 16, 18, 22)
        accumulators = (a, a, 30, 30, 2, 2, 255, 255)
        for n, (pc, period, accumulator) in enumerate(zip(pcs, periods, accumulators), 1):
            rows.append(('STEP', dict(registers(pc, 0x78, 0x47c if n == 1 else 0x77,
                                               accumulator, 0xee, 0x68 if n < 7 else 0x6c),
                                      n=n, periods=period)))
        for address, value in ((0x1820, 30), (0x1821, 2), (0x1824, 255)):
            rows.append(('ACCESS', dict(address=address, value=value, write=1)))
        rows.append(('PENDING', dict(periods=24, remaining=2, retired=8)))
        for attempt in range(5):
            rows.append(('ATTEMPT', dict(index=attempt)))
            rows.append(('ERROR', DIAGNOSTIC))
            rows.append(('FAULT', dict(registers(0xb2, 0x78, 0x77, 255, 0xee, 0x6c),
                                       n=8, periods=26, offered=27, commands=171,
                                       transfers=0, completions=0)))
            if attempt:
                rows.append(('REPLAY', dict(cut=(attempt-1)//2, index=(attempt-1)%2, identical=1)))
        rows.append(('CASE_END', dict(index=index, retired=8, commands=171)))
    rows.append(('END', dict(cases=2, instructions=16, faults=10, replays=8)))
    return rows


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original reset-entry trace')
    actual = []
    for line in text.splitlines():
        tag, separator, body = line.partition(' ')
        if not separator or not body:
            raise ValueError('Incomplete original reset-entry row')
        if tag == 'ERROR':
            actual.append((tag, body))
            continue
        row = {}
        for item in body.split():
            key, value = item.split('=')
            if key in row:
                raise ValueError('Duplicate original reset-entry field')
            row[key] = int(value)
        actual.append((tag, row))
    if actual != expected_rows():
        raise ValueError('Changed or incomplete original reset-entry chronology/registers/periods/faults')


def controls(text):
    changes = [text.replace('pc-known=0', 'pc-known=1', 1),
               text.replace('known=104', 'known=255', 1),
               text.replace('sp=119', 'sp=4727', 1),
               text.replace('periods=22', 'periods=21', 1),
               text.replace('value=30 write=1', 'value=31 write=1', 1),
               text.replace('unqualified idle FCBEF', 'successful idle FCBEF', 1),
               text.replace('remaining=2', 'remaining=1', 1),
               text.replace('transfers=0', 'transfers=1', 1),
               text.replace('identical=1', 'identical=0', 1),
               text+'EXTRA n=1\n', text[:-1]]
    for tag in ('RESET', 'TRIM', 'UPLOAD', 'START', 'STEP', 'ACCESS', 'ERROR', 'FAULT', 'REPLAY', 'CASE_END', 'END'):
        lines = text.splitlines(keepends=True)
        at = next(i for i, line in enumerate(lines) if line.startswith(tag+' '))
        changes.append(''.join(lines[:at]+lines[at+1:]))
    for changed in changes:
        if changed == text:
            raise ValueError('Directed own-trace mutation was not applied')
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a directed own-trace corruption')
    return len(changes)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 reset entry: pinned local Keyboard archive absent')
        return 77
    programmer, _ = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    code = trace.with_suffix('.programmer.bin')
    code.write_bytes(programmer)
    paths = (binary, archive, specification, code)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    result = subprocess.run([str(p.resolve()) for p in (binary, code, trace)],
                            capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original HCS08 reset entry preserves partial CCR and default FSTAT refusal.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original HCS08 reset-entry execution failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original reset-entry inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, cases=2,
        original_instructions=16, actual_FSTAT_failures=10, fresh_joint_fault_replays=8,
        actual_BDC_commands=342, actual_RAM_upload_bytes=328,
        collector_negative_controls=negative, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p):sha for p, sha in frozen.items()},
        full_boot=False, complete_update=False, original_HC12_helper_executed=False,
        physical_wire=False, physical_timing_qualified=False, target_part_qualified=False,
        hardware_equivalence=False, physical_traffic=False, fuzz_execution=False,
        scope='Actual warm pre-vector CPU reset and byte-level BDC background/trim/upload/PC/SP/GO, '
        'original eight-instruction programmer prefix with partial CCR and independent register/period/access expectations. '
        'Two boundary/mid-budget cuts and eight fresh joint fault continuations. '
        'Declared unsecured preimage/trim and unknown vector/RAM inputs; no original HC12/pin/SMF caller, '
        'GO queue-refill/physical timing, NVM operating rate, erase/program or complete recovery qualification.'), indent=2)+'\n')
    print('Verified original HCS08 entry from partial reset state through actual BDC commands.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
