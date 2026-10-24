# SPDX-License-Identifier: GPL-3.0-only
"""Original BDC upload/programmer from unknown cold registers; no NVM bypass."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_reset_entry import DIAGNOSTIC, SEEDS, registers


def unknown(values, hx_known):
    return dict(values, **{'a-known': 0, 'hx-known': hx_known})


def expected_rows():
    rows = []
    for index, (_, _, _, protection, trim) in enumerate(SEEDS):
        rows.append(('CASE', dict(index=index, **{'power-on': 1}, protection=protection, trim=trim)))
        rows.append(('RESET', dict(unknown(registers(0, 0, 255, 0, 0x68, 0x68, 0), 0xff00),
                                   status=0xc8, **{'ram-known': 0})))
        rows.append(('TRIM', dict(value=trim, commands=3)))
        rows.append(('UPLOAD', dict(bytes=164, known=164, commands=167, **{'pc-known': 0})))
        rows.append(('START', dict(unknown(registers(0xa0, 0, 0x47c, 0, 0x68, 0x68), 0xff00),
                                   commands=171, status=0x88)))
        pcs = (0xa2, 0xa3, 0xa5, 0xa8, 0xaa, 0xad, 0xaf, 0xb2)
        periods = (2, 4, 6, 10, 12, 16, 18, 22)
        accumulators = (0, 0, 30, 30, 2, 2, 255, 255)
        for n, (pc, period, accumulator) in enumerate(zip(pcs, periods, accumulators), 1):
            values = registers(pc, 0x78, 0x47c if n == 1 else 0x77,
                               accumulator, 0xee, 0x68 if n < 7 else 0x6c)
            if n <= 2:
                values = unknown(values, 0xffff)
            rows.append(('STEP', dict(values, n=n, periods=period)))
        for address, value in ((0x1820, 30), (0x1821, 2), (0x1824, 255)):
            rows.append(('ACCESS', dict(address=address, value=value, write=1)))
        rows.append(('PENDING', dict(periods=24, remaining=2, retired=8)))
        rows.append(('ATTEMPT', dict(index=0)))
        rows.append(('ERROR', DIAGNOSTIC))
        rows.append(('FAULT', dict(registers(0xb2, 0x78, 0x77, 255, 0xee, 0x6c),
                                   n=8, periods=26, offered=27, commands=171,
                                   transfers=0, completions=0)))
        cuts = ((0, 0, 0, 0), (1, 0, 0, 0), (2, 1, 0, 1), (2, 4, 2, 0),
                (2, 5, 2, 1), (2, 6, 3, 0), (3, 22, 8, 0), (4, 24, 8, 2))
        for cut, (stage, period, retired, remaining) in enumerate(cuts):
            for replay in range(2):
                rows.append(('COLD_REPLAY', dict(cut=cut, index=replay, stage=stage, periods=period,
                                                 retired=retired, remaining=remaining, identical=1)))
        rows.append(('CASE_END', dict(index=index, retired=8, commands=171)))
    rows.append(('END', dict(cases=2, instructions=180, faults=34, replays=32,
                             commands=1042, **{'ram-writes': 984})))
    return rows


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original cold-programmer trace')
    rows = []
    for line in text.splitlines():
        tag, separator, body = line.partition(' ')
        if not separator or not body:
            raise ValueError('Incomplete original cold-programmer row')
        if tag == 'ERROR':
            rows.append((tag, body))
            continue
        fields = {}
        for pair in body.split():
            key, value = pair.split('=')
            if key in fields:
                raise ValueError('Duplicate original cold-programmer field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows():
        raise ValueError('Changed original cold registers, commands, periods, faults or continuations')


def controls(text):
    lines = text.splitlines(keepends=True)
    examples = [text[:-1], text + 'EXTRA value=1\n', ''.join(lines[1:]), text + lines[-1]]
    changes = [('RESET ', 'a-known=0', 'a-known=255'),
               ('RESET ', 'hx-known=65280', 'hx-known=65535'),
               ('RESET ', 'pc-known=0', 'pc-known=1'),
               ('RESET ', 'known=104', 'known=255'),
               ('START ', 'a-known=0', 'a-known=255'),
               ('STEP n=1 ', 'a-known=0', 'a-known=255'),
               ('STEP n=3 ', 'a=30', 'a=0'),
               ('ACCESS address=6176 ', 'value=30', 'value=31'),
               ('ERROR ', 'unqualified idle FCBEF', 'successful idle FCBEF'),
               ('FAULT ', 'transfers=0', 'transfers=1'),
               ('COLD_REPLAY cut=3 ', 'periods=4', 'periods=5'),
               ('COLD_REPLAY cut=4 ', 'remaining=1', 'remaining=0'),
               ('COLD_REPLAY cut=0 ', 'stage=0', 'stage=1'),
               ('END ', 'instructions=180', 'instructions=16'),
               ('END ', 'commands=1042', 'commands=342'),
               ('END ', 'ram-writes=984', 'ram-writes=328')]
    # Select the exact instruction boundary in the native trace.
    for prefix, before, after in changes:
        if prefix.startswith('STEP n='):
            n = prefix.split('=')[1].strip()
            at = next(i for i, line in enumerate(lines) if line.startswith('STEP n='+n+' '))
        else:
            at = next(i for i, line in enumerate(lines) if line.startswith(prefix))
        if before not in lines[at]:
            raise ValueError('Invalid fixed cold-programmer control')
        changed = list(lines)
        changed[at] = changed[at].replace(before, after, 1)
        examples.append(''.join(changed))
    for tag in ('RESET', 'TRIM', 'UPLOAD', 'START', 'STEP', 'ACCESS', 'PENDING', 'ATTEMPT',
                'ERROR', 'FAULT', 'COLD_REPLAY', 'CASE_END', 'END'):
        at = next(i for i, line in enumerate(lines) if line.startswith(tag+' '))
        examples.append(''.join(lines[:at]+lines[at+1:]))
    for changed in examples:
        if changed == text:
            raise ValueError('Cold-programmer control did not change its input')
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed cold-programmer corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original cold programmer: pinned local Keyboard archive absent')
        return 77
    programmer, _ = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    code = trace.with_suffix('.programmer.bin')
    code.write_bytes(programmer)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, code)}
    result = subprocess.run([str(p.resolve()) for p in (binary, code, trace)] + ['power-on'],
                            capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original HCS08 reset entry preserves partial CCR and default FSTAT refusal.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original cold-programmer execution failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original cold-programmer inputs changed during execution')
    report = dict(passed=True, cases=2, power_on=True, supplied_pre_reset_registers=False,
        baseline_original_instructions=16, actual_retired_instructions=180,
        actual_FSTAT_failures=34, fresh_joint_replays=32, cuts_per_context=8,
        baseline_BDC_commands=342, actual_BDC_commands_including_replays=1042,
        baseline_RAM_upload_bytes=328, actual_RAM_writes_including_replays=984,
        collector_negative_controls=negative,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): sha for p, sha in frozen.items()},
        NVM_frequency_supplied=False, full_boot=False, complete_update=False,
        original_HC12_helper_executed=False, physical_wire=False, physical_timing_qualified=False,
        target_part_qualified=False, hardware_equivalence=False, physical_traffic=False,
        fuzz_execution=False, scope='Architectural power-on, logical BDC upload/PC/SP/GO and '
        'original native register definitions with reference instruction budgets. Fresh joint '
        'continuations include unknown reset/PC, unknown uploaded registers and pending first '
        'LDX/LDA effects. Actual strict idle-FSTAT refusal precedes any NVM command or transfer. '
        'Explicit unsecured/protection/factory-trim fixtures, unknown vectors/RAM; physical caller, '
        'clock/reset/wire timing, complete erase/program/interruption/recovery remain unqualified.')
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original cold BDC upload and native LDX/LDA knowledge through strict FSTAT refusal.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
