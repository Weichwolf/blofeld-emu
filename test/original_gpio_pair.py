# SPDX-License-Identifier: GPL-3.0-only
"""Check native original request returns without certifying unresolved board pads."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
ADDRESSES = (0x1200, 0xe816, 0xe836, 0x720, 0x1b7f, 0xe855, 0xe8f6, 0x76c)
MATRIX = tuple((case, order, phase) for case in range(8)
               for order in (0, 1) for phase in (0, 18))


def fields(line):
    return {key: int(value, 0) for key, value in
            re.findall(r'([\w-]+)=(0x[0-9a-f]+|\d+)', line)}


def validate(report, trace):
    lines = report.splitlines()
    sections = re.findall(r'^CASE (\d+) (\d+) (\d+)\n([\s\S]*?)(?=^CASE |\Z)', trace, re.M)
    if len(lines) != len(MATRIX) or len(sections) != len(MATRIX):
        raise ValueError('CASE incomplete original pair matrix')
    queries = 0
    unresolved = 0
    for line, section, expected in zip(lines, sections, MATRIX, strict=True):
        case, order, phase = expected
        if tuple(map(int, section[:3])) != expected:
            raise ValueError('CASE changed original pair ordering')
        tokens = line.split()
        if (tokens[:4] != ['RETURN', 'board-qualification=rejected',
                          'nvm-unchanged=1', 'program-unchanged=1']
                or tuple(map(int, tokens[4:7])) != expected):
            raise ValueError('RETURN missing native return, preservation or board rejection')
        result = fields(' '.join(tokens[7:]))
        expected_keys = {'mpc', 'dpc', 'ms', 'ds', 'source-register-checks',
                         'unresolved-events', 'queries', 'unresolved-queries', 'value'}
        value = 127 if case == 7 else 42
        if (set(result) != expected_keys or result['mpc'] != 0x1403
                or not result['dpc'] or not result['ms'] or not result['ds']
                or result['source-register-checks'] != 1 or result['value'] != value
                or not result['queries'] or result['unresolved-queries']):
            raise ValueError('RETURN native source, value or known-read evidence differs')
        body = section[3].splitlines()
        source = [fields(row) for row in body if row.startswith('SOURCE_REGISTER ')]
        if source != [dict(address=ADDRESSES[case], x0=value)]:
            raise ValueError('SOURCE actual native source register differs')
        pads = [fields(row) for row in body if row.startswith('PAD_QUERY ')]
        nets = [fields(row) for row in body if row.startswith('NET_UNQUALIFIED ')]
        if len(pads) != result['queries']:
            raise ValueError('PAD incomplete query audit')
        for sample in pads:
            if (sample['known'] != sample['requested'] or sample['conflicts']
                    or sample['levels'] & ~sample['known']):
                raise ValueError('PAD original firmware consumed an unresolved voltage')
        if len(nets) != result['unresolved-events'] or not nets:
            raise ValueError('NET incomplete unresolved-interval audit')
        for sample in nets:
            if (sample['known'] == 255 or sample['levels'] & ~sample['known']
                    or sample['known'] & sample['conflicts']):
                raise ValueError('NET unknown/conflict evidence differs')
        queries += len(pads)
        unresolved += len(nets)
    return dict(cases=len(lines), original_returns=len(lines), source_register_checks=len(lines),
                known_pad_queries=queries, unresolved_pad_queries=0,
                unresolved_events=unresolved, board_rejections=len(lines))


def negative_controls(report, trace):
    controls = [
        (report.replace('board-qualification=rejected', 'board-qualification=accepted', 1), trace),
        (report.replace('nvm-unchanged=1', 'nvm-unchanged=0', 1), trace),
        (report.replace('unresolved-queries=0', 'unresolved-queries=1', 1), trace),
        (report.replace('value=42', 'value=43', 1), trace),
        (report, re.sub(r'^(SOURCE_REGISTER .*? x0=)0x[0-9a-f]+', r'\g<1>0x0000',
                        trace, count=1, flags=re.M)),
        (report, re.sub(r'^(PAD_QUERY .*? known=)0x[0-9a-f]+', r'\g<1>0x0000',
                        trace, count=1, flags=re.M)),
        (report, re.sub(r'^(NET_UNQUALIFIED .*? known=)0x[0-9a-f]+', r'\g<1>0x00ff',
                        trace, count=1, flags=re.M)),
        (report, trace.replace('CASE 0 0 0\n', 'CASE 1 0 0\n', 1)),
    ]
    for changed_report, changed_trace in controls:
        if (changed_report, changed_trace) == (report, trace):
            raise ValueError('Missing fixed original pair oracle control')
        try:
            validate(changed_report, changed_trace)
        except ValueError:
            continue
        raise ValueError('Original pair oracle accepted a fixed evidence corruption')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original GPIO pair: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace = report.with_suffix('.trace')
    receipt = report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(report.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, check=False, timeout=210)
    diagnostics = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostics.splitlines()):
        raise ValueError('Original GPIO pair failed: ' + diagnostics)
    text, pin_trace = report.read_text(), trace.read_text()
    totals = validate(text, pin_trace)
    controls = negative_controls(text, pin_trace)
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Unchanged original MCU/DSP request handlers in declared caller/RAM, GPIO and '
              'nominal clock contexts. Includes the original signature data record. All native '
              'pad queries are known; unresolved intermediate drivers reject board qualification. '
              'No startup, actual board timing, physical wiring or flash-safety proof.'), indent=2) + '\n')
    print(f'Verified {totals["original_returns"]} native original request returns; '
          'all unresolved board profiles remain rejected.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
