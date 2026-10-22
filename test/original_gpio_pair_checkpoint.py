# SPDX-License-Identifier: GPL-3.0-only
"""Check joint native pair recovery with detached pad-source ownership."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
MATRIX = tuple((order, phase, cut) for order in (0, 1) for phase in (0, 18) for cut in range(3))


def validate(report, trace):
    lines = report.splitlines()
    sections = re.findall(r'^CASE order=(\d+) phase=(\d+) cut=(\d+)\n([\s\S]*?)(?=^CASE |\Z)',
                          trace, re.M)
    if len(lines) != len(MATRIX) or len(sections) != len(MATRIX):
        raise ValueError('CASE incomplete original pair checkpoint matrix')
    for line, section, expected in zip(lines, sections, MATRIX, strict=True):
        order, phase, cut = expected
        if tuple(map(int, section[:3])) != expected:
            raise ValueError('CASE changed checkpoint order/phase/cut')
        if fields(line) != dict(order=order, phase=phase, cut=cut, returns=3, restores=3, faults=1,
                                detached=6, **{'board-rejections': 3, 'signature': 127,
                                              'nvm-unchanged': 1, 'program-unchanged': 1}):
            raise ValueError('CASE missing native returns, recovery or retained source/NVM')
        body = section[3]
        captures = re.findall(r'^CAPTURE (.*)$', body, re.M)
        if len(captures) != 1:
            raise ValueError('CAPTURE missing executed boundary')
        capture = fields(captures[0])
        if (set(capture) != {'writes', 'conflicts', 'mcu-mask', 'source-loads'}
                or (cut == 0 and (capture['writes'] != 2 or capture['source-loads'] != 0))
                or (cut == 1 and (capture['writes'] < 3 or not capture['conflicts']
                                 or capture['mcu-mask'] != 255))
                or (cut == 2 and (capture['writes'] != 4 or capture['mcu-mask'] != 0))):
            raise ValueError('CAPTURE incorrect native write/conflict/release boundary')
        baselines = re.findall(r'^BASELINE_BEGIN\n([\s\S]*?)^BASELINE_END$', body, re.M)
        replays = re.findall(r'^REPLAY_BEGIN pass=(\d+)\n([\s\S]*?)^REPLAY_END$', body, re.M)
        if (len(baselines) != 1 or [row[0] for row in replays] != ['0', '1']
                or any(row[1] != baselines[0] for row in replays)):
            raise ValueError('REPLAY native timestamped suffix differs')
        baseline = baselines[0]
        source = [fields(row) for row in baseline.splitlines()
                  if row.startswith('SOURCE_REGISTER ')]
        if source != [dict(address=0x76c, x0=127)] * (1 - capture['source-loads']):
            raise ValueError('SOURCE original signature load evidence differs')
        for row in baseline.splitlines():
            if row.startswith('PAD_QUERY '):
                sample = fields(row)
                if (sample['known'] != sample['requested'] or sample['conflicts']
                        or sample['levels'] & ~sample['known']):
                    raise ValueError('PAD healthy continuation consumed unknown voltage')
        faults = re.findall(r'^FAULT_BEGIN\n([\s\S]*?)^FAULT_END$', body, re.M)
        if len(faults) != 1:
            raise ValueError('FAULT missing directed native unknown input')
        fault = faults[0]
        expected_device = 'DSP' if cut == 0 else 'MCU'
        if ('Unknown ' + expected_device not in fault or 'pc=' not in fault
                or ('omr=' if cut == 0 else 'ccr=') not in fault):
            raise ValueError('FAULT missing native consumer/device/context')
        unknown = [fields(row) for row in fault.splitlines() if row.startswith('PAD_QUERY ')]
        if not any(row['known'] != row['requested'] for row in unknown):
            raise ValueError('FAULT lost unresolved native pad query')
    return dict(cases=12, native_returns=36, joint_restores=36, native_unknown_faults=12,
                detached_source_rejections=72, board_rejections=36, signature=127)


def negative_controls(report, trace):
    controls = [
        (report.replace('signature=127', 'signature=126', 1), trace),
        (report.replace('detached=6', 'detached=0', 1), trace),
        (report.replace('board-rejections=3', 'board-rejections=0', 1), trace),
        (report, trace.replace('CAPTURE writes=2', 'CAPTURE writes=0', 1)),
        (report, trace.replace('REPLAY_BEGIN pass=0\n', 'REPLAY_BEGIN pass=0\nCORRUPTED\n', 1)),
        (report, trace.replace('Unknown DSP', 'Known DSP', 1)),
    ]
    for changed_report, changed_trace in controls:
        if (changed_report, changed_trace) == (report, trace):
            raise ValueError('Missing fixed native checkpoint oracle corruption')
        try:
            validate(changed_report, changed_trace)
        except ValueError:
            continue
        raise ValueError('Checkpoint oracle accepted fixed corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original pair checkpoints: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(report.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, check=False, timeout=270)
    diagnostics = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostics.splitlines()):
        raise ValueError('Original pair checkpoint execution failed: ' + diagnostics)
    text, pin_trace = report.read_text(), trace.read_text()
    totals = validate(text, pin_trace)
    controls = negative_controls(text, pin_trace)
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Original signature request in declared caller/RAM/clock/direct-link contexts. '
              'Three native event boundaries; actual unknown-read quarantine, detached callbacks, '
              'fresh reattachment and exact joint native suffix/state/time replay. '
              'No physical recovery, board-voltage safety or complete startup proof.'), indent=2) + '\n')
    print('Verified12 joint original snapshot contexts,12 native unknown-input faults and36 returns.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
