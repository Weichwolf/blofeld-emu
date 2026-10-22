# SPDX-License-Identifier: GPL-3.0-only
"""Check a continuous native original request stream with an executed own caller."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import ADDRESSES, fields
from original_mcu_request_release import PACKETS

ROOT = Path(__file__).resolve().parents[1]
ORDER = tuple(range(8)) + tuple(reversed(range(8)))
VALUES = (0, 1, 7, 63, 64, 126, 127, 127)
MATRIX = ((0, 0), (0, 18), (1, 0), (1, 18))


def validate(report, trace):
    lines = report.splitlines()
    sections = re.findall(r'^CASE order=(\d+) phase=(\d+)\n([\s\S]*?)(?=^CASE |\Z)', trace, re.M)
    if len(lines) != 4 or len(sections) != 4:
        raise ValueError('CASE incomplete continuous request matrix')
    queries = 0
    unresolved = 0
    for line, section, (order, phase) in zip(lines, sections, MATRIX, strict=True):
        if tuple(map(int, section[:2])) != (order, phase):
            raise ValueError('CASE changed continuous order/phase')
        result = fields(line)
        fixed = {key: value for key, value in result.items()
                 if key not in ('queries', 'unresolved-events')}
        if (fixed != dict(order=order, phase=phase, requests=16,
                          **{'source-loads': 16, 'data-writes': 64, 'unknown-queries': 0,
                             'nvm-unchanged': 1, 'program-unchanged': 1, 'caller-unchanged': 1,
                             'sources-unchanged': 1, 'board-rejected': 1})
                or not result.get('queries') or not result.get('unresolved-events')):
            raise ValueError('CASE missing sequential execution/preservation/board rejection')
        body = section[2]
        if len(re.findall(r'^MCU_CLOCK ', body, re.M)) != 1 or len(re.findall(r'^DSP_CLOCK ', body, re.M)) != 1:
            raise ValueError('PREFIX each stream must initialize the native clocks only once')
        sources = [fields(row) for row in body.splitlines() if row.startswith('SOURCE_REGISTER ')]
        if sources != [dict(address=ADDRESSES[n], x0=VALUES[n]) for n in ORDER]:
            raise ValueError('SOURCE actual original source-load sequence differs')
        returns = [fields(row) for row in body.splitlines() if row.startswith('RESULT ')]
        if len(returns) != 16:
            raise ValueError('RESULT incomplete native return sequence')
        previous_mcu = previous_dsp = 0
        for n, (row, selected) in enumerate(zip(returns, ORDER, strict=True)):
            if (row['index'] != n or row['value'] != VALUES[selected] or row['address'] != 0x2500 + n
                    or row['mcu-cycles'] <= previous_mcu or row['dsp-cycles'] <= previous_dsp):
                raise ValueError('RESULT changed value/order/address or reset native time')
            previous_mcu, previous_dsp = row['mcu-cycles'], row['dsp-cycles']
        writes = [(int(ticks), int(pc, 16), int(address, 16), int(value, 16))
                  for ticks, pc, address, value in
                  re.findall(r'^M-W (\d+) (0x[0-9a-f]+) (0x[0-9a-f]+) (0x[0-9a-f]+)$', body, re.M)]
        wire = [value for _, _, address, value in writes if address == 0x260]
        if wire != [value for n in ORDER for value in PACKETS[n]]:
            raise ValueError('PACKET original continuous request framing differs')
        slots = [fields(row) for row in body.splitlines() if row.startswith('CALLER_SLOT ')]
        if slots != [dict(pc=0x1400 + (17 * n) + 14, address=0x2500 + n, size=1)
                     for n in range(16)]:
            raise ValueError('CALLER results require the independently encoded native write slots')
        pads = [fields(row) for row in body.splitlines() if row.startswith('PAD_QUERY ')]
        if len(pads) != result['queries']:
            raise ValueError('PAD incomplete continuous query audit')
        for sample in pads:
            if (sample['known'] != sample['requested'] or sample['conflicts']
                    or sample['levels'] & ~sample['known']):
                raise ValueError('PAD native stream consumed unresolved voltage')
        nets = re.findall(r'^NET_UNQUALIFIED ', body, re.M)
        if len(nets) != result['unresolved-events']:
            raise ValueError('NET continuous unresolved audit differs')
        queries += len(pads)
        unresolved += len(nets)
    return dict(streams=4, requests=64, native_returns=64, original_source_loads=64,
                original_data_writes=256, known_pad_queries=queries, unresolved_queries=0,
                unresolved_events=unresolved, board_rejections=4)


def negative_controls(report, trace):
    controls = [
        (report.replace('requests=16', 'requests=15', 1), trace),
        (report.replace('sources-unchanged=1', 'sources-unchanged=0', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('RESULT index=0 value=0 ', 'RESULT index=0 value=1 ', 1)),
        (report, re.sub(r'^(SOURCE_REGISTER .*? x0=)0x[0-9a-f]+', r'\g<1>0x0001', trace,
                        count=1, flags=re.M)),
        (report, re.sub(r'^(M-W \d+ 0x[0-9a-f]+ 0x0260 )0x[0-9a-f]+', r'\g<1>0x00f5', trace,
                        count=1, flags=re.M)),
        (report, re.sub(r'^(CALLER_SLOT pc=)0x[0-9a-f]+', r'\g<1>0x0001', trace,
                        count=1, flags=re.M)),
        (report, re.sub(r'^(PAD_QUERY .*? known=)0x[0-9a-f]+', r'\g<1>0x0000', trace,
                        count=1, flags=re.M)),
    ]
    for changed_report, changed_trace in controls:
        if (changed_report, changed_trace) == (report, trace):
            raise ValueError('Missing fixed native sequence oracle corruption')
        try:
            validate(changed_report, changed_trace)
        except ValueError:
            continue
        raise ValueError('Sequence oracle accepted fixed corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original GPIO sequence: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(report.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, check=False, timeout=210)
    diagnostics = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostics.splitlines()):
        raise ValueError('Original GPIO sequence execution failed: ' + diagnostics)
    text, pin_trace = report.read_text(), trace.read_text()
    totals = validate(text, pin_trace)
    controls = negative_controls(text, pin_trace)
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Four continuously scheduled native MCU/DSP request streams. An executed own caller '
              'supplies sixteen argument frames and stores actual original replies per stream; '
              'no per-request CPU, GPIO, IRQ, clock or DSP reinitialization. Declared initial '
              'caller/RAM/source/clock/direct-link setup; no complete boot or physical board proof.'), indent=2) + '\n')
    print('Verified64 actual original request returns in4 continuous native streams.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
