# SPDX-License-Identifier: GPL-3.0-only
"""Check original MCU F5 writes through the real DSP parser and native F4 reads."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
VALUES = (0, 1, 7, 63, 64, 126, 127, 42)
MATRIX = ((0, 0), (0, 18), (1, 0), (1, 18))


def validate(report, trace):
    lines = report.splitlines()
    sections = re.findall(r'^CASE order=(\d+) phase=(\d+)\n([\s\S]*?)(?=^CASE |\Z)', trace, re.M)
    if len(lines) != 4 or len(sections) != 4:
        raise ValueError('CASE incomplete native parameter matrix')
    queries = unresolved = 0
    for line, section, (order, phase) in zip(lines, sections, MATRIX, strict=True):
        if tuple(map(int, section[:2])) != (order, phase):
            raise ValueError('CASE changed parameter order/phase')
        result = fields(line)
        fixed = {key: value for key, value in result.items()
                 if key not in ('queries', 'unresolved-events')}
        if (fixed != dict(order=order, phase=phase, writes=8, readbacks=8, produced=40, consumed=40,
                          **{'data-writes': 72, 'unresolved-queries': 0, 'program-unchanged': 1,
                             'nvm-unchanged': 1, 'caller-unchanged': 1, 'ring-retained': 1,
                             'board-rejected': 1})
                or not result.get('queries') or not result.get('unresolved-events')):
            raise ValueError('CASE missing actual native write/parser/readback or rejection')
        body = section[2]
        queued = [fields(row) for row in body.splitlines() if row.startswith('QUEUE_BYTE ')]
        received = [byte for value in VALUES for byte in (245, 64, 66, value, 247)]
        if queued != [dict(index=n, byte=byte, cursor=0x2d01 + n)
                      for n, byte in enumerate(received)]:
            raise ValueError('QUEUE actual original sender/IRQ publication differs')
        consumed = [fields(row) for row in body.splitlines() if row.startswith('CONSUMED ')]
        if consumed != [dict(count=n + 1, cursor=0x2d01 + n) for n in range(40)]:
            raise ValueError('CONSUMED actual original parser cursor differs')
        stores = [fields(row) for row in body.splitlines() if row.startswith('PARAMETER_STORE ')]
        if stores != [dict(index=n, pc=0x8d9, address=0xe818, value=value)
                      for n, value in enumerate(VALUES)]:
            raise ValueError('STORE original parsed write instruction effect differs')
        sources = [fields(row) for row in body.splitlines() if row.startswith('SOURCE_REGISTER ')]
        if sources != [dict(address=0xe818, x0=value) for value in VALUES]:
            raise ValueError('SOURCE actual original native readback differs')
        returns = [fields(row) for row in body.splitlines() if row.startswith('RESULT ')]
        if len(returns) != 8:
            raise ValueError('RESULT missing actual MCU replies')
        previous_mcu = previous_dsp = 0
        for n, (row, value) in enumerate(zip(returns, VALUES, strict=True)):
            if (row['index'] != n or row['value'] != value or row['address'] != 0x2500 + n
                    or row['mcu-cycles'] <= previous_mcu or row['dsp-cycles'] <= previous_dsp):
                raise ValueError('RESULT native return/time differs')
            previous_mcu, previous_dsp = row['mcu-cycles'], row['dsp-cycles']
        slots = [fields(row) for row in body.splitlines() if row.startswith('CALLER_SLOT ')]
        if slots != [dict(pc=0x1400 + (35 * n) + 32, address=0x2500 + n, size=1)
                     for n in range(8)]:
            raise ValueError('CALLER actual native RAM store slot differs')
        wire = [int(value, 16) for value in
                re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)$', body, re.M)]
        if wire != [byte for value in VALUES for byte in (245, 64, 66, value, 247, 244, 64, 194, 66)]:
            raise ValueError('PACKET independent original F5/F4 framing differs')
        pads = [fields(row) for row in body.splitlines() if row.startswith('PAD_QUERY ')]
        if len(pads) != result['queries']:
            raise ValueError('PAD incomplete query audit')
        for sample in pads:
            if (sample['known'] != sample['requested'] or sample['conflicts']
                    or sample['levels'] & ~sample['known']):
                raise ValueError('PAD native parameter chain consumed unknown voltage')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET incomplete unresolved interval audit')
        queries += len(pads)
        unresolved += result['unresolved-events']
    return dict(streams=4, original_parameter_stores=32, original_readbacks=32,
                original_published_bytes=160, original_consumed_bytes=160, original_data_writes=288,
                known_pad_queries=queries, unresolved_queries=0, unresolved_events=unresolved,
                board_rejections=4)


def negative_controls(report, trace):
    controls = [
        (report.replace('program-unchanged=1', 'program-unchanged=0', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('QUEUE_BYTE index=0 byte=245', 'QUEUE_BYTE index=0 byte=244', 1)),
        (report, trace.replace('CONSUMED count=1 ', 'CONSUMED count=0 ', 1)),
        (report, re.sub(r'^(PARAMETER_STORE .*?value=)0', r'\g<1>1', trace, count=1, flags=re.M)),
        (report, re.sub(r'^(SOURCE_REGISTER .*?x0=)0x[0-9a-f]+', r'\g<1>0x0001', trace,
                        count=1, flags=re.M)),
        (report, trace.replace('RESULT index=0 value=0 ', 'RESULT index=0 value=1 ', 1)),
        (report, re.sub(r'^(CALLER_SLOT pc=)0x[0-9a-f]+', r'\g<1>0x0000', trace,
                        count=1, flags=re.M)),
        (report, re.sub(r'^(M-W \d+ 0x[0-9a-f]+ 0x0260 )0x[0-9a-f]+', r'\g<1>0x00f4', trace,
                        count=1, flags=re.M)),
    ]
    for changed_report, changed_trace in controls:
        if (changed_report, changed_trace) == (report, trace):
            raise ValueError('Missing fixed parameter oracle corruption')
        try:
            validate(changed_report, changed_trace)
        except ValueError:
            continue
        raise ValueError('Parameter oracle accepted fixed corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original GPIO parameter write/read: local firmware absent')
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
        raise ValueError('Original parameter execution failed: ' + diagnostics)
    text, pin_trace = report.read_text(), trace.read_text()
    totals = validate(text, pin_trace)
    controls = negative_controls(text, pin_trace)
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Own native MCU caller invokes original Multi-global field2 F5 writes and F4 reads. '
              'Original GPIO producer, secondary ring, parser and parameter writer execute on '
              'continuous nominally scheduled processors. Declared initial full runtime records, '
              'caller/RAM/pin/clock state; no full initialization, other write classes, actual '
              'host MIDI/audio, physical timing or board-safety proof.'), indent=2) + '\n')
    print('Verified32 original parsed parameter writes and32 genuine native readbacks.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
