# SPDX-License-Identifier: GPL-3.0-only
"""Verify original Sound notification through native SCI IRQ and serial pads."""
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
MATRIX = tuple((policy, reject, order) for policy in (1, 3) for reject in (0, 1) for order in (0, 1))


def rows(body, label):
    return [fields(line) for line in body.splitlines() if line.startswith(label + ' ')]


def validate(report, trace):
    sections = re.findall(r'^CASE policy=(\d+) reject=(\d+) order=(\d+)\n([\s\S]*?)(?=^CASE |\Z)',
                          trace, re.M)
    lines = report.splitlines()
    if len(lines) != 8 or len(sections) != 8:
        raise ValueError('CASE incomplete native notification matrix')
    queries = unresolved = interrupts = 0
    for line, section, context in zip(lines, sections, MATRIX, strict=True):
        policy, reject, order = context
        if tuple(map(int, section[:3])) != context:
            raise ValueError('CASE changed notification order')
        result = fields(line)
        variable = ('queries', 'unresolved-events', 'sci-irqs', 'cycles')
        fixed = {k: v for k, v in result.items() if k not in variable}
        if (fixed != dict(policy=policy, reject=reject, order=order, writes=3, decisions=3,
                          stores=0 if reject else 3, bytes=9, produced=15, consumed=15,
                          **{'policy-reads': 1, 'sender-calls': 3, 'uart-calls': 3,
                             'unknown-queries': 0, 'program-unchanged': 1, 'nvm-unchanged': 1,
                             'caller-unchanged': 1, 'cache-valid': 1, 'ring-valid': 1,
                             'board-rejected': 1}) or any(not result.get(k) for k in variable)):
            raise ValueError('CASE missing native notification/SCI/preservation evidence')
        body = section[3]
        values, parameter, controller = ((17, 65, 127), 1, 27) if reject else ((0, 127, 42), 69, 62)
        expected = [b for value in values for b in (0xb0, controller, value)]
        decoded = rows(body, 'TX_BYTE')
        if (len(decoded) != 9 or any(row != dict(index=n, value=b, start=row['start'], stop=row['stop'])
                                    or row['stop'] - row['start'] != 9120
                                    for n, (row, b) in enumerate(zip(decoded, expected, strict=True)))):
            raise ValueError('WIRE actual independent MIDI decoding differs')
        starts = [row['start'] for row in decoded]
        if any(b - a < 9600 for a, b in zip(starts, starts[1:])):
            raise ValueError('WIRE serial frames overlap')
        edges = rows(body, 'TX_EDGE')
        if (not edges or edges[0]['high'] != 0
                or any(row['high'] != n % 2 for n, row in enumerate(edges))
                or any(b['cycle'] <= a['cycle'] for a, b in zip(edges, edges[1:]))):
            raise ValueError('EDGE invalid serial transition ledger')
        # Reconstruct every bit center independently from transition times,
        # including constant adjacent bits and the final stop interval.
        for row, byte in zip(decoded, expected, strict=True):
            for bit, high in enumerate((0, *((byte >> n) & 1 for n in range(8)), 1)):
                at = row['start'] + 480 + bit * 960
                preceding = [e for e in edges if e['cycle'] <= at]
                if (not preceding or preceding[-1]['high'] != high
                        or any(at - 480 < e['cycle'] < at + 480 for e in edges)):
                    raise ValueError('EDGE original TX bit level/width differs')
        writes = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x00cf (0x[0-9a-f]+)$', body, re.M)]
        if writes != expected:
            raise ValueError('UART actual original SCI data writes differ')
        if len(re.findall(r'^M \d+ 0xb5c1$', body, re.M)) != result['sci-irqs']:
            raise ValueError('IRQ actual original SCI entries differ')
        if len(re.findall(r'^M \d+ 0xb352$', body, re.M)) != 3:
            raise ValueError('SENDER actual original routing calls differ')
        if re.search(r'^M \d+ 0xb3ed$', body, re.M):
            raise ValueError('USB ordinary selector3 duplicated a message')
        readbacks = (64,) * 3 if reject else values
        sources = [dict(address=0x74c, x0=policy)]
        sources += [dict(address=0x936 + parameter, x0=value) for value in readbacks]
        if rows(body, 'SOURCE_REGISTER') != sources:
            raise ValueError('SOURCE actual policy and sound register reads differ')
        if rows(body, 'RESULT') != [dict(index=n, value=value) for n, value in enumerate(readbacks)]:
            raise ValueError('RESULT original caller readback differs')
        pads = rows(body, 'PAD_QUERY')
        if len(pads) != result['queries'] or any(p['known'] != p['requested'] or p['conflicts']
                                                for p in pads):
            raise ValueError('PAD actual consumer read is unresolved')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET unresolved notification intervals differ')
        queries += result['queries']
        unresolved += result['unresolved-events']
        interrupts += result['sci-irqs']
    return dict(streams=8, sound_attempts=24, accepted_stores=12, rejected_writes=12,
                native_sound_readbacks=24, policy_reads=8, notifications=24, serial_bytes=72,
                native_sci_interrupts=interrupts, published_consumed_bytes=120,
                known_pad_queries=queries, unresolved_queries=0, unresolved_events=unresolved,
                board_rejections=8)


def negative_controls(report, trace):
    controls = [
        (report.replace('bytes=9', 'bytes=8', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report.replace('ring-valid=1', 'ring-valid=0', 1), trace),
        (report, trace.replace('TX_BYTE index=0 value=176', 'TX_BYTE index=0 value=177', 1)),
        (report, trace.replace('TX_EDGE cycle=', 'LOST_EDGE cycle=', 1)),
        (report, trace.replace('SOURCE_REGISTER address=0x074c x0=0x0001',
                               'SOURCE_REGISTER address=0x074c x0=0x0000', 1)),
        (report, trace.replace('RESULT index=0 value=0', 'RESULT index=0 value=1', 1)),
    ]
    for altered_report, altered_trace in controls:
        if (altered_report, altered_trace) == (report, trace):
            raise ValueError('Missing fixed notification corruption')
        try:
            validate(altered_report, altered_trace)
        except ValueError:
            continue
        raise ValueError('Notification oracle accepted corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original notification: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(report.resolve()),
                             str(trace.resolve())], capture_output=True, text=True, timeout=570, check=False)
    diagnostics = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostics.splitlines()):
        raise ValueError('Original notification failed: ' + diagnostics)
    totals = validate(report.read_text(), trace.read_text())
    controls = negative_controls(report.read_text(), trace.read_text())
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Declared original Sound write/read and notification with policies1/3; mapped CC '
              'uses original UART queue, real SCI IRQ/RTI and observed PR3 NRZ at nominal31250 baud. '
              'Rejected Octave writes still notify the incoming value. No USB mirroring, genuine '
              'voice allocation/audio, full initialization, real device profile or physical timing.'), indent=2) + '\n')
    print('Verified24 original CC notifications and72 native SCI serial bytes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
