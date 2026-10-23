# SPDX-License-Identifier: GPL-3.0-only
"""Check original notification, UART mirror and real USB endpoint retry/ACK."""
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_notification import rows
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
MATRIX = tuple((reject, order) for reject in (0, 1) for order in (0, 1))


def validate(report, trace):
    sections = re.findall(r'^CASE reject=(\d+) order=(\d+)\n([\s\S]*?)(?=^CASE |\Z)', trace, re.M)
    lines = report.splitlines()
    if len(lines) != 4 or len(sections) != 4:
        raise ValueError('CASE incomplete original USB notification matrix')
    queries = unresolved = sci = 0
    for line, section, context in zip(lines, sections, MATRIX, strict=True):
        reject, order = context
        if tuple(map(int, section[:2])) != context:
            raise ValueError('CASE changed USB notification order')
        result = fields(line)
        fixed = {k: v for k, v in result.items() if k not in ('queries', 'unresolved-events', 'sci-irqs')}
        if (fixed != dict(reject=reject, order=order, stores=0 if reject else 3, readbacks=3,
                          notifications=3, retries=1,
                          **{'serial-bytes': 9, 'usb-irqs': 2, 'usb-events': 3, 'usb-packets': 2,
                             'mirror-calls': 3, 'policy-reads': 1, 'unknown-queries': 0,
                             'program-unchanged': 1, 'nvm-unchanged': 1, 'caller-unchanged': 1,
                             'cache-valid': 1, 'rings-valid': 1, 'identity-valid': 1,
                             'final-nak': 1, 'board-rejected': 1})
                or any(not result.get(k) for k in ('queries', 'unresolved-events', 'sci-irqs'))):
            raise ValueError('CASE missing native USB/serial/guard evidence')
        body = section[2]
        values, parameter, cc = ((17, 65, 127), 1, 27) if reject else ((0, 127, 42), 69, 62)
        serial = [b for v in values for b in (176, cc, v)]
        usb = [b for v in values for b in (11, 176, cc, v)]
        transfers = [dict(index=0, attempt=0, size=4, data1=0),
                     dict(index=0, attempt=1, size=4, data1=0),
                     dict(index=1, attempt=0, size=8, data1=1)]
        if rows(body, 'USB_IN') != transfers:
            raise ValueError('USB changed packet boundary/retry/DATA PID')
        expected_bytes = []
        for packet, attempt, data in ((0, 0, usb[:4]), (0, 1, usb[:4]), (1, 0, usb[4:])):
            expected_bytes += [dict(packet=packet, attempt=attempt, index=n, value=b)
                               for n, b in enumerate(data)]
        if rows(body, 'USB_BYTE') != expected_bytes:
            raise ValueError('USB original mapped CC bytes or retry identity differ')
        if rows(body, 'USB_ACK') != [dict(index=0), dict(index=1)]:
            raise ValueError('ACK missing actual host completion')
        if rows(body, 'USB_FINAL_NAK') != [dict(value=1)]:
            raise ValueError('NAK original endpoint not empty')
        decoded = rows(body, 'TX_BYTE')
        if len(decoded) != 9 or any(row['index'] != n or row['value'] != b or
                                    row['stop'] - row['start'] != 9120
                                    for n, (row, b) in enumerate(zip(decoded, serial, strict=True))):
            raise ValueError('SCI native serial output differs from mirrored USB')
        writes = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x00cf (0x[0-9a-f]+)$', body, re.M)]
        if writes != serial:
            raise ValueError('SCI actual original data writes differ')
        for pc, count in ((0xb4aa, 3), (0xb3ed, 3), (0xbe6b, 2), (0xb5c1, result['sci-irqs'])):
            if len(re.findall(rf'^M \d+ 0x{pc:04x}$', body, re.M)) != count:
                raise ValueError('NATIVE original mirror/queue/ISR execution differs')
        readbacks = (64,) * 3 if reject else values
        sources = [dict(address=0x74c, x0=3)]
        sources += [dict(address=0x936 + parameter, x0=v) for v in readbacks]
        if rows(body, 'SOURCE_REGISTER') != sources:
            raise ValueError('SOURCE genuine DSP policy/readback differs')
        if rows(body, 'RESULT') != [dict(index=n, value=v) for n, v in enumerate(readbacks)]:
            raise ValueError('RESULT notification must not replace native acceptance readback')
        pads = rows(body, 'PAD_QUERY')
        if len(pads) != result['queries'] or any(p['known'] != p['requested'] or p['conflicts'] for p in pads):
            raise ValueError('PAD unresolved native notification input')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET missing explicit unresolved interval')
        queries += result['queries']
        unresolved += result['unresolved-events']
        sci += result['sci-irqs']
    return dict(streams=4, original_sound_attempts=12, accepted_stores=6, rejected_writes=6,
                native_readbacks=12, usb_events=12, usb_bytes=48, endpoint_packets=8,
                lost_ack_retries=4, native_usb_interrupts=8, native_sci_interrupts=sci,
                serial_bytes=36, known_pad_queries=queries, unresolved_queries=0,
                unresolved_events=unresolved, board_rejections=4)


def negative_controls(report, trace):
    controls = [
        (report.replace('retries=1', 'retries=0', 1), trace),
        (report.replace('identity-valid=1', 'identity-valid=0', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('USB_IN index=0 attempt=1 size=4 data1=0',
                               'USB_IN index=0 attempt=1 size=4 data1=1', 1)),
        (report, trace.replace('USB_BYTE packet=0 attempt=1 index=0 value=11',
                               'USB_BYTE packet=0 attempt=1 index=0 value=10', 1)),
        (report, trace.replace('USB_ACK index=0', 'LOST_ACK index=0', 1)),
        (report, trace.replace('USB_FINAL_NAK value=1', 'USB_FINAL_NAK value=0', 1)),
        (report, trace.replace('RESULT index=0 value=0', 'RESULT index=0 value=1', 1)),
    ]
    for changed_report, changed_trace in controls:
        if (changed_report, changed_trace) == (report, trace):
            raise ValueError('Missing fixed USB notification corruption')
        try:
            validate(changed_report, changed_trace)
        except ValueError:
            continue
        raise ValueError('USB notification oracle accepted corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original USB notification: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(report.resolve()),
                             str(trace.resolve())], capture_output=True, text=True, timeout=570, check=False)
    diagnostic = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostic)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostic.splitlines()):
        raise ValueError('Original USB notification failed: ' + diagnostic)
    totals = validate(report.read_text(), trace.read_text())
    controls = negative_controls(report.read_text(), trace.read_text())
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, original_enumeration=False, fuzz_execution=False,
        physical_traffic=False, scope='Original Sound writes/notifications create UART bytes; own '
        'native caller invokes original mirror/USB producer. Real modeled IN/retry/ACK drives original '
        'USB ISR/flush while native SCI drains the same bytes. Declared own endpoint/decoder setup '
        'and full-record preparation; no original enumeration, host-visible device, physical timing, '
        'voice/audio or complete boot.'), indent=2) + '\n')
    print('Verified12 original USB-MIDI notification events and4 lost-ACK retries.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
