# SPDX-License-Identifier: GPL-3.0-only
"""Check complete original SNDP generation and unmapped-policy suppression."""
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_notification import rows
from original_gpio_pair import fields
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

ROOT = Path(__file__).resolve().parents[1]
MATRIX = tuple((policy, parameter, order) for policy, parameter in
               ((0, 70), (1, 70), (2, 69), (2, 70), (3, 70)) for order in (0, 1))


def spare_expectations(firmware):
    midi, _ = load_input(firmware)
    messages, _ = read_smf(midi)
    data = decode_block(split_blocks(messages)[4])[1]
    _, offset = hc12_object(data)
    rom = data[offset:offset + 0x8000]
    table = rom[0xfb63 - 0x8000:0xfb63 - 0x8000 + 73]
    base, remaining, position, mapped = 0, 70, 0, 127
    # Independent byte/run arithmetic over local reference data. Even-policy
    # spare slots retain the getter's mask/raw value; odd fallback retains
    # the mapper's saved mapped-count/run. Detailed stack derivation stays local.
    while position + 1 < len(table):
        run = table[position]
        position += 1
        if table[position] & 128:
            base = table[position] & 127
            position += 1
        skipped = run & 15 if base else run
        if base:
            mapped = run >> 4
        if remaining < skipped:
            return {2: (127, 2), 3: (mapped, run)}
        remaining -= skipped
        if base:
            if remaining < mapped:
                raise ValueError('Independent map unexpectedly assigns a CC to parameter70')
            remaining -= mapped
            base = (base + mapped) & 255
    raise ValueError('Independent controller run model did not find the unmapped field')


def validate(report, trace, expected_spares):
    sections = re.findall(r'^CASE policy=(\d+) parameter=(\d+) order=(\d+)\n([\s\S]*?)(?=^CASE |\Z)',
                          trace, re.M)
    lines = report.splitlines()
    if len(lines) != 10 or len(sections) != 10:
        raise ValueError('CASE incomplete original SysEx policy matrix')
    serial_total = queries = unresolved = sci_irqs = 0
    spares = []
    for line, section, context in zip(lines, sections, MATRIX, strict=True):
        policy, parameter, order = context
        enabled, device = policy >= 2, (17, 42)[order]
        if tuple(map(int, section[:3])) != context:
            raise ValueError('CASE changed SysEx context ordering')
        result = fields(line)
        variable = ('raw-bytes', 'sci-irqs', 'queries', 'unresolved-events')
        fixed = {k: v for k, v in result.items() if k not in variable}
        if (fixed != dict(policy=policy, parameter=parameter, order=order, device=device,
                          stores=1, sources=3 if enabled else 2, senders=int(enabled),
                          mappers=policy & 1, produced=5, consumed=5,
                          **{'uart-calls': 4 if enabled else 0, 'usb-events': 4 if enabled else 0,
                             'usb-packets': 2 if enabled else 0, 'usb-irqs': 2 if enabled else 0,
                             'event-bytes': 16 if enabled else 0, 'unknown-queries': 0,
                             'nvm-unchanged': 1, 'program-unchanged': 1, 'caller-unchanged': 1,
                             'identity-valid': 1, 'rings-valid': 1, 'final-nak': 1,
                             'board-rejected': 1})
                or not result.get('queries') or not result.get('unresolved-events')
                or bool(result.get('sci-irqs')) != enabled):
            raise ValueError('CASE missing original policy/SysEx/native preservation evidence')
        body = section[3]
        padding = rows(body, 'SPARE')
        packet = [240, 62, 19, device, 32, 0, 0, parameter, 7, 247]
        raw, events = [], []
        if enabled:
            if len(padding) != 2 or [row['index'] for row in padding] != [10, 11]:
                raise ValueError('SPARE missing actual pre-sender caller-stack observation')
            tail = [row['value'] for row in padding]
            if tuple(tail) != expected_spares[policy]:
                raise ValueError('SPARE differs from independent getter/mapper stack expectations')
            packet += tail
            spares.append(dict(policy=policy, parameter=parameter, order=order, bytes=tail))
            for offset in range(0, 12, 3):
                chunk = packet[offset:offset + 3]
                raw += chunk[:chunk.index(255)] if 255 in chunk else chunk
                events += [5 if offset == 9 else 4, *chunk]
        elif padding:
            raise ValueError('SPARE disabled policy entered SysEx')
        if rows(body, 'PACKET_BYTE') != ([dict(index=n, value=b) for n, b in enumerate(packet)]
                                         if enabled else []):
            raise ValueError('PACKET original SNDP fields or unchanged stack bytes differ')
        expected_sources = [dict(address=0x74c, x0=policy)]
        if enabled:
            expected_sources += [dict(address=0x745, x0=device)]
        expected_sources += [dict(address=0x936 + parameter, x0=7)]
        if rows(body, 'SOURCE_REGISTER') != expected_sources:
            raise ValueError('SOURCE genuine policy/DeviceID/native readback differs')
        if rows(body, 'RESULT') != [dict(value=7)]:
            raise ValueError('RESULT original sound write/readback differs')
        wire = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)$', body, re.M)]
        expected_wire = [245, 0, parameter, 7, 247, 244, 96, 172, 44]
        if enabled:
            expected_wire += [244, 96, 165, 37]
        expected_wire += [244, 0, parameter | 128, parameter]
        if wire != expected_wire:
            raise ValueError('GPIO original writes and genuine request sequence differ')
        decoded = rows(body, 'TX_BYTE')
        if (len(decoded) != len(raw) or result['raw-bytes'] != len(raw)
                or any(row['index'] != n or row['value'] != byte or row['stop'] - row['start'] != 9120
                       for n, (row, byte) in enumerate(zip(decoded, raw, strict=True)))):
            raise ValueError('UART original raw stream/stack trailing bytes differ')
        actual_uart = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x00cf (0x[0-9a-f]+)$', body, re.M)]
        if actual_uart != raw:
            raise ValueError('UART actual original SCI writes differ from serial pads')
        transfers = [dict(index=0, size=4, data1=0), dict(index=1, size=12, data1=1)] if enabled else []
        if rows(body, 'USB_IN') != transfers:
            raise ValueError('USB original SysEx batching/DATA PID differs')
        expected_usb = []
        if enabled:
            for packet_index, data in enumerate((events[:4], events[4:])):
                expected_usb += [dict(packet=packet_index, index=n, value=b) for n, b in enumerate(data)]
        if rows(body, 'USB_BYTE') != expected_usb:
            raise ValueError('USB original SysEx CIN/padding bytes differ')
        if rows(body, 'USB_ACK') != ([dict(index=0), dict(index=1)] if enabled else []):
            raise ValueError('ACK actual original USB drain differs')
        for pc, count in ((0xd6ab, int(enabled)), (0xb352, int(enabled)),
                          (0xd5e5, policy & 1), (0xb555, 4 if enabled else 0),
                          (0xb3ed, 4 if enabled else 0), (0xbe6b, 2 if enabled else 0),
                          (0xb5c1, result['sci-irqs'])):
            if len(re.findall(rf'^M \d+ 0x{pc:04x}$', body, re.M)) != count:
                raise ValueError('NATIVE original policy/SysEx/queue/interrupt entries differ')
        pads = rows(body, 'PAD_QUERY')
        if len(pads) != result['queries'] or any(p['known'] != p['requested'] or p['conflicts'] for p in pads):
            raise ValueError('PAD native request consumed unresolved data')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET unresolved SysEx intervals omitted')
        serial_total += len(raw)
        queries += result['queries']
        unresolved += result['unresolved-events']
        sci_irqs += result['sci-irqs']
    return dict(streams=10, sound_stores_readbacks=10, suppressed_notifications=4,
                sysex_notifications=6, genuine_device_id_reads=6, usb_events=24, usb_bytes=96,
                native_usb_interrupts=12, native_sci_interrupts=sci_irqs,
                raw_serial_bytes=serial_total, logical_sysex_bytes=60, known_pad_queries=queries,
                unresolved_queries=0, unresolved_events=unresolved, board_rejections=10,
                observed_caller_spare_bytes=spares)


def negative_controls(report, trace, expected_spares):
    consistent_spare_edit = trace.replace('SPARE index=10 value=127', 'SPARE index=10 value=126', 1)
    consistent_spare_edit = consistent_spare_edit.replace('PACKET_BYTE index=10 value=127',
                                                         'PACKET_BYTE index=10 value=126', 1)
    consistent_spare_edit = consistent_spare_edit.replace('TX_BYTE index=10 value=127',
                                                         'TX_BYTE index=10 value=126', 1)
    consistent_spare_edit = consistent_spare_edit.replace('0x00cf 0x007f', '0x00cf 0x007e', 1)
    consistent_spare_edit = consistent_spare_edit.replace('USB_BYTE packet=1 index=10 value=127',
                                                         'USB_BYTE packet=1 index=10 value=126', 1)
    controls = [
        (report.replace('sources=2', 'sources=3', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('PACKET_BYTE index=3 value=17', 'PACKET_BYTE index=3 value=18', 1)),
        (report, trace.replace('USB_IN index=1 size=12 data1=1', 'USB_IN index=1 size=12 data1=0', 1)),
        (report, trace.replace('SPARE index=10 value=', 'LOST_SPARE index=10 value=', 1)),
        (report, trace.replace('USB_ACK index=0', 'LOST_ACK index=0', 1)),
        (report, trace.replace('RESULT value=7', 'RESULT value=8', 1)),
        (report, consistent_spare_edit),
    ]
    for altered_report, altered_trace in controls:
        if (altered_report, altered_trace) == (report, trace):
            raise ValueError('Missing fixed SysEx oracle corruption')
        try:
            validate(altered_report, altered_trace, expected_spares)
        except ValueError:
            continue
        raise ValueError('SysEx oracle accepted corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original SysEx policy: local firmware absent')
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
        raise ValueError('Original SysEx policy failed: ' + diagnostic)
    expected_spares = spare_expectations(firmware)
    totals = validate(report.read_text(), trace.read_text(), expected_spares)
    controls = negative_controls(report.read_text(), trace.read_text(), expected_spares)
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, original_enumeration=False, fuzz_execution=False,
        physical_traffic=False, padding_is_observed_caller_state=True,
        scope='Original Sound store/notify/real policy and DeviceID requests/SNDP chunk sender, '
              'native SCI and actual USB endpoint ACK/IRQ. Disabled and unmapped CC-only paths '
              'must suppress notifications. Explicit full-record/RAM/clock/SCI/own endpoint '
              'context; raw trailing UART stack bytes preserved, not qualified as MIDI payload. '
              'No original enumeration, host-visible device, full boot, physical timing or audio.'), indent=2) + '\n')
    print('Verified6 original SysEx notifications and4 suppressed policy cases.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
