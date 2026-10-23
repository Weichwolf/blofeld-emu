# SPDX-License-Identifier: GPL-3.0-only
"""Check native download state continuity into the original MCU error/restart path."""

import copy
import functools
import hashlib
import json
import operator
import os
from pathlib import Path
import re
import subprocess
import sys

from original_update_transport import references, LAYOUTS, validate_block
from original_dsp_download import fixture
from original_update_diagnostic import reference as error_reference, check_frame
from original_update_forward import CHUNK_PCS, MATRIX
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
FRAME_SHA256 = 'b9aa570a725f1d5fde9d9f44a9708a02788e6efe3c375fbcc6de72d7d15f1732'


def inputs(firmware, destination):
    blocks = references(firmware, destination.with_suffix('.original.packets'))
    index = 0x14477 - 0xd000
    packet_index = 1 + index // 7
    packet = bytearray(blocks[4]['packets'][packet_index])
    decoded = []
    for offset in range(6, len(packet) - 1, 8):
        flags = packet[offset]
        decoded.extend(packet[offset + 1 + n] | (((flags >> (6 - n)) & 1) << 7)
                       for n in range(7))
    begin = (index % 7) * 3
    decoded[begin:begin + 3] = [0, 0, 0]  # One declared damaged validation marker, not an opcode patch.
    packed = []
    for offset in range(0, len(decoded), 7):
        group = decoded[offset:offset + 7]
        packed.append(sum(((value >> 7) & 1) << (6 - n) for n, value in enumerate(group)))
        packed.extend(value & 127 for value in group)
    packet[6:-1] = bytes(packed)
    packet[-1] = functools.reduce(operator.xor, packet[:-1], 0)
    blocks[4]['packets'][packet_index] = bytes(packet)
    blocks[4]['words'][index] = 0
    lines = ['6']
    for block in blocks:
        lines.extend((f"{len(block['packets'])} {len(block['words'])}", ' '.join(map(str, block['words']))))
        lines.extend(str(len(packet)) + ' ' + ' '.join(map(str, packet)) for packet in block['packets'])
    destination.write_text('\n'.join(lines) + '\n')
    _, texts, base, counts = error_reference(firmware)
    frame = bytearray(base)
    header = len(frame) - 1024
    for y in range(23, 23 + 18):
        for column in range(16):
            frame[header + y * 16 + column] ^= 255
    pages = (23 + 18 - 1) // 8 - 23 // 8 + 1
    counts['writes'] += 128 * pages
    counts['reads'] += 2 * 128 * pages
    if hashlib.sha256(frame).hexdigest() != FRAME_SHA256:
        raise ValueError('Independent original-font and XOR-rectangle calculation changed')
    return blocks, texts, bytes(frame), counts


def collect(path):
    cases, current, preload = [], None, None
    tags = {'SOURCE', 'CHUNK', 'CONSUMER', 'IRQ_PAD', 'IRQ_ENTRY', 'IRQ_RETURN', 'RESULT',
            'RESET_BEGIN', 'RESET_REPLAY', 'SYSTEM_RESET', 'RESET_BOOT', 'RESET_BOOT_FAULT', 'COP_ENABLED', 'COP_REQUEST', 'ENTRY_FLAGS', 'CLEAR', 'TEXT', 'RECTANGLE', 'PREPARED', 'FAULT_STATE', 'COUPLED_FAULT', 'QUARANTINE', 'COUPLED_RESULT'}
    with path.open() as stream:
        for line in stream:
            tag = line.split(' ', 1)[0]
            if tag == 'CASE':
                current = dict(case=fields(line), preloads=[], rows={}, pads=0, nets=0, q=[], delay=0, reset_active=False, reset_dsp=0, failed_pads=[])
                cases.append(current)
                preload = None
            elif current is None:
                raise ValueError('CASE must precede all native coupled evidence')
            elif tag == 'PRELOAD':
                preload = dict(index=fields(line)['block'], rows=[])
                current['preloads'].append(preload)
            elif tag == 'PRELOAD_END':
                current['preload_end'] = fields(line)
                preload = None
            elif preload is not None:
                if tag not in ('SETUP', 'STORE', 'PREFETCH', 'PACKET', 'INTERRUPTED', 'RESULT'):
                    raise ValueError('Unexpected native preload row')
                preload['rows'].append((tag, fields(line)))
            elif tag == 'PAD_QUERY':
                row = fields(line)
                if row['levels'] & ~row['known'] or row['known'] & row['conflicts']:
                    raise ValueError('PAD native coupled sample masks are invalid')
                if row['known'] != row['requested'] or row['conflicts']:
                    if (not current['reset_active'] or 'SYSTEM_RESET' not in current['rows'] or
                            row != dict(dsp=1, requested=128, known=0, levels=0, conflicts=128)):
                        raise ValueError('PAD unexpected unqualified consumed input')
                    current['failed_pads'].append(row)
                current['pads'] += 1
            elif tag == 'NET_UNQUALIFIED':
                row = fields(line)
                if row['known'] == 255 or row['levels'] & ~row['known'] or row['known'] & row['conflicts']:
                    raise ValueError('NET unresolved interval evidence differs')
                current['nets'] += 1
            elif tag == 'M-W':
                found = re.fullmatch(r'M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)\n', line)
                if found:
                    current['q'].append(int(found[1], 16))
            elif tag == 'D':
                current['reset_dsp'] += int(current['reset_active'])
            elif tag == 'M':
                current['delay'] += int(line.rstrip().endswith(' 0x128b'))
            elif tag in tags:
                current['rows'].setdefault(tag, []).append(line.rstrip())
                if tag == 'RESET_BEGIN':
                    current['reset_active'] = True
                if tag == 'COUPLED_RESULT':
                    current['coupled_pads'], current['coupled_nets'] = current['pads'], current['nets']
                if tag == 'SYSTEM_RESET':
                    current['pulse_dsp_steps'] = current['reset_dsp']
                if tag == 'RESULT':
                    current['prefix_pads'] = current['pads']
            elif tag not in ('D', 'MCU_CLOCK', 'DSP_CLOCK'):
                raise ValueError('Unexpected coupled trace label: ' + tag)
    return cases


def rows(case, tag):
    return [fields(row) for row in case['rows'].get(tag, [])]


def single(case, tag):
    values = rows(case, tag)
    if len(values) != 1:
        raise ValueError(tag + ' must occur once per coupled case')
    return values[0]


def validate(cases, blocks, texts, counts):
    if [case['case'] for case in cases] != [dict(order=o, phase=p) for o, p in MATRIX]:
        raise ValueError('CASE coupled order/phase matrix differs')
    words = blocks[4]['words'][0x14478 - 0xd000:]
    chunks = [(word >> shift) & 63 for word in words for shift in (18, 12, 6, 0)]
    for case in cases:
        if [block['index'] for block in case['preloads']] != [4, 5]:
            raise ValueError('PRELOAD main payload and helper must execute on the same owner')
        steps = sum(validate_block(block, blocks[index], LAYOUTS[index], 0xff1000, 5 if index == 4 else 1)
                    for block, index in zip(case['preloads'], (4, 5), strict=True))
        end = case['preload_end']
        if end != dict(pc=0x2e00, epoch=1, steps=steps, status=19, r0=0x2c25, sp=1):
            raise ValueError('PRELOAD_END one native epoch/counter and actual JMP context differ')
        prefix = single(case, 'RESULT')
        if (prefix['bytes'] != 188 or prefix['chunks'] != 256 or prefix['samples'] != 256
                or prefix['source-loads'] != 65 or prefix['irq-entries'] != 1 or prefix['irq-returns'] != 1
                or prefix['mcu-pc'] != 0x1200 or prefix['mcu-d'] != 0xf81b or prefix['mcu-sp'] != 0x21f4
                or prefix['queries'] != case['prefix_pads'] or prefix['board-rejected'] != 1):
            raise ValueError('RESULT original transfer/copy/call prefix differs')
        if rows(case, 'IRQ_PAD') != [dict(known=3, levels=n) for n in (3, 2, 3)]:
            raise ValueError('IRQ_PAD original request edge differs')
        if rows(case, 'IRQ_ENTRY') != [dict(vector=0xfff4, sp=0x21f7)]:
            raise ValueError('IRQ_ENTRY original request vector/frame differs')
        returned = single(case, 'IRQ_RETURN')
        if returned['sp'] != 0x2200 or not 0x1400 <= returned['pc'] < 0x1423:
            raise ValueError('IRQ_RETURN original foreground resume differs')
        if rows(case, 'SOURCE') != [dict(index=n, address=0x14478 + n, value=value)
                                    for n, value in enumerate(words[:66])]:
            raise ValueError('SOURCE first actual padding prefetch differs')
        if rows(case, 'CHUNK') != [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                                   for n, value in enumerate(chunks[:262])]:
            raise ValueError('CHUNK original checksum/padding output differs')
        if rows(case, 'CONSUMER') != [dict(index=n, value=value)
                                      for n, value in enumerate(chunks[:256])]:
            raise ValueError('CONSUMER original native decoder input differs')
        toggles = [(128 if n % 2 == 0 else 0) | chunks[255 + n] for n in range(6)]
        if case['q'] != [0, 0, 0, 128] + [128, 0, 128, 0] * 64 + toggles + [0]:
            raise ValueError('INDICATOR original six toggles and final data-latch clear differ')
        if single(case, 'ENTRY_FLAGS') != dict(flags=0x91):
            raise ValueError('ENTRY_FLAGS original receiver/display activity state differs')
        if single(case, 'CLEAR') != dict(writes=1024, reads=0):
            raise ValueError('CLEAR original blank frame differs')
        observed_text = case['rows'].get('TEXT', [])
        if (len(observed_text) != 2 or
                [fields(row.split(' value=', 1)[0]) for row in observed_text] !=
                [dict(index=n, **metadata) for n, (metadata, _) in enumerate(texts)] or
                [row.split(' value=', 1)[1] for row in observed_text] != [text for _, text in texts]):
            raise ValueError('TEXT original native callback arguments differ')
        if single(case, 'RECTANGLE') != dict(x=0, y=23, width=128, height=18, mode=2):
            raise ValueError('RECTANGLE original XOR band differs')
        if single(case, 'PREPARED') != dict(flags=0xd1):
            raise ValueError('PREPARED native status bit differs')
        if single(case, 'FAULT_STATE') != {'mcu-pc': 0x129c, 'mcu-d': 0xf001, 'mcu-sp': 0x21f2,
                                           'mcu-x': 30000, 'dsp-pc': 0x2e6b, 'cursor': 0x144ba,
                                           'delay': 30000, 'waits': 1}:
            raise ValueError('FAULT_STATE native failed-write registers or complete loop differ')
        faults = case['rows'].get('COUPLED_FAULT', [])
        if (len(faults) != 1 or 'MCU write at 0x003f' not in faults[0] or 'pc=0x129c ccr=0xb8' not in faults[0]
                or 'Unqualified MCU COP system reset (invalid ARMCOP service)' not in faults[0]):
            raise ValueError('FAULT exact native ARMCOP reset request/context differs')
        enabled = single(case, 'COP_ENABLED')
        request = single(case, 'COP_REQUEST')
        if (enabled != dict(control=1, phase=0, oscillator=enabled['oscillator']) or
                request != dict(control=1, phase=request['oscillator'] - enabled['oscillator'],
                                oscillator=request['oscillator'], **{'invalid-service': 1, 'service': 0xf0}) or
                not 0 < request['phase'] < 16384):
            raise ValueError('COP original enable/service sequence and exact elapsed OSC edges differ')
        if single(case, 'QUARANTINE') != {'mcu': 1, 'joint': 1, 'dsp-held': 1, 'dsp-faulted': 0}:
            raise ValueError('QUARANTINE MCU/joint refusal and held independent DSP differ')
        result = single(case, 'COUPLED_RESULT')
        fixed = {key: value for key, value in result.items()
                 if key not in ('queries', 'irq-queries', 'unresolved-events')}
        if (fixed != dict(texts=2, rectangles=1, **counts, delay=30000, waits=1, prepared=0xd1,
                          **{'source-loads': 66, 'chunks': 262, 'samples': 256, 'dsp-pc': 0x2e6b,
                             'cursor': 0x144ba, 'unknown-queries': 0, 'irq-unknown': 0,
                             'storage': 1, 'unknown-tail': 0x144bb, 'helper-returned': 0})
                or result['queries'] != case['coupled_pads'] or result['irq-queries'] <= prefix['irq-queries']
                or result['unresolved-events'] != case['coupled_nets'] or not case['nets'] or case['delay'] != 30000):
            raise ValueError('COUPLED_RESULT raster, complete delay, storage or pad audit differs')
        if single(case, 'RESET_REPLAY') != dict(cut=131, remaining=64, native=1, pins=1, trace=1):
            raise ValueError('RESET_REPLAY native joint pulse continuation differs')
        if single(case, 'RESET_BEGIN') != {'mcu-epoch': 2, 'dsp-epoch': 1}:
            raise ValueError('RESET_BEGIN MCU/DSP epoch continuity differs')
        # Predicted from the fixed original prefix and25-common-tick BRCLR loop,
        # before the first new joint-reset run. Phase18 shifts completion stamps.
        phase = case['case']['phase']
        expected = {0: (27284135, 737), 18: (27284110, 738)}[phase]
        reset = single(case, 'SYSTEM_RESET')
        if (reset != dict(edges=195, vector=0xfffa, target=0x8000, ticks=1024, osc=205, bus=5,
                epoch=2, periods=195, joint=1, storage=1, **{'dsp-held': 0, 'dsp-epoch': 1,
                'dsp-steps': expected[1], 'dsp-before': expected[0], 'dsp-after': 27302560,
                'common-start': 27284148, 'common-end': 27302580}) or
                case['pulse_dsp_steps'] != expected[1]):
            raise ValueError('SYSTEM_RESET pulse/native vector or continuous DSP clock/epoch differs')
        if single(case, 'RESET_BOOT') != dict(start=0x8000, attempt=0x8006, stop=0x8006 if phase == 0 and case['case']['order'] == 1 else 0x8009,
                completed=3, joint=1, perq=255, ppsq=0, refused=1,
                **{'pad-known': 127, 'pad-conflicts': 128, 'dsp-faulted': 1, 'mcu-faulted': 1, 'mcu-ticks': 1516870,
                   'dsp-ticks': 27303660 if phase == 0 and case['case']['order'] == 0 else 27303635}):
            raise ValueError('RESET_BOOT original native pull change or actual shared-net failure differs')
        if case['failed_pads'] != [dict(dsp=1, requested=128, known=0, levels=0, conflicts=128)]:
            raise ValueError('RESET_BOOT exactly one consumed opposed-pull pad must fail')
        faults = case['rows'].get('RESET_BOOT_FAULT', [])
        if (len(faults) != 1 or 'mcu-pc=0x8006' not in faults[0] or '0x002e6b' not in faults[0]
                or 'Unknown DSP GPIO data read at X:0xffffbd mask=0x000080 conflicts=0x000080' not in faults[0]):
            raise ValueError('RESET_BOOT_FAULT real native DSP input/MCU scheduling context differs')
        completion = 27303660 if phase == 0 and case['case']['order'] == 1 else (27303685 if phase == 0 else 27303678)
        if f'attempted-completion={completion}' not in faults[0]:
            raise ValueError('RESET_BOOT_FAULT PwO write-before-final-prefetch/tie boundary differs')
    return dict(cases=4, native_download_stores=121136, checksum_prefetches=17308,
                packet_calls=17316, native_helper_entries=4, native_error_frames=4,
                original_text_callbacks=8, original_rectangles=4, actual_lcd_writes=counts['writes'] * 4,
                actual_lcd_reads=counts['reads'] * 4, native_delay_iterations=120000,
                joint_reset_replays=4, joint_reset_clock_periods=780, native_reset_dsp_steps=2950, actual_consumed_pad_faults=4, cop_reset_requests=4, mcu_reset_pulses=4, native_cop_vector_fetches=4, resident_loader_continuations=4, native_source_reads=264, native_output_chunks=1048, native_input_samples=1024,
                board_rejections=4)


def controls(cases, blocks, texts, counts, frame):
    for tag, key in (('SOURCE', 'value'), ('CHUNK', 'value'), ('RECTANGLE', 'height'),
                     ('PREPARED', 'flags'), ('COP_ENABLED', 'control'), ('COP_REQUEST', 'phase'), ('RESET_REPLAY', 'remaining'), ('SYSTEM_RESET', 'vector'), ('SYSTEM_RESET', 'dsp-steps'), ('RESET_BOOT', 'pad-conflicts'), ('RESET_BOOT', 'stop'), ('QUARANTINE', 'joint'), ('COUPLED_RESULT', 'delay')):
        changed = list(cases)
        changed[0] = copy.deepcopy(cases[0])
        line = changed[0]['rows'][tag][0]
        changed[0]['rows'][tag][0] = re.sub(r'(\b' + key + r'=)(\d+)',
            lambda match: match[1] + str(int(match[2]) + 1), line, count=1)
        try:
            validate(changed, blocks, texts, counts)
        except ValueError:
            continue
        raise ValueError('Coupled oracle accepted fixed ' + tag + ' corruption')
    for kind in ('STORE', 'PRELOAD_END', 'FAULT'):
        changed = list(cases)
        changed[0] = copy.deepcopy(cases[0])
        if kind == 'STORE':
            next(row for tag, row in changed[0]['preloads'][0]['rows'] if tag == 'STORE')['value'] ^= 1
        elif kind == 'PRELOAD_END':
            changed[0]['preload_end']['epoch'] += 1
        else:
            changed[0]['rows']['COUPLED_FAULT'][0] = changed[0]['rows']['COUPLED_FAULT'][0].replace('0x003f', '0x003e')
        try:
            validate(changed, blocks, texts, counts)
        except ValueError:
            continue
        raise ValueError('Coupled oracle accepted fixed ' + kind + ' corruption')
    try:
        check_frame(frame[:-1] + bytes([frame[-1] ^ 1]), frame)
    except ValueError:
        pass
    else:
        raise ValueError('Coupled oracle accepted a flipped native raster pixel')
    return 17


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    if not firmware.is_file() or not assembler.is_file():
        print('SKIP coupled update: local firmware or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    packets = trace.with_suffix('.packets')
    blocks, texts, frame, counts = inputs(firmware, packets)
    templates, provenance = fixture(assembler, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware, packets, templates)}
    native = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(packets.resolve()),
                             str(templates.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=480, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Four native downloads and jointly clocked COP reset continuations verified.'
    if native.returncode or any(line and line != allowed and not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                               for line in output.splitlines()):
        raise ValueError('Native coupled update failed: ' + output)
    cases = collect(trace)
    totals = validate(cases, blocks, texts, counts)
    corruptions = controls(cases, blocks, texts, counts, frame)
    for order, phase in MATRIX:
        base = str(trace) + f'.{order}.{phase}'
        check_frame(Path(base + '.pbm').read_bytes(), frame)
        if not Path(base + '.png').read_bytes().startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('Coupled native screenshot missing')
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Coupled update inputs changed during execution')
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=corruptions,
        frame_sha256=FRAME_SHA256, independent_assembler=provenance,
        damaged_marker_words=1, input_packet_checksum_recomputed=True,
        full_boot=False, complete_update=False, helper_returned=False, watchdog_reset_executed=True, whole_board_reset=False, dsp_reset_executed=False, joint_reset_clocking=True,
        fuzz_execution=False, physical_traffic=False,
        scope='Same clocked DSP executes original main download/padding and helper launch without reset, '
              'then native GPIO/XIRQ/copy/MCU error-helper rendering and30000 delay iterations. '
              'One declared marker word is corrupted in the input packet with recomputed framing checksum. '
              'Own caller/clock/routes/LCD context; original invalid ARMCOP service requests reset, '
              'then joint SYSCLK/native-vector periods advance the same DSP through the MCU reset into the resident loader. '
              'DSP clock/epoch continue. The original loader enables MCU pull-ups before selecting pull-downs; '
              'the native DSP consumes an opposed-pull Q7 net and fails visibly. No voltage winner, upstream dispatch, full boot, live MIDI/audio, '
              'whole update/recovery/power-loss or physical qualification.'), indent=2) + '\n')
    print('Verified4 jointly clocked original error-helper resets and strict shared-pad frontiers.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
