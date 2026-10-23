# SPDX-License-Identifier: GPL-3.0-only
"""Check received original error-helper drawing and a strict missing DSP source word."""

import copy
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields
from original_update_forward import CHUNK_PCS, MATRIX
from inspect_firmware import decode_block, load_input, read_smf, split_blocks
from extract_runtime import hc12_object

ROOT = Path(__file__).resolve().parents[1]
FRAME_SHA256 = '4d5234aad7a550d6b6df1b44ed66944911408307662f6e07609ddfe676cdd60a'
TEXTS = ((0x12a2, 24, '6ae902b451f7bb32dc4727495c34160a234cf93b6a8a99945cd28b6f438d492a'),
         (0x12ae, 32, 'a7634024cfa05268d678177ef13fa232299a6e141757bda5a817eb6dd1d6ba22'))


def reference(firmware):
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    midi, _ = load_input(firmware)
    main = decode_block(split_blocks(read_smf(midi)[0])[4])[1]
    _, offset = hc12_object(main)
    mcu = main[offset:offset + 0x8000]
    source = main[(0x14478 - 0xd000) * 3:]
    helper, end = hc12_object(source)
    if len(helper) != 188 or end != len(source) or len(source) != 195:
        raise ValueError('Changed original diagnostic object extents')
    pixels = [[0] * 128 for _ in range(64)]
    texts, updates = [], 0
    def glyph(character):
        correction = int.from_bytes(mcu[0xea3b - 0x8000 + character:
                                        0xea3b - 0x8000 + character + 1], 'big', signed=True)
        return 0xe81c + 4 * character + correction
    for address, y, expected_hash in TEXTS:
        text = helper[address - 0x1200:].split(b'\0', 1)[0]
        if hashlib.sha256(text).hexdigest() != expected_hash:
            raise ValueError('Changed original diagnostic text identity')
        x, columns = 8, 0
        for character in text:
            begin = glyph(character)
            finish = begin + 2 if character == 32 else glyph(character + 1)
            if not 0 <= finish - begin < 16:
                raise ValueError('Unexpected bounded original glyph width')
            for column, value in enumerate(mcu[begin - 0x8000:finish - 0x8000]):
                for row in range(8):
                    pixels[y + row][x + column] = (value >> (7 - row)) & 1
            columns += finish - begin
            x += finish - begin + 1
        updates += columns + x - 8 - 1  # Actual glyph columns plus mode0 background width.
        texts.append((dict(pointer=address, y=y, x=8, font=1, mode=0), text.decode('ascii')))
    frame = b'P4\n128 64\n' + bytes(sum(pixels[y][x + n] << (7 - n) for n in range(8))
                                      for y in range(64) for x in range(0, 128, 8))
    if hashlib.sha256(frame).hexdigest() != FRAME_SHA256:
        raise ValueError('Independent original-font raster construction changed')
    return source, texts, frame, dict(writes=1024 + updates, reads=2 * updates)


def collect(path):
    cases, current = [], None
    with path.open() as stream:
        for line in stream:
            tag = line.split(' ', 1)[0]
            if tag == 'CASE':
                current = dict(case=fields(line), pads=0, nets=0, q=[], rows={})
                cases.append(current)
            elif current is None:
                raise ValueError('CASE missing before native trace')
            elif tag == 'PAD_QUERY':
                pad = fields(line)
                if (pad['known'] != pad['requested'] or pad['conflicts']
                        or pad['levels'] & ~pad['known']):
                    raise ValueError('PAD_QUERY unresolved native observation')
                current['pads'] += 1
            elif tag == 'NET_UNQUALIFIED':
                current['nets'] += 1
            elif tag == 'M-W':
                found = re.fullmatch(r'M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)\n', line)
                if found:
                    current['q'].append(int(found[1], 16))
            elif tag not in ('M', 'D', 'MCU_CLOCK', 'DSP_CLOCK'):
                if tag not in ('SOURCE', 'CHUNK', 'CONSUMER', 'IRQ_PAD', 'IRQ_ENTRY', 'IRQ_RETURN',
                               'RESULT', 'CLEAR', 'TEXT', 'DIAGNOSTIC_FAULT', 'QUARANTINE',
                               'DIAG_RESULT'):
                    raise ValueError('Unexpected native diagnostic trace label: ' + tag)
                value = line.removesuffix('\n')
                current['rows'].setdefault(tag, []).append(value)
                if tag == 'RESULT':
                    current['prefix_pads'] = current['pads']
    return cases


def rows(case, tag):
    return [fields(line) for line in case['rows'].get(tag, [])]


def single(case, tag):
    values = rows(case, tag)
    if len(values) != 1:
        raise ValueError(tag + ' must occur once')
    return values[0]


def validate(cases, source, texts, counts):
    if [case['case'] for case in cases] != [dict(order=o, phase=p) for o, p in MATRIX]:
        raise ValueError('CASE diagnostic order/phase matrix differs')
    words = [int.from_bytes(source[n:n + 3], 'big') for n in range(0, len(source), 3)]
    chunks = [(word >> shift) & 63 for word in words for shift in (18, 12, 6, 0)]
    for case in cases:
        result = single(case, 'RESULT')
        if (result['bytes'] != 188 or result['chunks'] != result['samples'] or result['chunks'] != 256
                or result['source-loads'] != 65 or result['irq-entries'] != result['irq-returns']
                or result['irq-entries'] != 1 or result['mcu-pc'] != 0x1200
                or result['mcu-d'] != 0xf81b or result['mcu-sp'] != 0x21f4
                or result['queries'] != case['prefix_pads'] or result['board-rejected'] != 1):
            raise ValueError('RESULT original received-helper entry differs')
        if rows(case, 'IRQ_PAD') != [dict(known=3, levels=n) for n in (3, 2, 3)]:
            raise ValueError('IRQ_PAD original request pulse differs')
        if rows(case, 'IRQ_ENTRY') != [dict(vector=0xfff4, sp=0x21f7)]:
            raise ValueError('IRQ_ENTRY original vector/frame differs')
        returned = single(case, 'IRQ_RETURN')
        offsets = (0, 2, 5, 7, 10, 12, 15, 17, 20, 23, 26, 28)
        if returned.get('sp') != 0x2200 or returned.get('pc') not in [0x1400 + n for n in offsets]:
            raise ValueError('IRQ_RETURN must resume a real own-caller instruction boundary')
        if rows(case, 'SOURCE') != [dict(index=n, address=0x14478 + n, value=word)
                                    for n, word in enumerate(words)]:
            raise ValueError('SOURCE diagnostic consumes anything beyond the actual65 words')
        if rows(case, 'CHUNK') != [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                                   for n, value in enumerate(chunks)]:
            raise ValueError('CHUNK actual original checksum fragments differ')
        if rows(case, 'CONSUMER') != [dict(index=n, value=value)
                                      for n, value in enumerate(chunks[:256])]:
            raise ValueError('CONSUMER native receiver input differs')
        indicator = [((128 if n % 2 == 0 else 0) | (chunks[255] if n == 0 else chunks[255 + n]))
                     for n in range(4)]
        if case['q'] != [0, 0, 0, 128] + [128, 0, 128, 0] * 64 + indicator:
            raise ValueError('INDICATOR actual original Q7 toggles/data preservation differ')
        if single(case, 'CLEAR') != dict(writes=1024, reads=0):
            raise ValueError('CLEAR literal zero frame was not established')
        observed_texts = case['rows'].get('TEXT', [])
        expected_texts = [dict(index=n, **metadata) for n, (metadata, _) in enumerate(texts)]
        if (len(observed_texts) != 2 or
                [fields(line.split(' value=', 1)[0]) for line in observed_texts] != expected_texts or
                [line.split(' value=', 1)[1] for line in observed_texts] != [text for _, text in texts]):
            raise ValueError('TEXT actual original root callback arguments differ')
        faults = case['rows'].get('DIAGNOSTIC_FAULT', [])
        if (len(faults) != 1 or 'DSP reference step pc=0x002e5e omr=0x600085' not in faults[0]
                or 'Unknown DSP memory read at Y:0x0144b9' not in faults[0]
                or 'Scheduled peer execution mcu-pc=' not in faults[0]):
            raise ValueError('FAULT missing exact native post-object read/context')
        if single(case, 'QUARANTINE') != dict(mcu=1, dsp=1, unchanged=1):
            raise ValueError('QUARANTINE both clocked owners must refuse further progress')
        end = single(case, 'DIAG_RESULT')
        fixed = {key: value for key, value in end.items()
                 if key not in ('queries', 'irq-queries', 'unresolved-events')}
        expected = dict(texts=2, **counts, chunks=260, samples=256, **{'source-loads': 65,
            'unknown-queries': 0, 'irq-unknown': 0, 'missing': 0x144b9, 'cursor': 0x144ba,
            'prepared': 0, 'nvm-unchanged': 1, 'program-unchanged': 1, 'payload-unchanged': 1,
            'caller-unchanged': 1, 'helper-unchanged': 1, 'ram-tail-unchanged': 1})
        if (fixed != expected or end['queries'] != case['pads']
                or end['irq-queries'] <= result['irq-queries']
                or end['unresolved-events'] != case['nets'] or case['nets'] == 0):
            raise ValueError('DIAG_RESULT original drawing, storage or input audit differs')
    return dict(cases=4, native_error_frames=4, native_text_callbacks=8, literal_clear_cells=4096,
                actual_lcd_writes=counts['writes'] * 4, actual_lcd_reads=counts['reads'] * 4,
                unknown_source_faults=4, quarantined_owners=8, native_source_reads=260,
                native_output_chunks=1040, native_input_samples=1024, board_rejections=4)


def check_frame(observed, expected):
    if observed != expected:
        raise ValueError('FRAME native LCD pixels differ from the independent original-font raster')


def controls(cases, source, texts, counts, frame):
    for tag, key in (('SOURCE', 'value'), ('CHUNK', 'value'), ('CLEAR', 'writes'),
                     ('QUARANTINE', 'unchanged'), ('DIAG_RESULT', 'prepared')):
        changed = copy.deepcopy(cases)
        line = changed[0]['rows'][tag][0]
        replacement, amount = re.subn(r'(\b' + key + r'=)(\d+)',
                                     lambda match: match[1] + str(int(match[2]) + 1), line, count=1)
        if amount != 1:
            raise ValueError('Missing fixed diagnostic oracle corruption')
        changed[0]['rows'][tag][0] = replacement
        try:
            validate(changed, source, texts, counts)
        except ValueError:
            continue
        raise ValueError('Diagnostic oracle accepted fixed ' + tag + ' corruption')
    changed_frame = frame[:-1] + bytes([frame[-1] ^ 1])
    try:
        check_frame(changed_frame, frame)
    except ValueError:
        pass
    else:
        raise ValueError('Raster oracle accepted a flipped native pixel')
    return 6


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original update diagnostic: local firmware absent')
        return 77
    source, texts, frame, counts = reference(firmware)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=240, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    allowed = 'Four original MCU error-helper frames and post-object DSP faults verified.'
    if result.returncode or any(line and line != allowed and
                                not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                                for line in diagnostics.splitlines()):
        raise ValueError('Original update diagnostic failed: ' + diagnostics)
    cases = collect(trace)
    totals = validate(cases, source, texts, counts)
    fixed = controls(cases, source, texts, counts, frame)
    for order, phase in MATRIX:
        base = str(trace) + f'.{order}.{phase}'
        check_frame(Path(base + '.pbm').read_bytes(), frame)
        if not Path(base + '.png').read_bytes().startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('FRAME native saved screenshot is missing')
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original diagnostic input changed')
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, frame_sha256=FRAME_SHA256, fixed_oracle_corruptions=fixed,
        full_boot=False, complete_update=False, helper_returned=False,
        fuzz_execution=False, physical_traffic=False,
        scope='Actual original DSP error delivery and MCU helper drawing after own native '
              'LCD GPIO/display setup. Independent original-font raster and clear/text/stream '
              'effects; strict first missing post-object Y144b9 read quarantines both owners. '
              'Declared caller/RAM/clock/routes and reset LCD model, not physical validation '
              'or complete initialization, helper return, COP restart or whole update/recovery.'), indent=2) + '\n')
    print('Verified4 actual original error frames and4 strict post-object DSP source faults.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
