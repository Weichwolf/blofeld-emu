# SPDX-License-Identifier: GPL-3.0-only
"""Qualify original first Keyboard sender, received display helper and resident return."""

import copy
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from keyboard_update_reference import LAYOUTS, verified_blocks
from original_dsp_download import fixture
from original_dsp_voice import ROOT, original_records
from original_gpio_pair import fields
from original_update_transport import packet_references, validate_block
from inspect_firmware import decode_block, load_input, read_smf, split_blocks
from extract_runtime import hc12_object

MATRIX = ((0, 0), (0, 18), (1, 0), (1, 18))
CHUNK_PCS = (0x2e26, 0x2e2b, 0x2e30, 0x2e35)
FRAME_SHA256 = '224df22472ad9063584b927cc9c880f731d97c732feec3cce1a69704547a5d86'


def drawing(firmware, blocks):
    main = decode_block(split_blocks(read_smf(load_input(firmware)[0])[0])[4])[1]
    _, offset = hc12_object(main)
    mcu = main[offset:offset + 0x8000]
    helper, _ = hc12_object(decode_block(blocks[0])[1])
    pixels = [[0] * 128 for _ in range(64)]
    expected, updates = [], 0
    for pointer, font, x, y in ((0x125e, 2, 8, 8), (0x1271, 1, 0, 0)):
        text = helper[pointer - 0x1200:].split(b'\0', 1)[0]
        if not 0 < len(text) <= 32:
            raise ValueError('Changed bounded original initial-helper text')
        base, table, stride, height = ((0xe5c4, 0xe79b, 3, 6) if font == 2
                                       else (0xe81c, 0xea3b, 4, 8))
        def glyph(character):
            correction = int.from_bytes(mcu[table - 0x8000 + character:
                                            table - 0x8000 + character + 1], 'big', signed=True)
            return base + stride * character + correction
        cursor, columns = x, 0
        for character in text:
            begin = glyph(character)
            end = begin + 2 if character == 32 else glyph(character + 1)
            if not 0 <= end - begin < 16:
                raise ValueError('Changed original bounded font-column extent')
            for column, value in enumerate(mcu[begin - 0x8000:end - 0x8000]):
                for row in range(height):
                    pixels[y + row][cursor + column] ^= (value >> (7 - row)) & 1
            columns += end - begin
            cursor += end - begin + 1
        updates += columns + cursor - x - 1
        expected.append(dict(index=len(expected), pointer=pointer, y=y, x=x, font=font, mode=0))
    # Original mode1 clears the separate y20..37 band over three controller pages.
    for row in range(20, 38):
        for column in range(128):
            pixels[row][column] = 0
    updates += 3 * 128
    frame = b'P4\n128 64\n' + bytes(sum(pixels[y][x + bit] << (7 - bit) for bit in range(8))
                                      for y in range(64) for x in range(0, 128, 8))
    if hashlib.sha256(frame).hexdigest() != FRAME_SHA256:
        raise ValueError('Independent original Font6/Font8 raster identity changed')
    return expected, frame, dict(writes=1024 + updates, reads=2 * updates)


def collect(path):
    cases, current, replay, preload = [], None, None, None
    allowed = {'SOURCE', 'CHUNK', 'CONSUMER', 'PASSIVE_SAMPLE', 'TRAILER_BYTE', 'ENTRY',
               'ACK', 'SENDER_RETURN', 'HELPER_RETURN', 'INDICATOR', 'INDICATOR_READ',
               'TEXT', 'RECTANGLE', 'CLEAR_ENTRY', 'RETURN_BOUNDARY', 'INITIAL_END', 'RETAINED', 'PAD_FAILURE'}
    ignored = {'M', 'D', 'M-W', 'MCU_CLOCK', 'DSP_CLOCK'}
    with path.open('rb') as stream:
        for raw in stream:
            line = raw.decode('ascii').rstrip('\n')
            tag = line.split(' ', 1)[0]
            if tag == 'CASE':
                current = dict(case=fields(line), rows={}, pads=0, nets=0, failed_pads=[],
                               preload={}, replays=[])
                cases.append(current)
                replay, preload = None, None
                continue
            if current is None:
                raise ValueError('Initial Keyboard trace requires CASE')
            if tag == 'PRELOAD':
                preload = fields(line)['block']
                current['preload'][preload] = []
                continue
            if tag == 'PRELOAD_END':
                current['preload_end'] = fields(line)
                preload = None
                continue
            if preload is not None:
                current['preload'][preload].append((tag, fields(line)))
                continue
            if tag == 'REPLAY':
                if replay is not None:
                    replay['sha256'] = digest.hexdigest()
                replay = dict(index=fields(line)['index'], rows={}, pads=0, nets=0, bytes=0)
                current['replays'].append(replay)
                digest = hashlib.sha256()
                continue
            if tag == 'REPLAY_END':
                if replay is None:
                    raise ValueError('Missing actual initial Keyboard suffix replay')
                replay['sha256'] = digest.hexdigest()
                replay = None
                current['replay_end'] = fields(line)
                continue
            target = replay if replay is not None else current
            if replay is not None:
                digest.update(raw)
                replay['bytes'] += len(raw)
            if tag == 'PAD_QUERY':
                pad = fields(line)
                if pad['known'] != pad['requested'] or pad['conflicts'] or pad['levels'] & ~pad['known']:
                    if replay is not None or 'prefix_pads' not in current:
                        raise ValueError('Unexpected unknown pad in original initial Keyboard path')
                    current['failed_pads'].append(pad)
                target['pads'] += 1
            elif tag == 'NET_UNQUALIFIED':
                target['nets'] += 1
            elif tag in allowed:
                target['rows'].setdefault(tag, []).append(fields(line))
                if tag == 'RETURN_BOUNDARY':
                    current['prefix_pads'], current['prefix_nets'] = current['pads'], current['nets']
            elif tag not in ignored:
                raise ValueError('Unexpected initial Keyboard trace tag: ' + tag)
    return cases


def require_rows(scope, tag, expected):
    if scope['rows'].get(tag, []) != expected:
        raise ValueError('Changed native initial Keyboard ' + tag)


def validate(cases, expected, texts, counts):
    if [case['case'] for case in cases] != [dict(order=o, phase=p) for o, p in MATRIX]:
        raise ValueError('Incomplete initial Keyboard order/phase matrix')
    words = expected[0]['words'][:41]
    chunks = [(word >> shift) & 63 for word in words for shift in (18, 12, 6, 0)]
    for case in cases:
        steps = sum(validate_block(dict(rows=case['preload'][index]), expected[index], LAYOUTS[index],
                                   0xff1000, 5 if index == 0 else 1) for index in (0, 1))
        if case['preload_end'] != dict(pc=0x2e00, epoch=1, steps=steps):
            raise ValueError('Original initial downloads changed epoch/launch continuity')
        require_rows(case, 'ENTRY', [dict(pc=0x1200, d=0xf81b, sp=0x21f4)])
        require_rows(case, 'SOURCE', [dict(index=n, address=0xef00 + n, value=word)
                                      for n, word in enumerate(words)])
        require_rows(case, 'CHUNK', [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                                     for n, value in enumerate(chunks)])
        require_rows(case, 'CONSUMER', [dict(index=n, value=value) for n, value in enumerate(chunks)])
        require_rows(case, 'ACK', [dict(pc=0x2e38, next=0x2e3a)])
        require_rows(case, 'SENDER_RETURN', [dict(pc=0xff1036, sp=0)])
        require_rows(case, 'HELPER_RETURN', [dict(pc=0xbb56, sp=0x21f6, flags=0xd1)])
        require_rows(case, 'RETURN_BOUNDARY', [dict(pc=0xbb56, **{'dsp-pc': 0xff1036})])
        require_rows(case, 'TRAILER_BYTE', [dict(index=n, value=0, **{'return': 0xbb6c}) for n in range(2)])
        require_rows(case, 'PAD_FAILURE', [dict(trailer=2, samples=164, unknown=1)])
        # DDRQ=80 leaves all seven low pins as inputs; the later AND3f does not remove Q6's read.
        if case['failed_pads'] != [dict(dsp=0, requested=127, known=0, levels=0, conflicts=0)]:
            raise ValueError('Missing actual withdrawn post-cache pad failure')
        require_rows(case, 'INDICATOR', [dict(index=n) for n in range(1, 6)])
        require_rows(case, 'INDICATOR_READ', [dict(index=n + 1, value=value)
                                              for n, value in enumerate((chunks[-1], 128, 0, 128, 0))])
        require_rows(case, 'TEXT', texts)
        require_rows(case, 'RECTANGLE', [dict(index=0, x=0, y=20, width=128, height=18, mode=1)])
        require_rows(case, 'CLEAR_ENTRY', [dict(pc=0x8d70)])
        if [x['index'] for x in case['replays']] != [0, 1] or case['replay_end'] != dict(restores=2, identical=1):
            raise ValueError('Missing two actual trusted initial Keyboard continuations')
        a, b = case['replays']
        if not a['bytes'] or (a['bytes'], a['sha256']) != (b['bytes'], b['sha256']):
            raise ValueError('Complete original joint suffix traces differ')
        for replay in case['replays']:
            require_rows(replay, 'PASSIVE_SAMPLE', [dict(index=n, value=0) for n in range(12)])
            require_rows(replay, 'CONSUMER', [dict(index=164 + n, value=0) for n in range(12)])
            # The return address follows the original near JSR in the trailer loop.
            trailers = replay['rows'].get('TRAILER_BYTE', [])
            if len(trailers) != 9 or any(row != dict(index=n, value=0, **{'return': 0xbb6c})
                                         for n, row in enumerate(trailers)):
                raise ValueError('Original nine decoded trailer bytes differ')
            end = copy.deepcopy(replay['rows'].get('INITIAL_END', []))
            if (len(end) != 1 or end[0].pop('queries', -1) != case['prefix_pads'] + replay['pads']
                    or end[0].pop('unresolved-events', -1) != case['prefix_nets'] + replay['nets']
                    or end[0].pop('irq-queries', 0) <= 0):
                raise ValueError('Initial Keyboard complete pad-query trace accounting differs')
            if end != [dict(**{'mcu-pc': 0x2021, 'dsp-pc': 0xff1036},
                samples=176, chunks=164, loaded=41, **{'helper-returned': 1, 'sender-returned': 1},
                ack=1, indicators=5, texts=2, rectangles=1, trailer=9, **counts, flags=0xc0,
                unknown=0, **{'irq-unknown': 0})]:
                raise ValueError('Original complete initial Keyboard return state differs')
        require_rows(case, 'RETAINED', [dict(helper=1, source=1, program=1, **{'main-loop': 1},
                                             nvm=1, scratch=1, callers=1, **{'board-rejected': 1})])
    return dict(cases=4, original_helper_returns=4, original_sender_returns=4, copied_helper_bytes=476,
                original_source_words=164, original_output_chunks=656, original_copy_samples=656,
                trusted_suffix_replays=8, withdrawn_pad_failures=4, passive_sample_reads=96,
                decoded_trailer_bytes=72,
                native_text_calls=8, native_lcd_writes=counts['writes'] * 4,
                native_lcd_reads=counts['reads'] * 4)


def controls(cases, expected, texts, counts):
    changes = []
    for tag, key in (('SOURCE', 'value'), ('CHUNK', 'value'), ('ENTRY', 'sp'), ('TEXT', 'font')):
        changed = copy.deepcopy(cases)
        changed[0]['rows'][tag][0][key] += 1
        changes.append(changed)
    changed = copy.deepcopy(cases)
    changed[0]['replays'][0]['rows']['PASSIVE_SAMPLE'][0]['value'] = 1
    changes.append(changed)
    changed = copy.deepcopy(cases)
    changed[0]['replays'][1]['sha256'] = 'changed'
    changes.append(changed)
    changed = copy.deepcopy(cases)
    changed[0]['prefix_pads'] -= 1
    changes.append(changed)
    changed = copy.deepcopy(cases)
    changed[0]['rows']['TRAILER_BYTE'][0]['value'] = 1
    changes.append(changed)
    for changed in changes:
        try:
            validate(changed, expected, texts, counts)
        except ValueError:
            continue
        raise ValueError('Fixed initial Keyboard oracle corruption accepted')
    return len(changes)


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    if not all(path.is_file() for path in (firmware, keyboard, assembler)):
        print('SKIP original initial Keyboard path: local archives or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    blocks, _, _ = verified_blocks(keyboard)
    packets = trace.with_suffix('.packets')
    expected = packet_references(blocks, LAYOUTS, packets)
    templates, provenance = fixture(assembler, trace)
    branch = trace.with_suffix('.branch.bin')
    source = ROOT / 'test/keyboard_initial_branch.asm'
    subprocess.run([assembler, source, '-f', 'u24be', '-o', branch], check=True, timeout=10)
    if branch.stat().st_size != 3:
        raise ValueError('Independent loop branch must be one word')
    words = trace.with_suffix('.initial-templates.txt')
    words.write_text(templates.read_text() + f"{int.from_bytes(branch.read_bytes(), 'big'):06x}\n")
    texts, frame, counts = drawing(firmware, blocks)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, packets, words, source)}
    run = subprocess.run([str(path.resolve()) for path in (binary, firmware, packets, words, trace)],
                         capture_output=True, text=True, check=False, timeout=270)
    output = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Original initial Keyboard sender and display-helper observations completed.'
    if run.returncode or any(row and row != allowed and
                            not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                            for row in output.splitlines()):
        raise ValueError('Original initial Keyboard execution failed: ' + output)
    cases = collect(trace)
    totals = validate(cases, expected, texts, counts)
    corruptions = controls(cases, expected, texts, counts)
    for order, phase in MATRIX:
        base = str(trace) + f'.{order}.{phase}'
        if Path(base + '.pbm').read_bytes() != frame:
            raise ValueError('Actual original LCD frame differs from independent glyph/rectangle pixels')
        if not Path(base + '.png').read_bytes().startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('Missing saved original initial Keyboard screenshot')
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != value for path, value in frozen.items()):
        raise ValueError('Original initial Keyboard input changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **totals,
        fixed_oracle_corruptions=corruptions, frame_sha256=FRAME_SHA256,
        independent_assembler=provenance, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, controller_executed=False, fuzz_execution=False,
        physical_traffic=False, scope='Original first Keyboard packet bodies, sender, XIRQ/resident '
        'copy/display-helper/return and two restored resident trailer suffixes. Explicit caller, '
        'reset LCD, logical Port-C bias, clocks/routes/pads; no full SMF interleaving, later '
        'controller programming/recovery, physical wiring/timing, normal boot/MIDI/audio or '
        'whole-instrument acceptance.'), indent=2) + '\n')
    print('Verified four original initial Keyboard sender/display-helper/return paths and eight suffix replays.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
