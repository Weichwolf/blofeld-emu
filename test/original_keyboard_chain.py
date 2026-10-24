# SPDX-License-Identifier: GPL-3.0-only
"""Original stage2 downloads/reset/LCD and a strict, explicitly conditional BKGD refusal."""

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from keyboard_chain_trace import MATRIX, controls, require, validate
from keyboard_update_reference import LAYOUTS, verified_blocks
from lcd_export import png
from original_dsp_download import fixture
from original_dsp_voice import ROOT, original_records
from extract_runtime import hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks
from original_update_transport import packet_references

FRAME_SHA256 = '240fdd792792ab032f36cf679574e0404eef73e3dfe699e0d46d7ee0b1a91d7f'
SUCCESS = 'Original Keyboard reset/LCD/BDC chain retains its strict reference wire frontier.'


def drawing(firmware, blocks):
    main = decode_block(split_blocks(read_smf(load_input(firmware)[0])[0])[4])[1]
    _, offset = hc12_object(main)
    mcu = main[offset:offset+0x8000]
    helper, _ = hc12_object(decode_block(blocks[2])[1])
    text = helper[0x15d0-0x1200:].split(b'\0', 1)[0]
    require(0 < len(text) <= 32, 'Changed bounded original saving text')
    pixels = [[0]*128 for _ in range(64)]
    # Original entry SP+2:text word,+4:y,+5:x,+6:font. The helper pushes
    # font2, x8, y16 and the text word, then performs the actual JSR.
    def glyph(character):
        correction = int.from_bytes(mcu[0xe79b-0x8000+character:0xe79b-0x8000+character+1],
                                    'big', signed=True)
        return 0xe5c4+3*character+correction
    cursor, columns = 8, 0
    for character in text:
        begin = glyph(character)
        end = begin+2 if character == 32 else glyph(character+1)
        require(0 <= end-begin < 16 and cursor+end-begin <= 128, 'Changed font column geometry')
        for column, value in enumerate(mcu[begin-0x8000:end-0x8000]):
            for row in range(6):
                pixels[16+row][cursor+column] ^= (value >> (7-row)) & 1
        columns += end-begin
        cursor += end-begin+1
    packed = bytes(sum(pixels[y][x+bit] << (7-bit) for bit in range(8))
                   for y in range(64) for x in range(0, 128, 8))
    frame = b'P4\n128 64\n'+packed
    require(hashlib.sha256(frame).hexdigest() == FRAME_SHA256,
            'Independent original saving raster identity changed')
    require(columns == 65 and cursor-8-1 == 83, 'Independent glyph/background traffic changed')
    png_rows = b''.join(b'\0'+bytes(value ^ 255 for value in packed[y*16:(y+1)*16])
                        for y in range(64))
    return frame, png_rows


def verify_artifacts(trace, expected, frame, png_rows):
    text = trace.read_text()
    totals = validate(text, expected)
    totals['collector_negative_controls'] = controls(text, expected)
    for order, phase, _ in MATRIX:
        base = str(trace)+f'.{order}.{phase}'
        require(Path(base+'.pbm').read_bytes() == frame,
                'Native original LCD frame differs from independent font/background pixels')
        png(Path(base+'.png'), 128, png_rows)
    return totals


def main():
    binary, firmware, keyboard, hcs08, hc12, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT/'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    require(all(path.is_file() for path in (binary, hcs08, hc12)), 'Missing native chain build/languages')
    if not all(path.is_file() for path in (firmware, keyboard, assembler)):
        print('SKIP original Keyboard chain: local original archives or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    for order, phase, _ in MATRIX:
        for suffix in ('.pbm', '.png'):
            Path(str(trace)+f'.{order}.{phase}'+suffix).unlink(missing_ok=True)
    original_records(firmware, json.loads((ROOT/'doc/dsp-voice-reference.json').read_text()))
    blocks, _, _ = verified_blocks(keyboard)
    packets = trace.with_suffix('.packets')
    expected = packet_references(blocks, LAYOUTS, packets)
    templates, provenance = fixture(assembler, trace)
    frame, png_rows = drawing(firmware, blocks)
    frozen = {path:hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, hcs08, hc12, assembler, packets, templates)}
    native = subprocess.run([str(path.resolve()) for path in (binary, firmware, packets, templates, trace)],
                            capture_output=True, text=True, timeout=500, check=False)
    output = native.stdout+native.stderr
    trace.with_suffix('.log').write_text(output)
    require(native.returncode == 0 and native.stdout.splitlines().count(SUCCESS) == 1
            and all(line == SUCCESS or re.fullmatch(r'Jit@\d+: No profiler detected', line)
                    for line in output.splitlines()), 'Original native chain failed: '+output)
    totals = verify_artifacts(trace, expected, frame, png_rows)
    require(all(hashlib.sha256(path.read_bytes()).hexdigest() == value for path, value in frozen.items()),
            'Original chain inputs changed during execution')
    receipt.write_text(json.dumps(dict(passed=True, **totals, independent_assembler=provenance,
        frame_sha256=FRAME_SHA256, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(path):value for path, value in frozen.items()},
        full_boot=False, complete_update=False, target_application_executed=False,
        physical_wiring_qualified=False, physical_timing_qualified=False,
        exact_silicon_SYNC_detection_qualified=False, hardware_equivalence=False,
        fuzz_execution=False, physical_traffic=False,
        scope='Original stage2 packet downloads, DSP sender, native MCU XIRQ/copy/helper/checksum, '
              'reset/SYNC, pending BACKGROUND and actual recursive saving-text LCD path. Two '
              'declared order/phase contexts and four fresh joint native suffix restorations. '
              'Own callers, warm target preimage,4MHz target/30MHz MCU, direct GPIO/panel inputs '
              'and named wire reference contract; its128-period recognition and collision refusal '
              'are not silicon detector/recovery specifications. Stops before trim/upload/GO/FSTAT; '
              'no full stage1/SMF/startup, target NVM programming, physical recovery or boot/audio '
              'equivalence.'), indent=2)+'\n')
    print('Verified original stage2/reset/LCD chain and conditional reference wire refusals.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
