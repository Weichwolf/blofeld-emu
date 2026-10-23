# SPDX-License-Identifier: GPL-3.0-only
"""Run the original MCU receiver with explicit GPIO sources and unknown target pins."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from keyboard_update_reference import verified_blocks
from original_dsp_voice import ROOT, original_records
from extract_runtime import hc12_object
from inspect_firmware import decode_block


def reference_files(keyboard, trace):
    blocks, reference, _ = verified_blocks(keyboard)
    data = decode_block(blocks[2])[1]
    helper, end = hc12_object(data)
    consumed = 2 + len(helper)
    initial = data[consumed:end]
    layout = reference['receiver_entry']
    if (len(helper), consumed, end, len(initial), hashlib.sha256(initial).hexdigest()) != (
            1018, layout['source_byte_offset'], layout['container_end'],
            layout['bytes'], layout['sha256']):
        raise ValueError('Changed programming-helper entry-word alignment')
    if consumed % 3 or len(initial) != 3:
        raise ValueError('MCU length-prefixed helper must leave exactly its checksum word')
    code = trace.with_suffix('.helper.bin')
    code.write_bytes(helper)
    word = trace.with_suffix('.entry-word.bin')
    word.write_bytes(initial)
    return code, word, layout


def validate(trace):
    lines = trace.read_text().splitlines()
    words = [row for row in lines if row.startswith('WORD ')]
    missing = [row for row in lines if row.startswith('MISSING ')]
    replay = [row for row in lines if row.startswith('REPLAY ')]
    controller = [row for row in lines if row.startswith('CONTROLLER ')]
    if (len(words), len(missing), len(replay), len(controller)) != (777, 4, 2, 3):
        raise ValueError('Incomplete directed receiver/unknown-input/restore/controller matrix')
    if lines[-1] != 'TOTAL words=777 bytes=2331 missing=4 replays=2 prefixes=3':
        raise ValueError('Changed original receiver totals')
    if any('steps=97 bytes=3 requests=4 reads=4' not in row for row in words):
        raise ValueError('Original word/cached-byte call contract changed')
    known = [row for row in controller if row.startswith('CONTROLLER known=1 ')]
    if len(known) != 2 or any('dbne=130560 ' not in row or 'mask=0x80' not in row for row in known):
        raise ValueError('Original delay loops or unknown released PE7 changed')
    if 'known=0 ' not in controller[-1] or 'dbne=0 ' not in controller[-1] or 'mask=0x6c' not in controller[-1]:
        raise ValueError('Unsupplied initial PE inputs must fail before controller delay')
    return dict(word_cases=777, returned_bytes=2331, missing_chunk_cases=4,
                exact_native_suffix_restores=2, controller_prefixes=3,
                original_dbne_steps=2 * 2 * 255 * 256)


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    if not binary.is_file():
        raise ValueError('Original Keyboard receiver executable missing')
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original Keyboard receiver: local original archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, layout = reference_files(keyboard, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, helper, word)}
    native = subprocess.run([str(path.resolve()) for path in (binary, firmware, helper, word, trace)],
                            capture_output=True, text=True, timeout=150, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    if native.returncode or output != 'Original Keyboard receiver and controller-input frontiers verified.\n':
        raise ValueError('Original Keyboard receiver execution failed: ' + output)
    counts = validate(trace)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != value for path, value in frozen.items()):
        raise ValueError('Original Keyboard receiver input changed during execution')
    receipt.write_text(json.dumps(dict(
        passed=True, **counts, reference_alignment=layout,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, keyboard_controller_executed=False,
        fuzz_execution=False, physical_traffic=False,
        scope='Unchanged1018-byte MCU helper under an own native caller and declared GPIO/OSC/TIM '
              'conditions. Four actual six-bit pin reads and three original byte returns; '
              'independent byte/cache/frame expectations and native delay-instruction checks. '
              'Target PE7 remains unknown after release. No original DSP sender/copy handoff, '
              'physical timing, controller image/ISA, programming/recovery or normal Keyboard '
              'boot/MIDI/audio qualification.'), indent=2) + '\n')
    print('Verified777 original receiver words, four missing chunks and three strict pin frontiers.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
