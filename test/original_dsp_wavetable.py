# SPDX-License-Identifier: GPL-3.0-only
"""Pin original providers before checking the isolated native wavetable FIR proof."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from extract_runtime import dsp_records, hc12_object
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def verify_inputs(root, firmware, reference):
    downloads = json.loads((root / 'doc/downloads.json').read_text())['files']
    pinned = next(row for row in downloads if row['file'] == firmware.name)
    if hashlib.sha256(firmware.read_bytes()).hexdigest() != pinned['sha256']:
        raise ValueError('Changed original wavetable firmware archive')
    midi, _ = load_input(firmware)
    if hashlib.sha256(midi).hexdigest() != reference['midi_sha256']:
        raise ValueError('Changed original wavetable MIDI')
    messages, _ = read_smf(midi)
    payload = decode_block(split_blocks(messages)[4])[1]
    _, offset = hc12_object(payload)
    records, _, _, _ = dsp_records(payload[offset + ((0x8000 + 2) // 3) * 3 + 3:])
    for expected in reference['blocks']:
        actual = records[expected['record']]
        observed = (actual['space'], actual['address'], actual['words'],
                    hashlib.sha256(actual['data']).hexdigest())
        wanted = tuple(expected[key] for key in ('space', 'address', 'words', 'sha256'))
        if observed != wanted:
            raise ValueError('Changed original wavetable program/coefficient provider')


def validate_report(text, reference):
    lines = text.splitlines()
    if len(lines) != 1:
        raise ValueError('Original wavetable result requires one complete summary')
    values = {}
    for token in lines[0].split():
        key, value = token.split('=')
        if key in values:
            raise ValueError('Duplicate original wavetable result field')
        values[key] = int(value)
    if values.pop('instructions', 0) <= 0 or values != reference['counts']:
        raise ValueError('Incomplete original wavetable execution')


def main():
    if len(sys.argv) != 4:
        raise ValueError('Usage: original_dsp_wavetable.py binary firmware.zip private.trace')
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original wavetable FIR: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-wavetable-reference.json').read_text())
    verify_inputs(ROOT, firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    with trace.open('w') as output:
        result = subprocess.run([str(binary.resolve()), str(firmware.resolve())],
                                stdout=output, check=False, timeout=110)
    if result.returncode:
        return result.returncode
    validate_report(trace.read_text(), reference)
    print('Original wavetable FIR:2448 declared native calls and42 instruction starts.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
