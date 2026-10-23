# SPDX-License-Identifier: GPL-3.0-only
"""Check original secondary checksum instructions after declared NAND identification."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_runtime_clocks import one, rows
from original_update_validation import ROOT, MASK
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

BASE, FIRST, COUNT = 0x10000, 0x10001, 0xb01
CASES = ('original', 'bad-first', 'bad-last', 'changed-page', 'changed-prefetch', 'balanced',
         'wrap-zero', 'wrap-nonzero', 'missing-first', 'missing-last', 'missing-prefetch',
         'missing-page')
HOLES = {'missing-first': FIRST, 'missing-last': FIRST + COUNT - 1,
         'missing-prefetch': FIRST + COUNT, 'missing-page': BASE}


def inputs(original, name):
    data = list(original)
    if name == 'bad-first':
        data[1] ^= 1
    if name == 'bad-last':
        data[COUNT] ^= 1
    if name == 'changed-page':
        data[0] ^= 1
    if name == 'changed-prefetch':
        data[COUNT + 1] ^= MASK
    if name == 'balanced':
        data[1:3] = [(data[1] + 1) % (1 << 24), (data[2] - 1) % (1 << 24)]
    if name in ('wrap-zero', 'wrap-nonzero'):
        data[1:COUNT + 1] = [0] * COUNT
        data[1:3] = [MASK, int(name == 'wrap-zero')]
    return data


def check_case(body, data, name):
    hole = HOLES.get(name)
    addresses = list(range(FIRST, FIRST + COUNT + 1))
    missing = hole if hole in addresses else None
    if missing is not None:
        addresses = addresses[:addresses.index(missing)]
    expected = [dict(address=address, value=data[address - BASE],
                     pc=0x2e13 if address == FIRST else 0x2e15) for address in addresses]
    if rows(body, 'READ') != expected:
        raise ValueError('READ original secondary operand/value/order differs: ' + name)
    if one(body, 'GUARDS') != dict(program=388, payload=len(data), hole=hole or 0, unchanged=1):
        raise ValueError('GUARDS original secondary helper/payload changed: ' + name)
    faults = re.findall(r'^FAULT (.+)$', body, re.M)
    if missing is not None:
        pc = 0x2e13 if missing == FIRST else 0x2e15
        if (len(faults) != 1 or f'Unknown DSP memory read at Y:0x{missing:06x}' not in faults[0]
                or f'pc=0x{pc:06x}' not in faults[0] or 'omr=' not in faults[0]):
            raise ValueError('FAULT secondary missing word/context differs')
        stop = one(body, 'STOP')
        if (stop['pc'] != pc + 1 or stop['r1'] != missing + 1
                or stop['omr'] != 0x600389 or stop['sp'] != 1 or stop['r7'] != 0x789):
            raise ValueError('FAULT secondary committed fetch/cursor/caller differs')
        if one(body, 'QUARANTINE') != dict(unchanged=1):
            raise ValueError('QUARANTINE secondary fault must prevent further native progress')
        if rows(body, 'DECISION') or rows(body, 'END'):
            raise ValueError('FAULT incomplete secondary checksum cannot select a result')
        steps = 2 if missing == FIRST else missing - FIRST + 3
        outcome = 'unknown'
    else:
        if faults or rows(body, 'STOP') or rows(body, 'QUARANTINE'):
            raise ValueError('FAULT unexpected secondary failure')
        checksum = sum(data[1:COUNT + 1]) % (1 << 24)
        accepted = checksum == 0
        decision, end = one(body, 'DECISION'), one(body, 'END')
        accumulator = checksum << 24 | (0xff << 48 if checksum & 0x800000 else 0)
        if (decision != end or end['pc'] != (0x2e1a if accepted else 0x2e47)
                or end['a'] != accumulator or end['r1'] != FIRST + COUNT + 1
                or end['x'] & MASK != COUNT or end['y'] & MASK != data[COUNT + 1]
                or bool(end['sr'] & 4) != accepted or end['omr'] != 0x600389
                or end['sp'] != 1 or end['r7'] != 0x789):
            raise ValueError('DECISION END secondary branch/accumulator/cursor/state differs')
        steps, outcome = COUNT + 7, 'accepted' if accepted else 'rejected'
    if one(body, 'COUNTS') != dict(steps=steps, writes=0):
        raise ValueError('COUNTS secondary native census or unexpected write differs')
    return outcome, len(expected)


def validate(text, original):
    cases = re.findall(r'^CASE name=([^\n]+)\n([\s\S]*?)(?=^CASE |\Z)', text, re.M)
    if [name for name, _ in cases] != list(CASES):
        raise ValueError('CASE secondary matrix missing, repeated or reordered')
    totals = dict(accepted=0, rejected=0, unknown=0, native_payload_reads=0)
    for name, body in cases:
        outcome, count = check_case(body, inputs(original, name), name)
        totals[outcome] += 1
        totals['native_payload_reads'] += count
    return totals


def negative_controls(text, original):
    for label, field in (('READ', 'value'), ('DECISION', 'a'), ('END', 'pc'),
                         ('COUNTS', 'steps'), ('GUARDS', 'payload'), ('QUARANTINE', 'unchanged')):
        changed, count = re.subn('^' + label + r' (.*?\b' + field + r'=)(\d+)',
                                 lambda match: label + ' ' + match[1] + str(int(match[2]) + 1),
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing secondary fixed oracle control')
        try:
            validate(changed, original)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong secondary fixed oracle failure') from error
        else:
            raise ValueError('Secondary oracle accepted fixed ' + label + ' corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original secondary validation: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    midi, _ = load_input(firmware)
    payload = decode_block(split_blocks(read_smf(midi)[0])[2])[1]
    original = [int.from_bytes(payload[i:i + 3], 'big') for i in range(0, len(payload), 3)]
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve()),
                             'secondary'], capture_output=True, text=True, timeout=110, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr or any(marker in result.stdout for marker in
                                                ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Original secondary validation failed: ' + result.stdout + result.stderr)
    text = trace.read_text()
    totals = validate(text, original)
    negative_controls(text, original)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Secondary validation input identity changed')
    report.write_text(json.dumps(dict(
        passed=True, cases=len(CASES), **totals, checksummed_words=COUNT, extra_prefetch_words=1,
        fixed_oracle_corruptions=6, fuzz_execution=False, physical_traffic=False,
        scope='Unchanged original secondary checksum instructions after declared '
              'post-identification entry. Own native caller/register/RAM context; no preceding '
              'GPIO/NAND identification, delay, programming, helper transfer or complete update '
              'is claimed. The unchecked page selector remains outside this checksum.'), indent=2) + '\n')
    print('Verified12 original secondary checksum cases and exact native read boundaries.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
