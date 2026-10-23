# SPDX-License-Identifier: GPL-3.0-only
"""Execute original update validation and check independent checksum/read boundaries."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_runtime_clocks import one, rows

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

BASE, COUNT, MARKER = 0xd000, 0x7473, 0x14477
MASK = 0xffffff
CASES = ('original', 'bad-marker', 'bad-first', 'bad-last', 'changed-prefetch',
         'balanced', 'wrap-zero', 'wrap-nonzero', 'missing-marker', 'missing-first',
         'missing-last', 'missing-prefetch', 'short-circuit')
HOLES = {'missing-marker': MARKER, 'missing-first': BASE,
         'missing-last': BASE + COUNT - 1, 'missing-prefetch': BASE + COUNT,
         'short-circuit': BASE}


def inputs(original, name):
    data = list(original)
    if name in ('bad-marker', 'short-circuit'):
        data[MARKER - BASE] ^= 1
    if name == 'bad-first':
        data[0] ^= 1
    if name == 'bad-last':
        data[COUNT - 1] ^= 1
    if name == 'changed-prefetch':
        data[COUNT] ^= MASK
    if name == 'balanced':
        data[:2] = [(data[0] + 1) % (1 << 24), (data[1] - 1) % (1 << 24)]
    if name in ('wrap-zero', 'wrap-nonzero'):
        data[:COUNT] = [0] * COUNT
        data[0] = MASK
        data[1] = int(name == 'wrap-zero')
    return data


def expected_reads(data, hole):
    addresses = [MARKER]
    if data[MARKER - BASE] == 0x123123:
        addresses.extend(range(BASE, BASE + COUNT + 1))
    result = []
    for address in addresses:
        if address == hole:
            return result, address
        pc = 0x2e09 if address == MARKER else 0x2e10 if address == BASE else 0x2e12
        result.append(dict(address=address, value=data[address - BASE], pc=pc))
    return result, None


def check_case(body, data, name):
    expected, missing = expected_reads(data, HOLES.get(name))
    if rows(body, 'READ') != expected:
        raise ValueError('READ native access/value/order differs: ' + name)
    if one(body, 'GUARDS') != dict(program=400, payload=len(data),
                                    hole=HOLES.get(name, 0), unchanged=1):
        raise ValueError('GUARDS original program/payload changed: ' + name)
    faults = re.findall(r'^FAULT (.+)$', body, re.M)
    if missing is not None:
        pc = 0x2e09 if missing == MARKER else 0x2e10 if missing == BASE else 0x2e12
        if (len(faults) != 1 or f'Unknown DSP memory read at Y:0x{missing:06x}' not in faults[0]
                or f'pc=0x{pc:06x}' not in faults[0] or 'omr=' not in faults[0]):
            raise ValueError('FAULT must retain exact missing address and execution context')
        stop = one(body, 'STOP')
        # The adapter quarantines committed native progress; it does not undo
        # opcode/extension fetch or an AGU postincrement preceding a failed read.
        fetched = 2 if missing == MARKER else 1
        cursor = 0x456 if missing == MARKER else missing + 1
        if (stop['pc'] != pc + fetched or stop['r1'] != cursor or stop['omr'] != 0x600389
                or stop['sp'] != 1 or stop['r7'] != 0x789
                or one(body, 'QUARANTINE') != dict(unchanged=1)):
            raise ValueError('FAULT original caller/register provenance differs')
        if rows(body, 'DECISION') or rows(body, 'END'):
            raise ValueError('FAULT missing data must not produce a validation decision')
        steps = 5 if missing == MARKER else 9 if missing == BASE else missing - BASE + 10
        outcome = 'unknown'
    else:
        if faults or rows(body, 'STOP') or rows(body, 'QUARANTINE'):
            raise ValueError('FAULT unexpected original validation failure: ' + name)
        summed = data[MARKER - BASE] == 0x123123
        checksum = sum(data[:COUNT]) % (1 << 24)
        success = summed and checksum == 0
        decision = one(body, 'DECISION')
        accumulator = checksum << 24 | (0xff << 48 if checksum & 0x800000 else 0)
        if (decision['pc'] != (0x2e19 if success else 0x2e16)
                or decision['r1'] != (BASE + COUNT + 1 if summed else BASE)
                or decision['a'] != (accumulator if summed else 0)):
            raise ValueError('DECISION original marker/checksum branch or cursor differs: ' + name)
        if summed and (decision['x'] & MASK != COUNT or decision['y'] & MASK != data[COUNT]
                       or bool(decision['sr'] & 4) != (checksum == 0)):
            raise ValueError('DECISION sum count, final prefetch or zero flag differs: ' + name)
        end = one(body, 'END')
        if (end['pc'] != (0x2e19 if success else 0x2e4f)
                or end['r1'] != (BASE + COUNT + 1 if success else MARKER + 1)
                or end['a'] != 0 or end['omr'] != 0x600389
                or end['r7'] != 0x789 or end['sp'] != 1):
            raise ValueError('END original accepted/error-helper selection differs: ' + name)
        steps = COUNT + (14 if success else 16) if summed else 10
        outcome = 'accepted' if success else 'rejected'
    if one(body, 'COUNTS') != dict(steps=steps, writes=1):
        raise ValueError('COUNTS original instruction census or priority write differs: ' + name)
    return outcome, len(expected)


def validate(text, original):
    cases = re.findall(r'^CASE name=([^\n]+)\n([\s\S]*?)(?=^CASE |\Z)', text, re.M)
    if [name for name, _ in cases] != list(CASES):
        raise ValueError('CASE original validation matrix missing, duplicated or reordered')
    totals = dict(accepted=0, rejected=0, unknown=0, native_payload_reads=0)
    for name, body in cases:
        outcome, count = check_case(body, inputs(original, name), name)
        totals[outcome] += 1
        totals['native_payload_reads'] += count
    return totals


def negative_controls(text, original):
    for label, field in (('READ', 'value'), ('DECISION', 'r1'), ('END', 'pc'),
                         ('COUNTS', 'steps'), ('GUARDS', 'payload')):
        changed, count = re.subn('^' + label + r' (.*?\b' + field + r'=)(\d+)',
                                 lambda m: label + ' ' + m[1] + str(int(m[2]) + 1),
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing fixed validation oracle control')
        try:
            validate(changed, original)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong fixed validation oracle failure') from error
        else:
            raise ValueError('Validation oracle accepted fixed ' + label + ' corruption')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original update validation: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    midi, _ = load_input(firmware)
    payload = decode_block(split_blocks(read_smf(midi)[0])[4])[1]
    original = [int.from_bytes(payload[i:i + 3], 'big') for i in range(0, len(payload), 3)]
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=110, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr or any(marker in result.stdout for marker in
                                                ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Original update validation failed: ' + result.stdout + result.stderr)
    totals = validate(trace.read_text(), original)
    negative_controls(trace.read_text(), original)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original validation input identity changed during execution')
    report.write_text(json.dumps({'passed': True, 'cases': len(CASES), **totals,
                                  'checksummed_words': COUNT, 'extra_prefetch_words': 1,
                                  'fixed_oracle_corruptions': 5,
                                  'scope': 'Unmodified original main-update validation with an own '
                                  'native caller and declared original, damaged or incomplete RAM '
                                  'payloads. Stops at native success/error-helper selection; no '
                                  'GPIO helper transfer, NAND programming, complete update or boot.',
                                  'fuzz_execution': False, 'physical_traffic': False}, indent=2) + '\n')
    print('Verified 13 original update validation cases and exact operand-read boundaries.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
