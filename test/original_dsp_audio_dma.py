# SPDX-License-Identifier: GPL-3.0-only
"""Qualify the original dual-DMA control fragment under explicit reference inputs."""

import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
CORNERS = (0x800001, 0xffffff, 0, 0x7fffff, 0x800000, 1, 0x654321, 0xabcdef)


def value(channel, ordinal, changed):
    index = ordinal % 64
    if changed and index == 0 and (channel == 3 or ordinal != 0):
        return 0x5abcde
    return CORNERS[index % 8] ^ ((index // 8) * 0x10203)


def bmc_word(bits, expected, preamble):
    if len(bits) != 64 or bits[:8] != preamble:
        raise ValueError('Changed literal DAX subframe preamble')
    payload = 0
    previous = bits[7]
    for bit in range(28):
        first, second = bits[8 + 2 * bit:10 + 2 * bit]
        if first == previous:
            raise ValueError('Missing independent BMC data-cell boundary')
        payload |= int(first != second) << bit
        previous = second
    if payload.bit_count() % 2 or payload & 0x7ffffff != expected:
        raise ValueError('DAX audio, metadata or even parity differs from the ring oracle')


def validate(lines):
    cases = []
    for line in lines:
        words = line.split()
        if not words:
            raise ValueError('Empty dual-DMA trace record')
        if words[0] == 'CASE':
            if len(words) != 2 or cases and cases[-1]['result'] is None:
                raise ValueError('Incomplete dual-DMA case boundary')
            cases.append(dict(name=words[1], words=[], pcm=[], bmc=None, result=None))
        elif words[0] in ('WORD', 'PCM'):
            if (not cases or cases[-1]['bmc'] is not None
                    or len(words) != (5 if words[0] == 'WORD' else 4)):
                raise ValueError('Unexpected native event after stream finalization')
            key = 'words' if words[0] == 'WORD' else 'pcm'
            cases[-1][key].append(tuple(map(int, words[1:])))
        elif words[0] == 'BMC':
            if not cases or len(words) != 2 or cases[-1]['bmc'] is not None:
                raise ValueError('Duplicate or malformed BMC stream')
            cases[-1]['bmc'] = words[1]
        elif words[0] == 'RESULT':
            if (not cases or len(words) != 6 or cases[-1]['result'] is not None
                    or cases[-1]['bmc'] is None):
                raise ValueError('Duplicate or premature dual-DMA result')
            cases[-1]['result'] = tuple(map(int, words[1:]))
        else:
            raise ValueError('Unexpected dual-DMA trace record')
    if [case['name'] for case in cases] != ['baseline', 'changed0', 'changed1']:
        raise ValueError('Incomplete dual-DMA baseline/held-word replay matrix')
    for case in cases:
        changed = case['name'] != 'baseline'
        expected = [(7 + ordinal * 256 + offset, channel, ordinal,
                     value(channel, ordinal, changed))
                    for ordinal in range(132) for channel, offset in ((0, 1), (3, 3))]
        if case['words'] != expected or case['result'] != (132, 132, 66, 8449, 33795):
            raise ValueError('Original ring/counter/source-capture/reference-phase scope differs')
        pcm = []
        for frame in range(66):
            left = value(0, 2 * frame - 1, changed) if frame else 0
            right = value(0, 2 * frame, changed)
            pcm.append((frame, left if left < 0x800000 else left - 0x1000000,
                        right if right < 0x800000 else right - 0x1000000))
        if case['pcm'] != pcm or len(case['bmc']) != 8449 or set(case['bmc']) != {'0', '1'}:
            raise ValueError('Actual PCM/BMC stream is incomplete or differs from the literal oracle')
        for frame in range(66):
            left = value(3, 2 * frame - 2, changed) if frame else 0
            right = value(3, 2 * frame - 1, changed) if frame else 0
            start = frame * 128
            bmc_word(case['bmc'][start:start + 64], left,
                     '00010111' if frame == 0 else '00011101')
            bmc_word(case['bmc'][start + 64:start + 128], right, '00011011')
        if case['bmc'][-1] != '0':
            raise ValueError('The retained partial next-frame preamble unit differs')
    if any(cases[1][key] != cases[2][key] for key in ('words', 'pcm', 'bmc', 'result')):
        raise ValueError('Complete held-word suffix replay differs')
    return cases


def controls(lines):
    for tag, column in (('WORD', 1), ('WORD', 2), ('WORD', 4), ('PCM', 2), ('RESULT', 2)):
        changed = copy.copy(lines)
        index = next(i for i, row in enumerate(changed) if row.startswith(tag + ' '))
        fields = changed[index].split()
        fields[column] = str(int(fields[column]) + 1)
        changed[index] = ' '.join(fields)
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Fixed dual-DMA collector corruption was accepted')
    changed = copy.copy(lines)
    index = next(i for i, row in enumerate(changed) if row.startswith('BMC '))
    bits = list(changed[index][4:])
    bits[8] = '1' if bits[8] == '0' else '0'
    changed[index] = 'BMC ' + ''.join(bits)
    try:
        validate(changed)
    except ValueError:
        pass
    else:
        raise ValueError('Fixed BMC boundary corruption was accepted')
    for tag in ('BMC ', 'RESULT '):
        changed = copy.copy(lines)
        index = next(i for i, row in enumerate(changed) if row.startswith(tag))
        changed.insert(index, changed[index])
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Fixed duplicate ' + tag + ' record was accepted')
    return 8


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not binary.is_file():
        raise ValueError('Dual-DMA native executable missing')
    if not firmware.is_file():
        print('SKIP original dual DMA: private firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    trace.with_suffix('.json').unlink(missing_ok=True)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    run = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                         capture_output=True, text=True, check=False, timeout=100)
    output = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Original dual repeating DMA controls, real ESAI/DAX streams and held-word replay verified.\n'
    if run.returncode or output != allowed:
        raise ValueError('Original dual-DMA reference execution failed: ' + output)
    lines = trace.read_text().splitlines()
    validate(lines)
    mutations = controls(lines)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != sha for path, sha in frozen.items()):
        raise ValueError('A dual-DMA native input changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(
        passed=True, cases=3, words_per_channel_per_case=132, ring_words=64,
        completed_ring_wraps_per_channel_per_case=2, pcm_frames_per_case=66,
        dax_frames_per_case=66, retained_dax_partial_units=1, held_source_replays=2,
        fixed_collector_corruptions=mutations,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, original_complete_initialization=False, physical_traffic=False,
        fuzz_execution=False, automatic_core_dma_timing=False,
        scope='Two unchanged original DMA-enable instructions, own native configuration caller, '
              'declared source ring and coherent HCKT/ACI reference inputs. CPU remains paused; '
              'no original reserved access is accepted, no normal boot/live audio or hardware '
              'clock/board equivalence is established.'), indent=2) + '\n')
    print('Verified original dual-DMA ring, both pin streams and held-word suffixes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
