# SPDX-License-Identifier: GPL-3.0-only
"""Independent integer oracle for original sample interpolation and local WPC PCM."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from inspect_wpc import inspect
from wpc_catalogue import catalogue, page_at, INLINE_OFFSET, PAGE_BYTES

W = 1 << 24
H = W // 2
FRACTIONS = (0, 1, 127, 8191, 16383, 16384, 16385, 32767)
BANKS = ((128, 0xde96), (64, 0xe9ee), (32, 0xebf6), (16, 0xecfe))
TOTAL = 4 * 32768 + 136 * 4 * 3 * len(FRACTIONS) + 4 * len(FRACTIONS) * 8


def signed(value):
    return value if value < H else value - W


def quantize(value):
    return min(max(value // W, -H), H - 1)


def predict(bank, phase, rate, depth, fm, samples, coefficients):
    half_phases, base = BANKS[bank]
    fraction = phase & 32767
    if fraction < 16384:
        row, stride = half_phases * fraction // 16384, 1
        pointer = base + 8 * row
    else:
        row, stride = (half_phases * (32768 - fraction) + 16383) // 16384, -1
        pointer = base + 8 * row + 7
    addresses = tuple(pointer + stride * tap for tap in range(8))
    assert all(base <= address < base + 8 * (half_phases + 1) for address in addresses)
    total = 2 * sum(signed(coefficients[address]) * signed(value)
                    for address, value in zip(addresses, samples))
    modulated = quantize(2 * signed(fm) * signed(rate))
    next_phase = (((signed(phase) + signed(rate)) * W +
                   2 * signed(depth) * modulated) // W) % W
    return next_phase, quantize(total) % W, pointer, stride



def controls():
    base = BANKS[0][1]
    coefficients = {base + index: 0x100000 for index in range(1032)}
    window = (256,) * 8 + (H,)
    for fraction, pointer, stride in ((0, base, 1), (16383, base + 1016, 1),
                                       (16384, base + 1031, -1), (32767, base + 15, -1)):
        assert predict(0, fraction, 0, 0, 0, window, coefficients) == (
            fraction, 256, pointer, stride)
    assert predict(0, W - 1, 1, 0, 0, window, coefficients)[0] == 0
    assert predict(0, 0, H, H, H, window, coefficients)[0] == 1
    assert quantize(-1) == -1 and quantize(H * W) == H - 1

def vectors(decoded, samples):
    # All fractional phases use a fixed asymmetric signed16 window at ring wrap.
    own = tuple((value * 256) % W for value in (-32768, -12345, -1, 0, 1, 1234, 32767, 37, -8192))
    for bank in range(4):
        for fraction in range(32768):
            yield bank, (511 << 15) | fraction, 0x8000, 0, 0, own
    for sample in samples:
        page = page_at(decoded, sample.header_page)
        begin = sample.first_data_page * PAGE_BYTES
        body = decoded[begin:begin + 2 * (sample.frames - 225)]
        pcm = page[INLINE_OFFSET:] + body[:2 * (sample.frames - 225)]
        assert len(pcm) == 2 * sample.frames and hashlib.sha256(pcm).hexdigest() == sample.pcm_sha256
        for start in (0, sample.loop_begin if sample.loop_begin is not None else sample.frames // 2,
                      sample.frames - 9):
            start = min(start, sample.frames - 9)
            window = tuple((int.from_bytes(pcm[2 * (start + i):2 * (start + i + 1)],
                                           'big', signed=True) * 256) % W for i in range(9))
            for bank in range(4):
                for fraction in FRACTIONS:
                    yield bank, ((start % 512) << 15) | fraction, 0x8000, 0x200000, 0x135700, window
    corners = ((0, 0, 0), (0x8000, 0, 0), (0x7fffff, 0x7fffff, 0x7fffff),
               (0x800000, 0x7fffff, 0x800000), (0xffffff, 1, 0xffffff),
               (0x10000, 0x800000, 0x7fffff), (0x7fffff, 0x800000, 0x800000),
               (0x8000, 0x400000, 0x800000))
    for bank in range(4):
        for fraction in FRACTIONS:
            for index, (rate, depth, fm) in enumerate(corners):
                values = tuple(((H - 1 if (index + i) % 2 else -H) if index > 1 else
                                (H - 1 if index else -H)) % W for i in range(9))
                yield bank, (511 << 15) | fraction, rate, depth, fm, values


def write_vectors(path, decoded, samples, coefficients, limit=None):
    count = 0
    clipped = 0
    limited_fm = 0
    with path.open('w') as output:
        output.write('blofeld-sampler-v1 ' + '0' * 12 + '\n')
        for bank, phase, rate, depth, fm, window in vectors(decoded, samples):
            result = predict(bank, phase, rate, depth, fm, window, coefficients)
            clipped += result[1] in (H, H - 1)
            product = 2 * signed(fm) * signed(rate)
            limited_fm += not -H <= product // W < H
            row = (count, bank, phase, rate, depth, fm, *result, *window)
            output.write(' '.join(map(str, row)) + '\n')
            count += 1
            if limit is not None and count == limit:
                break
        output.seek(len('blofeld-sampler-v1 '))
        output.write(f'{count:012d}')
    return count, clipped, limited_fm


def validate(trace, vectors_path, expected):
    rows = trace.read_text().splitlines()
    if len(rows) != expected + 1:
        raise ValueError('Incomplete native sampler trace')
    with vectors_path.open() as source:
        next(source)
        for index, line in enumerate(source):
            values = tuple(map(int, line.split()))
            if rows[index] != f'CASE {index} {values[6]} {values[7]}':
                raise ValueError('Native sampler evidence differs from independent complete cases')
    if rows[-1] != f'RESULT {expected} {expected * 36} 36':
        raise ValueError('Missing original sampler instruction/return coverage')


def inputs(firmware):
    shared = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    reference = json.loads((ROOT / 'doc/dsp-sample-reference.json').read_text())
    coefficients, _ = original_records(firmware, shared)
    if (reference['records'] != [shared['records'][index] for index in (39, 40, 114)] or
            reference['midi_sha256'] != shared['midi_sha256']):
        raise ValueError('Changed original sampler program/coefficient record selection')
    decoded, _ = inspect()
    _, samples = catalogue(decoded)
    assert len(samples) == 136
    for bank, (phases, base) in enumerate(BANKS):
        descriptor = tuple(coefficients[0xed86 + bank * 4 + i] for i in range(4))
        assert descriptor == (phases, 8, base, 0)
    bounds = tuple(max(sum(abs(signed(coefficients[base + 8 * row + tap]))
                           for tap in range(8)) for row in range(phases + 1))
                   for phases, base in BANKS)
    assert bounds == tuple(reference['coefficient_l1_bounds']) and max(bounds) < H
    return reference, coefficients, decoded, samples


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    controls()
    trace.parent.mkdir(parents=True, exist_ok=True)
    try:
        reference, coefficients, decoded, samples = inputs(firmware)
    except FileNotFoundError:
        print('SKIP original sampler: pinned private firmware/library/Spectre references absent')
        return 77
    vectors_path = trace.with_suffix('.inputs')
    count, clipped, limited_fm = write_vectors(vectors_path, decoded, samples, coefficients)
    assert count == TOTAL and clipped == 0 and limited_fm > 0
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(vectors_path.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=220, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            'AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Native sample interpolation failed: ' + diagnostics)
    validate(trace, vectors_path, count)
    trace.with_suffix('.json').write_text(json.dumps({
        'passed': True, 'calls': count, 'fraction_cases': 4 * 32768,
        'official_pcm_cases': 136 * 4 * 3 * len(FRACTIONS), 'sample_headers': 136,
        'corner_cases': 4 * len(FRACTIONS) * 8, 'limited_outputs': clipped,
        'limited_fm_transfers': limited_fm, 'coefficient_l1_bounds': reference['coefficient_l1_bounds'],
        'native_instructions': count * 36, 'coefficient_reads': count * 8,
        'sample_reads': count * 8, 'input_sha256': hashlib.sha256(vectors_path.read_bytes()).hexdigest(),
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'scope': reference['qualification'], 'fuzz_execution': False, 'physical_audio': False,
    }, indent=2) + '\n')
    print(f'Verified {count} unchanged native sampler calls, all fractional phases/four banks, '
          '136 official logical PCM sources and independent phase/output/access expectations.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
