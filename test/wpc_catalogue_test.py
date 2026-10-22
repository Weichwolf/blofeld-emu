# SPDX-License-Identifier: GPL-3.0-only
"""Independent synthetic WPC spans and pinned factory catalogue identities."""

import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import wpc_catalogue as wpc
from inspect_wpc import inspect, require


def put(data, page, offset, value):
    start = page * 528 + offset
    data[start:start + 3] = value.to_bytes(3, 'big')


def own_input():
    data = bytearray(7 * 528)
    put(data, 0, 66, 2)
    for key in range(128):
        put(data, 0, 72 + key * 3, 2 if key < 60 else 0 if key < 80 else 3)
    data[528 + 71] = 1
    for header, first, last, tail, rate in ((1, 6, 6, 0, 22050),
                                            (2, 4, 5, 2, 44100),
                                            (3, 5, 5, 1, 48000)):
        put(data, header, 24, first)
        put(data, header, 27, rate)
        put(data, header, 30, 0x123456)
        put(data, header, 33, tail)
        put(data, header, 48, last)
    put(data, 2, 33, (3 << 15) | 0x200 | 2)
    put(data, 2, 36, 4)
    data[528 + 78:2 * 528] = b'\x00\x07' * 225
    data[2 * 528 + 78:3 * 528] = bytes.fromhex('80007fff0000') * 75
    data[3 * 528 + 78:4 * 528] = bytes.fromhex('00010002fffe') * 75
    data[4 * 528:5 * 528] = bytes([0x12]) * 528
    data[5 * 528:6 * 528] = bytes([0x34]) * 528
    data[6 * 528:7 * 528] = bytes([0x56]) * 528
    return bytes(data)


def rejects(data, label):
    try:
        wpc.catalogue(data)
    except ValueError as error:
        require(label in str(error), 'Wrong catalogue rejection: ' + str(error))
        return
    raise ValueError('Invalid catalogue accepted: ' + label)


def synthetic():
    data = own_input()
    programs, samples = wpc.catalogue(data)
    require([program.ranges for program in programs] ==
            [((0, 59, 2), (80, 127, 3)), ((0, 127, 1),)],
            'Own key transitions retain holes, final key 127 and inline single-program samples')
    expected = [b'\x00\x07' * 225,
                bytes.fromhex('80007fff0000') * 75 + bytes([0x12]) * 528 + bytes([0x34]) * 4,
                bytes.fromhex('00010002fffe') * 75 + bytes([0x34]) * 2]
    require([(sample.frames, sample.rate, sample.pitch_word, sample.loop_begin)
             for sample in samples] == [(225, 22050, 0x123456, None),
                                        (491, 44100, 0x123456, 228),
                                        (226, 48000, 0x123456, None)],
            'Independent literal frame/rate/raw-pitch/loop expectations differ')
    require([sample.pcm_sha256 for sample in samples] ==
            [hashlib.sha256(pcm).hexdigest() for pcm in expected],
            'PCM identities must use all 225 inline frames and exact additional BE16 frames')
    summary = wpc.summary(programs, samples)
    require((summary['mapped_keys'], summary['pcm_frames'], summary['looped_samples']) ==
            (236, 942, 1), 'Own catalogue coverage census differs')
    controls = ((0, 66, 0, 'program count'), (0, 66, 129, 'program count'),
                (0, 66, 8, 'program count'), (0, 72, 7, 'page reference'),
                (0, 72, 0xffffff, 'page reference'),
                (2, 24, 6, 'data range'), (2, 48, 7, 'data range'),
                (2, 33, 264, 'tail'), (2, 27, 0, 'rate'),
                (2, 36, 3, 'loop'), (2, 36, 6, 'loop'))
    for page, offset, value, label in controls:
        changed = bytearray(data)
        put(changed, page, offset, value)
        rejects(bytes(changed), label)
    for length in (0, 1, 527, 528, 529, 1056, 2640, len(data) - 1):
        rejects(data[:length], 'WPC')
    rejects(bytearray(data), 'immutable')
    changed = bytearray(data)
    put(changed, 2, 48, 0xfc0005)
    require(wpc.catalogue(bytes(changed))[1] == samples,
            'The original 18-bit end-page selection does not consume upper flags')
    changed = bytearray(data)
    for key in range(80, 128):
        put(changed, 0, 72 + key * 3, 2)
    require(len(wpc.catalogue(bytes(changed))[1]) == 2,
            'Repeated header references are analyzed once without merging key-range holes')
    old_limit = wpc.MAX_BYTES
    try:
        wpc.MAX_BYTES = len(data) - 1
        rejects(data, 'bounded complete pages')
        wpc.MAX_BYTES = len(data)
        changed = bytearray(data)
        for header in (1, 2, 3):
            put(changed, header, 24, 4)
            put(changed, header, 48, 6)
            put(changed, header, 33, 263)
        rejects(bytes(changed), 'aggregate PCM')
    finally:
        wpc.MAX_BYTES = old_limit
    require(wpc.catalogue(data) == (programs, samples), 'Checks must not mutate decoded input')
    print('Verified own WPC key/PCM/loop spans and 22 directed rejection controls; no fuzzing.')


def reference(output):
    manifest = json.loads((ROOT / 'doc/wpc-reference.json').read_text())
    needed = [ROOT / 'doc' / row['archive'] for row in manifest['spectre']]
    needed.append(ROOT / 'doc/library' / manifest['wpc_file'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    if not all(path.is_file() for path in needed):
        print('SKIP WPC catalogue reference: original local assets absent')
        return 77
    decoded, _ = inspect()
    programs, samples = wpc.catalogue(decoded)
    observed = wpc.summary(programs, samples)
    require(json.loads(json.dumps(observed)) == manifest['catalogue'],
            'Changed pinned original WPC catalogue/PCM identity')
    output.write_text(json.dumps(dict(passed=True, **observed, fuzz_execution=False,
                                     physical_traffic=False,
                                     qualification='Logical original Spectre WPC catalogue/PCM '
                                     'identities only; no native client execution, physical NAND '
                                     'map, installation or original firmware playback.'), indent=2) + '\n')
    print('Verified 41 programs/136 sample streams; physical mapping and playback remain open.')
    return 0


if __name__ == '__main__':
    try:
        if sys.argv[1:] == ['--synthetic']:
            synthetic()
        else:
            require(len(sys.argv) == 2, 'Expected --synthetic or private reference report')
            sys.exit(reference(Path(sys.argv[1])))
    except (OSError, ValueError, KeyError, StopIteration) as error:
        sys.exit(str(error))
