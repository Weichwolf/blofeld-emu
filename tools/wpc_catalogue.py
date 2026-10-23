#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Bounded logical catalog/PCM identities from decoded original Spectre WPC pages."""

from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import json
import sys

from inspect_wpc import inspect, require

PAGE_BYTES = 528
PAGE_FRAMES = PAGE_BYTES // 2
INLINE_OFFSET = 78
INLINE_FRAMES = (PAGE_BYTES - INLINE_OFFSET) // 2
MAX_BYTES = 64 * 1024 * 1024
MAX_PROGRAMS = 128


@dataclass(frozen=True)
class Program:
    page: int
    name_sha256: str
    ranges: tuple[tuple[int, int, int], ...]


@dataclass(frozen=True)
class Sample:
    header_page: int
    name_sha256: str
    first_data_page: int
    last_data_page: int
    frames: int
    rate: int
    pitch_word: int
    loop_begin: int | None
    pcm_sha256: str


def word(page, offset):
    return int.from_bytes(page[offset:offset + 3], 'big')


def page_at(data, number):
    require(0 <= number < len(data) // PAGE_BYTES, 'WPC page reference outside supplied file')
    return data[number * PAGE_BYTES:(number + 1) * PAGE_BYTES]


def program_at(data, number):
    page = page_at(data, number)
    name = hashlib.sha256(page[:16]).hexdigest()
    if page[71]:
        return Program(number, name, ((0, 127, number),))
    # The final key has an explicit zero successor; a zero pointer leaves a hole.
    old, begin, ranges = word(page, 72), 0, []
    for key in range(128):
        new = word(page, 75 + 3 * key) if key < 127 else 0
        if new != old:
            if old:
                page_at(data, old)
                ranges.append((begin, key, old))
            begin = key + 1
        old = new
    return Program(number, name, tuple(ranges))


def sample_at(data, number, remaining_bytes):
    page = page_at(data, number)
    first, last = word(page, 24), word(page, 48) & 0x3ffff
    flags = word(page, 33)
    tail = flags & 0x1ff
    require(first <= last and last < len(data) // PAGE_BYTES, 'WPC sample data range invalid')
    require(tail < PAGE_FRAMES, 'WPC sample tail exceeds a PCM page')
    body_frames = (last - first) * PAGE_FRAMES + tail
    frames = INLINE_FRAMES + body_frames
    require(2 * frames <= remaining_bytes, 'WPC aggregate PCM analysis budget exceeded')
    rate = word(page, 27)
    require(rate > 0, 'WPC sample rate is zero')
    loop = None
    if flags & 0x200:
        loop = (word(page, 36) - first) * PAGE_FRAMES + INLINE_FRAMES + (flags >> 15)
        require(0 <= loop < frames, 'WPC sample loop outside logical PCM stream')
    digest = hashlib.sha256(page[INLINE_OFFSET:])
    digest.update(data[first * PAGE_BYTES:first * PAGE_BYTES + 2 * body_frames])
    return Sample(number, hashlib.sha256(page[:16]).hexdigest(), first, last, frames, rate,
                  word(page, 30), loop, digest.hexdigest())


def catalogue(decoded):
    require(isinstance(decoded, bytes), 'WPC parser requires immutable decoded bytes')
    require(decoded and len(decoded) <= MAX_BYTES and len(decoded) % PAGE_BYTES == 0,
            'WPC decoded input must contain bounded complete pages')
    data = memoryview(decoded)
    count = word(data[:PAGE_BYTES], 66)
    require(0 < count <= MAX_PROGRAMS and count <= len(data) // PAGE_BYTES,
            'WPC program count invalid')
    programs = tuple(program_at(data, number) for number in range(count))
    headers = sorted({header for program in programs for _, _, header in program.ranges})
    samples, total = [], 0
    for header in headers:
        sample = sample_at(data, header, MAX_BYTES - total)
        total += 2 * sample.frames
        samples.append(sample)
    return programs, tuple(samples)


def summary(programs, samples):
    identities = dict(programs=[asdict(program) for program in programs],
                      samples=[asdict(sample) for sample in samples])
    encoded = json.dumps(identities, sort_keys=True, separators=(',', ':')).encode('ascii')
    return dict(programs=len(programs), unique_sample_headers=len(samples),
                mappings=sum(len(program.ranges) for program in programs),
                mapped_keys=sum(high - low + 1 for program in programs
                                for low, high, _ in program.ranges),
                pcm_frames=sum(sample.frames for sample in samples),
                looped_samples=sum(sample.loop_begin is not None for sample in samples),
                rates=dict(sorted(Counter(sample.rate for sample in samples).items())),
                catalogue_sha256=hashlib.sha256(encoded).hexdigest())


def main():
    require(len(sys.argv) == 1, 'Usage: wpc_catalogue.py (pinned local references only)')
    decoded, reference = inspect()
    programs, samples = catalogue(decoded)
    result = summary(programs, samples)
    result['decoded_sha256'] = reference['decoded_sha256']
    result['qualification'] = ('Logical original Spectre WPC catalog and BE16 PCM identities. '
                               'No physical NAND/ECC map, original client execution, installation '
                               'or firmware playback equivalence. Names/PCM remain local.')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, StopIteration) as error:
        sys.exit(str(error))
