#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Bounded declared-page view of original sample-preparation frames, without hole filling."""

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
from types import MappingProxyType

from wpc_catalogue import Program, word

PAGE_BYTES = 528
PAGES_PER_GROUP = 32
DATA_BYTES = PAGE_BYTES * PAGES_PER_GROUP
GROUP_BYTES = DATA_BYTES + 24
FRAME_BYTES = GROUP_BYTES + 4
STAGING = (0x10000, 0x11610, 0x12c20, 0x14230)
MAX_FRAME_BYTES = 32 * 1024 * 1024
MAX_PCM_BYTES = 64 * 1024 * 1024
ROW_LIMIT = 1 << 24  # Native helper argument width; not an established NAND geometry.


def require(condition, message):
    if not condition:
        raise ValueError(message)


@dataclass(frozen=True)
class PreparedPages:
    pages: Mapping[int, bytes]
    group_rows: tuple[int, ...]

    def page(self, row):
        require(type(row) is int and 0 <= row < ROW_LIMIT, 'Declared sample row outside24-bit domain')
        if row not in self.pages:
            raise ValueError(f'Unknown declared sample page at row0x{row:06x}')
        return self.pages[row]

    def spans(self):
        result = []
        for row in sorted(self.pages):
            if result and row == result[-1][1]:
                result[-1] = (result[-1][0], row + 1)
            else:
                result.append((row, row + 1))
        return tuple(result)


def parse_frames(frames):
    require(type(frames) is bytes and 0 < len(frames) <= MAX_FRAME_BYTES and
            len(frames) % FRAME_BYTES == 0, 'Sample frames require bounded complete groups')
    pages, rows = {}, []
    for group, begin in enumerate(range(0, len(frames), FRAME_BYTES)):
        staging = int.from_bytes(frames[begin:begin + 4], 'big')
        require(staging == STAGING[group % 4], 'Unexpected sample staging sequence')
        data = begin + 4
        row = int.from_bytes(frames[data + DATA_BYTES:data + DATA_BYTES + 3], 'big')
        require(frames[data + DATA_BYTES + 3:data + DATA_BYTES + 6] == b'\xdd\xef\x0e',
                'Missing original sample group marker')
        require(row % PAGES_PER_GROUP == 0 and row + PAGES_PER_GROUP <= ROW_LIMIT,
                'Unaligned or out-of-domain declared sample row')
        rows.append(row)
        for offset in range(PAGES_PER_GROUP):
            require(row + offset not in pages, 'Overlapping declared sample groups')
            start = data + offset * PAGE_BYTES
            pages[row + offset] = frames[start:start + PAGE_BYTES]
    return PreparedPages(MappingProxyType(pages), tuple(rows))



@dataclass(frozen=True)
class SampleHeader:
    header_page: int
    name_sha256: str
    first_data_page: int
    native_end_page: int
    client_end_page: int
    client_frames: int
    rate: int
    pitch_word: int
    client_loop_begin: int | None
    client_pcm_sha256: str

def sample_at(pages, row, budget):
    page = pages.page(row)
    require(page[71] == 1, 'Referenced page is not an observed sample header')
    first, last = word(page, 24), word(page, 48) & 0x3ffff
    native_end = word(page, 39)
    flags = word(page, 33)
    tail = flags & 0x1ff
    require(first <= native_end <= last and tail < 264,
            'Unqualified native/client sample extent or tail')
    frames = 225 + (last - first) * 264 + tail
    require(2 * frames <= budget, 'Declared PCM aggregate budget exceeded')
    rate = word(page, 27)
    require(rate > 0, 'Declared sample rate is zero')
    loop = None
    if flags & 0x200:
        loop = 225 + (word(page, 36) - first) * 264 + (flags >> 15)
        require(0 <= loop < frames, 'Declared sample loop outside logical PCM')
    digest = hashlib.sha256(page[78:])
    for number in range(first, last + 1):
        count = tail if number == last else 264
        digest.update(pages.page(number)[:count * 2])
    return SampleHeader(row, hashlib.sha256(page[:16]).hexdigest(), first, native_end, last, frames,
                        rate, word(page, 30), loop, digest.hexdigest())


@dataclass(frozen=True)
class Unresolved:
    program: int
    low: int
    high: int
    target: int


def catalogue(pages, first_program, program_count):
    # The caller selects a fixed reference extent; no new sentinel/count protocol is inferred.
    require(type(program_count) is int and 0 < program_count <= 128 and
            type(first_program) is int and 0 <= first_program < ROW_LIMIT and
            first_program + program_count <= ROW_LIMIT, 'Invalid program extent')
    programs, references, unresolved = [], set(), []
    for row in range(first_program, first_program + program_count):
        page = pages.page(row)
        require(page[71] in (0, 1), 'Unqualified declared program page type')
        pointers = (row,) * 128 if page[71] else tuple(word(page, 72 + 3 * key) for key in range(128))
        ranges, low = [], 0
        for end in range(1, 129):
            if end < 128 and pointers[end] == pointers[low]:
                continue
            target = pointers[low]
            if target not in pages.pages:
                unresolved.append(Unresolved(row, low, end - 1, target))
            else:
                require(pages.page(target)[71] == 1, 'Program reference is not a sample header')
                ranges.append((low, end - 1, target))
                references.add(target)
            low = end
        programs.append(Program(row, hashlib.sha256(page[:16]).hexdigest(), tuple(ranges)))
    samples, used = [], 0
    for row in sorted(references):
        sample = sample_at(pages, row, MAX_PCM_BYTES - used)
        used += 2 * sample.client_frames
        samples.append(sample)
    return tuple(programs), tuple(samples), tuple(unresolved)
