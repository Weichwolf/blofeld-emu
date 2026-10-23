# SPDX-License-Identifier: GPL-3.0-only
"""Directed declared-page controls and pinned original sample-MIDI address evidence."""

from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from sample_midi_pages import catalogue, parse_frames, sample_at, FRAME_BYTES, PAGE_BYTES, STAGING
from original_sample_preparation import load_reference, sample_blocks, extract_inputs


def rejects(operation, message):
    try:
        operation()
    except ValueError as error:
        assert message in str(error), str(error)
        return
    raise ValueError('Directed invalid declared-page input reported success')


def controls():
    def field(page, offset, value):
        page[offset:offset + 3] = value.to_bytes(3, 'big')
    program = bytearray(PAGE_BYTES)
    program[:3] = b'Own'
    for key in range(128):
        field(program, 72 + 3 * key, 0x3400 if key < 127 else 0xffffff)
    header = bytearray(PAGE_BYTES)
    header[71] = 1
    for offset, value in ((24, 0x3401), (27, 44100), (33, 3), (39, 0x3401), (48, 0x3401)):
        field(header, offset, value)
    inline = b''.join(struct.pack('>h', number - 112) for number in range(225))
    header[78:] = inline
    body = struct.pack('>3h', -32768, 0, 32767) + bytes(PAGE_BYTES - 6)
    def group(index, row, pages):
        payload = b''.join(pages) + bytes((32 - len(pages)) * PAGE_BYTES)
        return STAGING[index].to_bytes(4, 'big') + payload + row.to_bytes(3, 'big') + b'\xdd\xef\x0e' + bytes(18)
    first = group(0, 0x3000, [program])
    second = group(1, 0x3400, [header, body])
    frames = first + second
    assert len(frames) == 2 * FRAME_BYTES
    pages = parse_frames(frames)
    assert pages.spans() == ((0x3000, 0x3020), (0x3400, 0x3420))
    programs, samples, unknown = catalogue(pages, 0x3000, 1)
    assert programs[0].ranges == ((0, 126, 0x3400),)
    assert len(samples) == 1 and samples[0].client_frames == 228 and samples[0].rate == 44100
    assert samples[0].client_pcm_sha256 == hashlib.sha256(inline + body[:6]).hexdigest()
    assert [asdict(x) for x in unknown] == [dict(program=0x3000, low=127, high=127, target=0xffffff)]
    extended = bytearray(header); field(extended, 48, 0x3402)
    separate = parse_frames(first + group(1, 0x3400, [extended, body]))
    _, distinct, _ = catalogue(separate, 0x3000, 1)
    assert distinct[0].native_end_page == 0x3401 and distinct[0].client_end_page == 0x3402
    assert distinct[0].client_frames == 492
    rejects(lambda: sample_at(pages, 0x3400, 455), 'aggregate budget')
    rejects(lambda: parse_frames(bytearray(frames)), 'bounded complete groups')
    rejects(lambda: pages.page(0x3020), 'Unknown declared sample page at row0x003020')
    rejects(lambda: pages.page(-1), '24-bit domain')
    rejects(lambda: parse_frames(b''), 'bounded complete groups')
    rejects(lambda: parse_frames(frames[:-1]), 'bounded complete groups')
    changed = bytearray(frames); changed[4 + 32 * PAGE_BYTES + 3] ^= 1
    rejects(lambda: parse_frames(bytes(changed)), 'group marker')
    changed = bytearray(frames); changed[FRAME_BYTES:FRAME_BYTES + 4] = STAGING[0].to_bytes(4, 'big')
    rejects(lambda: parse_frames(bytes(changed)), 'staging sequence')
    rejects(lambda: parse_frames(first + group(1, 0x3000, [header])), 'Overlapping')
    rejects(lambda: parse_frames(first + group(1, 0x3401, [header])), 'Unaligned')
    rejects(lambda: catalogue(pages, 0x3000, 129), 'program extent')
    changed_header = bytearray(header); field(changed_header, 24, 0x3420); field(changed_header, 39, 0x3420); field(changed_header, 48, 0x3420)
    missing = parse_frames(first + group(1, 0x3400, [changed_header, body]))
    rejects(lambda: catalogue(missing, 0x3000, 1), 'Unknown declared sample page at row0x003420')
    changed_header = bytearray(header); field(changed_header, 33, 264)
    bad_tail = parse_frames(first + group(1, 0x3400, [changed_header, body]))
    rejects(lambda: catalogue(bad_tail, 0x3000, 1), 'extent or tail')
    changed_header = bytearray(header); field(changed_header, 33, 0x200); field(changed_header, 36, 0x3500)
    bad_loop = parse_frames(first + group(1, 0x3400, [changed_header, body]))
    rejects(lambda: catalogue(bad_loop, 0x3000, 1), 'loop outside')
    print('Directed sample-row, unknown-page, reference, PCM and bounded rejection controls pass')


def reference_inputs():
    reference = json.loads((ROOT / 'doc/sample-midi-reference.json').read_text())
    preparation = json.loads((ROOT / 'doc/sample-preparation-reference.json').read_text())
    assert reference['prepared_frames'] == preparation['frames']
    archive = ROOT / 'doc/library' / reference['archive']
    messages, observed = load_reference(archive)
    _, frames = extract_inputs(sample_blocks(messages), preparation, observed)
    pages = parse_frames(bytes(frames))
    return reference, pages, catalogue(pages, reference['first_program'], reference['program_count'])


def reference_check(report_path):
    try:
        reference, pages, (programs, samples, unknown) = reference_inputs()
    except FileNotFoundError:
        print('SKIP original sample MIDI pages: pinned local archive absent')
        return 77
    identities = dict(programs=[asdict(x) for x in programs], samples=[asdict(x) for x in samples],
                      unresolved=[asdict(x) for x in unknown])
    summary = dict(groups=len(pages.group_rows), declared_pages=len(pages.pages), spans=pages.spans(),
                   programs=len(programs), samples=len(samples), client_view_pcm_frames=sum(x.client_frames for x in samples),
                   distinct_native_client_ends=sum(x.native_end_page != x.client_end_page for x in samples),
                   rates=dict(sorted(Counter(x.rate for x in samples).items())),
                   unresolved_ranges=len(unknown), unresolved_keys=sum(x.high-x.low+1 for x in unknown),
                   unresolved_targets=sorted({x.target for x in unknown}),
                   catalogue_sha256=hashlib.sha256(json.dumps(identities, sort_keys=True,
                        separators=(',', ':')).encode('ascii')).hexdigest(),
                   payload_sha256=hashlib.sha256(b''.join(pages.page(n) for n in sorted(pages.pages))).hexdigest())
    if json.loads(json.dumps(summary)) != reference['expected']:
        raise ValueError('Changed fixed MIDI declared-page/catalogue reference: ' + json.dumps(summary))
    for row in (0, 0x3040, 0x33ff, 0xedc0, 0xffffff):
        rejects(lambda row=row: pages.page(row), 'Unknown declared sample page')
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(dict(passed=True, **summary, scope=reference['qualification'],
                                         fuzz_execution=False, physical_traffic=False), indent=2)+'\n')
    print('Verified original MIDI declared page map, logical catalogue/PCM and explicit unresolved references')
    return 0


if __name__ == '__main__':
    if len(sys.argv) == 1:
        controls()
    elif len(sys.argv) == 3 and sys.argv[1] == '--reference':
        sys.exit(reference_check(Path(sys.argv[2])))
    else:
        sys.exit('Usage: sample_midi_pages_test.py [--reference private.report]')
