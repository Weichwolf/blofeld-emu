# SPDX-License-Identifier: GPL-3.0-only
"""Check the original Keyboard sender, resident copy and helper receiver on one board pair."""

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from keyboard_update_reference import LAYOUTS, verified_blocks
from original_dsp_download import fixture
from original_dsp_voice import ROOT, original_records
from original_gpio_pair import fields
from original_update_transport import packet_references, validate_block

MATRIX = ((0, 0), (0, 18), (1, 0), (1, 18))
CHUNK_PCS = (0x2e1d, 0x2e22, 0x2e27, 0x2e2c)


def rows(text, tag):
    return [fields(line) for line in text.splitlines() if line.startswith(tag + ' ')]


def validate(text, expected):
    cases = re.findall(r'^CASE order=(\d+) phase=(\d+)\n([\s\S]*?)(?=^CASE |\Z)', text, re.M)
    if tuple((int(order), int(phase)) for order, phase, _ in cases) != MATRIX:
        raise ValueError('Incomplete original Keyboard handoff order/phase matrix')
    words = expected[2]['words']
    chunks = [(word >> shift) & 63 for word in words for shift in (18, 12, 6, 0)]
    for _, _, body in cases:
        _, transfers = body.split('PRELOAD block=2\n', 1)
        data, program = transfers.split('PRELOAD block=3\n', 1)
        code, _ = program.split('PRELOAD_END ', 1)
        steps = 0
        for index, section, setup in ((2, data, 5), (3, code, 1)):
            block = dict(rows=[(line.split(' ', 1)[0], fields(line))
                               for line in section.splitlines()])
            steps += validate_block(block, expected[index], LAYOUTS[index], 0xff1000, setup)
        if rows(body, 'PRELOAD_END') != [dict(pc=0x2e00, epoch=1, steps=steps)]:
            raise ValueError('Original packet downloads must retain their actual native epoch/frame')
        if rows(body, 'ENTRY') != [dict(pc=0x1200, d=0xf81b, sp=0x21f4,
                                        chunks=1360, samples=1360, loaded=341)]:
            raise ValueError('Original resident copy/JSR frame and exact helper boundary differ')
        if rows(body, 'SOURCE') != [dict(index=n, address=0xef00 + n, value=value)
                                    for n, value in enumerate(words[:342])]:
            raise ValueError('Actual original sender loads or first controller-word prefetch differ')
        if rows(body, 'CHUNK') != [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                                   for n, value in enumerate(chunks[:1364])]:
            raise ValueError('Original sender six-bit pin output differs')
        if rows(body, 'CONSUMER') != [dict(index=n, value=value)
                                      for n, value in enumerate(chunks[:1360])]:
            raise ValueError('Resident receiver pin samples differ from actual sender')
        if rows(body, 'HELPER_SAMPLE') != [dict(index=n, value=chunks[1360 + n] | (128 if n % 2 == 0 else 0))
                                           for n in range(4)]:
            raise ValueError('Original helper must read the remaining object checksum pins')
        if rows(body, 'BYTE') != [dict(index=n, value=(words[340] >> shift) & 255,
                                       sp=0x21e5 if n < 2 else 0x21e7,
                                       x=0x21ec if n < 2 else 0x21ed)
                                 for n, shift in enumerate((16, 8, 0))]:
            raise ValueError('Native helper initial byte/argument/cache frames differ')
        final = rows(body, 'FINAL')
        if len(final) != 1 or final[0].pop('queries', 0) <= 0 or final[0] != dict(
                chunks=1364, samples=1360, **{'helper-samples': 4}, bytes=3, loaded=342,
                **{'irq-entries': 1, 'irq-returns': 1}, unknown=0, **{'irq-unknown': 0},
                **{'dsp-pc': 0x2e1b, 'mcu-pc': 0x121a}, nvm=1,
                **{'original-source': 1}, caller=1, **{'board-rejected': 1}):
            raise ValueError('Native joint handoff, pad audit or retained source/NVM differs')
    return dict(cases=4, native_download_stores=6400, checksum_prefetches=920,
                packet_calls=928, native_copied_helper_bytes=4072, actual_input_chunks=5456,
                original_initial_returned_bytes=12, native_source_prefetches=1368)


def controls(text, expected):
    for tag, key in (('SOURCE', 'address'), ('CHUNK', 'value'), ('ENTRY', 'sp'), ('BYTE', 'value')):
        changed = re.sub(r'(^' + tag + r' [^\n]*\b' + key + r'=)(\d+)',
                         lambda match: match[1] + str(int(match[2]) + 1), text, count=1, flags=re.M)
        if changed == text:
            raise ValueError('Fixed handoff oracle corruption was not applied')
        try:
            validate(changed, expected)
        except ValueError:
            continue
        raise ValueError('Handoff oracle accepted fixed ' + tag + ' corruption')
    return 4


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    if not binary.is_file():
        raise ValueError('Original Keyboard handoff executable missing')
    if not all(path.is_file() for path in (firmware, keyboard, assembler)):
        print('SKIP original Keyboard handoff: local archives or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    blocks, _, _ = verified_blocks(keyboard)
    packets = trace.with_suffix('.packets')
    expected = packet_references(blocks, LAYOUTS, packets)
    templates, provenance = fixture(assembler, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, packets, templates)}
    native = subprocess.run([str(path.resolve()) for path in (binary, firmware, packets, templates, trace)],
                            capture_output=True, text=True, timeout=270, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Four original Keyboard DSP/MCU copy and receiver handoffs verified.'
    if native.returncode or any(line and line != allowed and
                               not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                               for line in output.splitlines()):
        raise ValueError('Native Keyboard handoff failed: ' + output)
    text = trace.read_text()
    counts = validate(text, expected)
    corruptions = controls(text, expected)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original Keyboard handoff input changed during execution')
    receipt.write_text(json.dumps(dict(passed=True, **counts, fixed_oracle_corruptions=corruptions,
        independent_assembler=provenance, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, external_controller_executed=False,
        fuzz_execution=False, physical_traffic=False,
        scope='Original two stage2 packet bodies, DSP sender, XIRQ/resident MCU copy/JSR and '
              'initial helper checksum-word receiver calls on one clocked pair. Own callers, '
              'declared clocks/routes/pads; no stage1/short-message outer dispatch, controller '
              'ISA/image/programming/recovery, physical wiring/timing, normal boot/MIDI/audio '
              'or complete update qualification.'), indent=2) + '\n')
    print('Verified four native Keyboard sender/copy/helper-receiver handoffs.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
