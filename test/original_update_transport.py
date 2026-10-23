# SPDX-License-Identifier: GPL-3.0-only
"""Execute official packet bodies through the original DSP command decoder."""

import copy
import hashlib
import json
from pathlib import Path
import os
import re
import subprocess
import sys

from original_dsp_download import fixture
from original_dsp_voice import original_records
from inspect_firmware import decode_block, load_input, read_smf, split_blocks
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
LAYOUTS = ((3, 0xef00, 256, 256, 37), (0, 0x2e00, 63, 63, 9),
           (3, 0x10000, 2883, 2884, 412), (0, 0x2e00, 388, 389, 56),
           (3, 0xd000, 29881, 29883, 4269), (0, 0x2e00, 400, 401, 58))


def references(firmware, destination):
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    midi, _ = load_input(firmware)
    blocks = split_blocks(read_smf(midi)[0])
    result, lines = [], [str(len(LAYOUTS))]
    for block, (mode, address, declared, stored, count) in zip(blocks, LAYOUTS, strict=True):
        _, data, info = decode_block(block)
        if (info['mode'], info['address_words'], len(data) // 3, len(block['packets'])) != (
                mode, address, declared, count):
            raise ValueError('Changed original six-block transfer layout')
        packets = [block['header'][:-1]] + [packet[:-1] for packet in block['packets']]
        words, sizes = [], []
        for packet in packets[1:]:
            # Independent bit placement and byte-triple assembly, including full padding words.
            decoded = []
            for offset in range(6, len(packet) - 1, 8):
                flags = packet[offset]
                decoded.extend(packet[offset + 1 + n] + 128 * ((flags >> (6 - n)) & 1)
                               for n in range(7))
            chunk = [decoded[n] + 256 * decoded[n + 1] + 65536 * decoded[n + 2]
                     for n in range(0, len(decoded) - 2, 3)]
            words.extend(chunk)
            sizes.append(len(chunk))
        if (len(words) != stored or b''.join(word.to_bytes(3, 'big') for word in words[:declared]) != data):
            raise ValueError('Independent packet word assembly differs from declared original payload')
        lines.extend((f'{len(packets)} {len(words)}', ' '.join(map(str, words))))
        lines.extend(str(len(packet)) + ' ' + ' '.join(map(str, packet)) for packet in packets)
        result.append(dict(packets=packets, words=words, sizes=sizes))
    destination.write_text('\n'.join(lines) + '\n')
    return result


def collect(path):
    blocks, current = [], None
    for line in path.read_text().splitlines():
        tag, _, body = line.partition(' ')
        if tag == 'BLOCK':
            current = dict(index=fields(body)['index'], rows=[])
            blocks.append(current)
        elif current is not None and tag in ('SETUP', 'STORE', 'PREFETCH', 'PACKET', 'INTERRUPTED', 'RESULT'):
            current['rows'].append((tag, fields(body)))
        else:
            raise ValueError('Unexpected original transfer trace row')
    return blocks


def validate(observed, reference):
    if [block['index'] for block in observed] != list(range(6)):
        raise ValueError('BLOCK all six isolated original transfers are required')
    total_steps = 0
    for block, expected, (mode, address, declared, stored, count) in zip(
            observed, reference, LAYOUTS, strict=True):
        rows = iter(block['rows'])
        def take(tag):
            actual_tag, value = next(rows, ('MISSING', {}))
            if actual_tag != tag:
                raise ValueError(tag + ' missing or out of original execution order')
            return value
        setup = take('SETUP')
        if setup != dict(instructions=4, pc=0xff0005, omr=0x600389):
            raise ValueError('SETUP native OMR instruction and three settling NOPs differ')
        header = take('PACKET')
        returned = header['pc']
        if (header['index'] != 0 or header['stored'] or header['cursor'] != address
                or header['sequence'] or header['sp'] or header['prefetches']
                or not 0xff0000 <= returned < 0xff0080):
            raise ValueError('HEADER original configuration/caller differs')
        steps = setup['instructions'] + header['steps']
        written = 0
        for packet in range(1, count + 1):
            if packet == count:
                if take('INTERRUPTED') != dict(stored=written, cursor=address + written):
                    raise ValueError('INTERRUPTED no final-packet destination words may exist yet')
            original = expected['packets'][packet]
            prefetched = take('PREFETCH')
            if (prefetched['packet'] != packet or prefetched['address'] != 0x778 + len(original) - 1
                    or prefetched['value'] != original[-1] << 16
                    or not 0x2361 <= prefetched['pc'] <= 0x23d7):
                raise ValueError('PREFETCH exact original trailing checksum read differs')
            for _ in range(expected['sizes'][packet - 1]):
                if take('STORE') != dict(index=written, space=0 if mode == 0 else 2,
                                         address=address + written,
                                         value=expected['words'][written], pc=0x23c6):
                    raise ValueError('STORE native ordered address/value/space differs')
                written += 1
            state = take('PACKET')
            launch = mode == 0 and packet == count
            fixed = {key: value for key, value in state.items() if key != 'steps'}
            if (fixed != dict(index=packet, stored=written, cursor=address + written,
                              sequence=(packet % 128) << 16, pc=address if launch else returned,
                              sp=int(launch), prefetches=packet)
                    or not 0 < state['steps'] < 2048):
                raise ValueError('PACKET original cursor/sequence/return or real launch frame differs')
            steps += state['steps']
        result = take('RESULT')
        if result != dict(stored=stored, declared=declared, extra=stored - declared,
                          prefetches=count, steps=steps, launched=int(mode == 0), guards=1):
            raise ValueError('RESULT complete original padding, storage guards or launch differs')
        if next(rows, None) is not None:
            raise ValueError('RESULT unexpected execution after transfer boundary')
        total_steps += steps
    return dict(blocks=6, packet_calls=4847, checksum_prefetches=4841,
                destination_stores=33876, declared_words=33871, complete_padding_words=5,
                interrupted_prefixes=6, helper_entries=3, native_steps=total_steps)


def controls(observed, reference):
    for tag, key in (('STORE', 'value'), ('STORE', 'address'), ('PREFETCH', 'value'),
                     ('PACKET', 'sequence'), ('INTERRUPTED', 'stored'), ('RESULT', 'extra')):
        changed = copy.deepcopy(observed)
        row = next(value for name, value in changed[4]['rows'] if name == tag)
        row[key] += 1
        try:
            validate(changed, reference)
        except ValueError:
            continue
        raise ValueError('Transfer oracle accepted fixed ' + tag + ' corruption')
    return 6


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    if not firmware.is_file() or not assembler.is_file():
        print('SKIP original update transport: local firmware or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    packets = trace.with_suffix('.packets')
    expected = references(firmware, packets)
    templates, provenance = fixture(assembler, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, packets, templates)}
    native = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(packets.resolve()),
                             str(templates.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=90, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Six original update transfers and their complete padding stores verified.'
    if native.returncode or any(line and line != allowed and
                               not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                               for line in output.splitlines()):
        raise ValueError('Original update transfer execution failed: ' + output)
    observed = collect(trace)
    totals = validate(observed, expected)
    corruptions = controls(observed, expected)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original transfer input changed during execution')
    hashes = [hashlib.sha256(b''.join(word.to_bytes(3, 'big') for word in block['words'])).hexdigest()
              for block in expected]
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=corruptions,
        stored_block_sha256=hashes, independent_assembler=provenance,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        main_first_additional_address=0x144b9, main_additional_words=2,
        full_boot=False, complete_update=False, helper_bodies_executed=False,
        fuzz_execution=False, physical_traffic=False,
        scope='Six isolated original DSP command transfers with exact official packet bodies '
              'in a declared high-aligned input buffer and own linear/unscaled native caller. '
              'Original RAM stores, live patches, full padding words, sequence wrap, packet-boundary '
              'interrupted prefixes and real JMP frames. No original wire producer/dispatch, '
              'helper bodies, calibrated timing, flash programming or complete boot/update/recovery.'), indent=2) + '\n')
    print('Verified6 original transfers,33876 native stores and5 complete padding words.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
