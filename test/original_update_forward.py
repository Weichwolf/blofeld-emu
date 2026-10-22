# SPDX-License-Identifier: GPL-3.0-only
"""Check unchanged DSP error forwarding through real MCU XIRQ, copy and RAM call."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields
from inspect_firmware import decode_block, load_input, read_smf, split_blocks

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ((0, 0), (0, 18), (1, 0), (1, 18))
CHUNK_PCS = (0x2e63, 0x2e68, 0x2e6d, 0x2e72)


def tagged(body, tag):
    return [fields(row) for row in body.splitlines() if row.startswith(tag + ' ')]


def validate(text, original):
    sections = re.findall(r'^CASE order=(\d+) phase=(\d+)\n([\s\S]*?)(?=^CASE |\Z)', text, re.M)
    if len(sections) != len(MATRIX):
        raise ValueError('CASE incomplete original error-forwarding matrix')
    words = [int.from_bytes(original[n:n + 3], 'big') for n in range(0, 195, 3)]
    samples = [(word >> shift) & 63 for word in words[:64] for shift in (18, 12, 6, 0)]
    queries = irq_queries = unresolved = 0
    for section, expected in zip(sections, MATRIX, strict=True):
        if tuple(map(int, section[:2])) != expected:
            raise ValueError('CASE reordered original forwarding context')
        body = section[2]
        results = tagged(body, 'RESULT')
        if len(results) != 1:
            raise ValueError('RESULT missing original helper entry')
        result = results[0]
        fixed = {key: value for key, value in result.items()
                 if key not in ('queries', 'irq-queries', 'unresolved-events')}
        if (fixed != {'bytes': 188, 'chunks': 256, 'samples': 256, 'source-loads': 65,
                      'irq-entries': 1, 'irq-returns': 1, 'irq-unknown': 0,
                      'unknown-queries': 0, 'mcu-pc': 0x1200, 'mcu-d': 0xf81b,
                      'mcu-sp': 0x21f4, 'nvm-unchanged': 1, 'program-unchanged': 1,
                      'payload-unchanged': 1, 'caller-unchanged': 1, 'board-rejected': 1}
                or any(result.get(key, 0) <= 0 for key in
                       ('queries', 'irq-queries', 'unresolved-events'))):
            raise ValueError('RESULT original transfer/frame/preservation/audit differs')
        if tagged(body, 'SOURCE') != [dict(index=n, address=0x14478 + n, value=word)
                                     for n, word in enumerate(words)]:
            raise ValueError('SOURCE actual original prefetch/load/register sequence differs')
        if tagged(body, 'CHUNK') != [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                                    for n, value in enumerate(samples)]:
            raise ValueError('CHUNK original native MOVEP effect differs')
        if tagged(body, 'CONSUMER') != [dict(index=n, value=value) for n, value in enumerate(samples)]:
            raise ValueError('CONSUMER actual original MCU read sequence differs')
        order = [line.split()[0] for line in body.splitlines()
                 if line.startswith(('CHUNK ', 'CONSUMER '))]
        if order != ['CHUNK', 'CONSUMER'] * 256:
            raise ValueError('CHUNK CONSUMER native producer/consumer order differs')
        if tagged(body, 'IRQ_ENTRY') != [dict(vector=0xfff4, sp=0x21f7)]:
            raise ValueError('IRQ_ENTRY original vector and real hardware frame differ')
        returns = tagged(body, 'IRQ_RETURN')
        if (len(returns) != 1 or returns[0].get('pc') not in (0x1400, 0x1403, 0x1405)
                or returns[0].get('sp') != 0x2200 or len(returns[0]) != 2):
            raise ValueError('IRQ_RETURN original RTI must resume the own native flag loop')
        if tagged(body, 'IRQ_PAD') != [dict(known=3, levels=n) for n in (3, 2, 3)]:
            raise ValueError('IRQ_PAD original DSP pulse must reach the declared resolved route')
        writes = [int(value, 16) for value in
                  re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)$', body, re.M)]
        if writes != [0, 0, 0, 128] + [128, 0, 128, 0] * 64:
            raise ValueError('PHASE original IRQ/receiver Q7 handshake differs')
        pads = tagged(body, 'PAD_QUERY')
        if len(pads) != result['queries']:
            raise ValueError('PAD_QUERY incomplete actual consumption audit')
        for pad in pads:
            if (pad['known'] != pad['requested'] or pad['conflicts']
                    or pad['levels'] & ~pad['known']):
                raise ValueError('PAD_QUERY unresolved voltage was consumed')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET_UNQUALIFIED missing intermediate electrical uncertainty')
        queries += result['queries']
        irq_queries += result['irq-queries']
        unresolved += result['unresolved-events']
    return dict(cases=4, original_irq_entries=4, original_rti_returns=4,
                native_ram_helper_calls=4, original_helper_bytes=752,
                original_source_reads=260, original_output_chunks=1024,
                original_input_samples=1024, known_data_pad_queries=queries,
                known_irq_pad_queries=irq_queries, unresolved_events=unresolved,
                board_rejections=4)


def negative_controls(text, original):
    controls = [text.replace('RESULT bytes=188', 'RESULT bytes=187', 1),
                text.replace('board-rejected=1', 'board-rejected=0', 1),
                re.sub(r'^(SOURCE .*?value=)\d+', r'\g<1>16777216', text, count=1, flags=re.M),
                re.sub(r'^(CHUNK .*?value=)\d+', r'\g<1>64', text, count=1, flags=re.M),
                re.sub(r'^(CONSUMER .*?value=)\d+', r'\g<1>64', text, count=1, flags=re.M),
                text.replace('IRQ_ENTRY vector=65524', 'IRQ_ENTRY vector=65522', 1),
                text.replace('IRQ_PAD known=3 levels=3', 'IRQ_PAD known=3 levels=2', 1),
                re.sub(r'^NET_UNQUALIFIED .*\n', '', text, count=1, flags=re.M)]
    for changed in controls:
        if changed == text:
            raise ValueError('Missing fixed forwarding oracle corruption')
        try:
            validate(changed, original)
        except ValueError:
            continue
        raise ValueError('Forwarding oracle accepted fixed corrupted evidence')
    return len(controls)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original update forwarding: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    midi, _ = load_input(firmware)
    payload = decode_block(split_blocks(read_smf(midi)[0])[4])[1]
    original = payload[(0x14478 - 0xd000) * 3:]
    if len(original) != 195 or int.from_bytes(original[:2], 'big') != 188:
        raise ValueError('Changed original helper framing')
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=110, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    allowed = 'Original DSP/MCU error helper: four native IRQ/transfer/call contexts.'
    if result.returncode or any(line and line != allowed and
                                not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                                for line in diagnostics.splitlines()):
        raise ValueError('Original update forwarding failed: ' + diagnostics)
    text = trace.read_text()
    totals = validate(text, original)
    controls = negative_controls(text, original)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original forwarding input identity changed')
    report.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=controls, board_qualified=False,
        full_boot=False, complete_update=False, fuzz_execution=False, physical_traffic=False,
        scope='Original main-updater bad-marker branch, DSP six-bit sender, native MCU XIRQ, '
              'receiver/copy and RAM call. Declared loaded records, own caller/RAM/clocks, '
              'initial Q7/PE9 outputs and direct C/Q plus PE9/XIRQ route. No per-group host '
              'input or acknowledgement. Stops before the RAM helper executes; physical '
              'routing/timing, full boot, NAND programming and whole recovery remain open.'), indent=2) + '\n')
    print('Verified4 original DSP-to-MCU IRQ/error-helper transfers and native RAM calls.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
