# SPDX-License-Identifier: GPL-3.0-only
"""Original empty/send BEQ paths under an explicit short-branch refill reference."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def expected_rows(pages):
    image = {}
    for start in range(0, len(pages), 514):
        address = int.from_bytes(pages[start:start+2], 'big')
        image.update((address+i, byte) for i, byte in enumerate(pages[start+2:start+514]))
    vector = (image[0xffe0] << 8) | image[0xffe1]
    if vector != 0xea29:
        raise ValueError('Changed original SPI vector')
    bit_offset = int.from_bytes(bytes([image[vector+3]]), 'big', signed=True)
    branch = ((vector+4+bit_offset) & 0xffff) + 6
    empty_offset = int.from_bytes(bytes([image[branch+1]]), 'big', signed=True)
    empty_pc = (branch+2+empty_offset) & 0xffff

    def endpoint(seed, empty):
        pc = empty_pc if empty else branch+7
        return 'SHORT', dict(empty=int(empty), seed=seed, pc=pc, sp=0x479,
            a=0 if empty else 1, hx=(seed << 8) | ((255-seed) if empty else 0),
            ccr=0x6a if empty else 0x68, known=0xef, pushed=seed,
            retired=5 if empty else 8, ready=2, inflight=1, next=pc+3,
            q0=image[pc], q1=image[pc+1], q2=image[pc+2], discarded=branch+2,
            speculative=image[branch+2], reads=19 if empty else 24, data_reads=3 if empty else 4)

    rows = [endpoint(seed, empty) for seed in range(8) for empty in (False, True)]
    for empty in (False, True):
        periods = 18 if empty else 24
        for cut in range(periods+1):
            for replay in range(2):
                rows.extend([('SREPLAY', dict(empty=int(empty), cut=cut, index=replay)), endpoint(7, empty)])
    rows.append(('SUMMARY', dict(periods=2409, baselines=16, replays=88, retired_empty=5, retired_send=8)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated original short-branch trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original short-branch field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original branch route, native successor, actual read or fresh continuation')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('SREPLAY empty=0 cut=24 index=1', 'SREPLAY empty=0 cut=23 index=1'))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed original short-branch corruption')
    return len(invalid)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original short-branch: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    command = [str(p.resolve()) for p in (binary, supplied, trace)] + ['--short-branch']
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original BEQ paths preserve native successors and actual short-branch refill.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original short-branch failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original short-branch inputs changed')
    report = dict(passed=True, foreground_contexts=16, checkpoint_cuts=44, fresh_joint_replays=88,
        checked_endpoints=104, successful_bus_advance_calls=2409, refused_instruction_attempts=16,
        total_bus_advance_calls=2425, native_instructions_empty=5, native_instructions_send=8,
        original_BEQ_taken_and_not_taken=True, original_send_LDA_TAX_INCA=True,
        strict_next_MOV_STA=True, collector_controls=negative,
        reference_policy='discard mature sequential successor; native decision; selected-PC p1..p3 refill',
        physical_program_addresses_qualified=False, physical_register_phase_qualified=False,
        host_injected_handler_PC_frame_instruction_or_operand=False,
        full_controller_boot=False, general_pipeline_qualified=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original BEQ routes, native successors and fresh short-branch refill continuations.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
