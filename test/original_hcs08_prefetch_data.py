# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original data/bit-branch execution under an explicit refill reference."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_prefetch import validate as validate_linear


def expected_rows(pages):
    image = {}
    for start in range(0, len(pages), 514):
        address = int.from_bytes(pages[start:start+2], 'big')
        image.update((address+i, byte) for i, byte in enumerate(pages[start+2:start+514]))
    if (image[0xffe0] << 8) | image[0xffe1] != 0xea29:
        raise ValueError('Changed original SPI vector')
    displacement = int.from_bytes(bytes([image[0xea2c]]), 'big', signed=True)
    target = (0xea2d + displacement) & 0xffff
    pc = target + 6

    def endpoint(seed):
        return 'DATA', dict(seed=seed, pc=pc, sp=0x479, a=3,
            hx=(seed << 8) | (255-seed), ccr=0x68, known=0xef, pushed=seed,
            retired=4, ready=2, inflight=1, next=pc+3,
            q0=image[pc], q1=image[pc+1], q2=image[pc+2], operand=0x97, value=0,
            discarded=0xea2d, speculative=image[0xea2d], reads=16, status=0x20)

    rows = [endpoint(seed) for seed in range(8)]
    for cut in range(16):
        for replay in range(2):
            rows.extend([('DREPLAY', dict(cut=cut, index=replay)), endpoint(7)])
    rows.append(('SUMMARY', dict(periods=744, baselines=8, replays=32, retired=4)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated data/branch trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate data/branch field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original data/branch result, actual read or fresh continuation')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('DREPLAY cut=15 index=1', 'DREPLAY cut=14 index=1'))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed data/branch corruption')
    # The retained linear contract has different scope and must not accept this trace.
    try:
        validate_linear(text, pages)
    except ValueError:
        return len(invalid)+1
    raise ValueError('Linear collector accepted the data/branch contract')


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original data/branch: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    command = [str(p.resolve()) for p in (binary, supplied, trace)] + ['--data-branch']
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original prefetched data/branch cycles preserve actual reads and native decisions.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original data/branch failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original data/branch inputs changed')
    report = dict(passed=True, foreground_contexts=8, checkpoint_cuts=16, fresh_joint_replays=32,
        checked_endpoints=40, successful_bus_advance_calls=744,
        refused_instruction_attempts=8, total_bus_advance_calls=752,
        native_instructions_per_endpoint=4, original_PSHH_BRCLR_LDA_CMP=True,
        data_reads_per_endpoint=3, strict_next_BEQ=True, collector_controls=negative,
        reference_policy='bit-branch: r1 data; p2 speculation; p3..p5 native selected-PC refill',
        operand_read_periods=dict(direct=1, extended=2, bit_branch=1),
        physical_program_addresses_qualified=False, physical_register_phase_qualified=False,
        host_injected_handler_PC_frame_instruction_or_operand=False,
        full_controller_boot=False, general_pipeline_qualified=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original native data/bit-branch path and fresh read/refill continuations.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
