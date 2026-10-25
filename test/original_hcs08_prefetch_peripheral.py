# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original SPI control/data writes and genuine indexed/stack reads."""

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
    displacement = int.from_bytes(bytes([image[vector+3]]), 'big', signed=True)
    branch = ((vector+4+displacement) & 0xffff)+6
    empty_offset = int.from_bytes(bytes([image[branch+1]]), 'big', signed=True)
    empty_selected = (branch+2+empty_offset) & 0xffff

    def endpoint(seed, empty):
        pc = empty_selected+4 if empty else branch+17
        return 'PERIPHERAL', dict(empty=int(empty), seed=seed, pc=pc, sp=0x47a,
            a=0 if empty else 0x90, hx=(seed << 8) | ((255-seed) if empty else 0),
            ccr=0x68 if empty else 0x6c, known=0xef, retired=7 if empty else 13,
            ready=2, inflight=1, next=pc+3, q0=image[pc], q1=image[pc+1], q2=image[pc+2],
            control=0x54 if empty else 0x74, queued=-1 if empty else 0x90,
            cursor=0 if empty else 1, read_addr=0x47a, read_value=seed,
            reads=23 if empty else 34, data_reads=4 if empty else 6)

    rows = [endpoint(seed, empty) for seed in range(8) for empty in (False, True)]
    for empty in (False, True):
        periods = 25 if empty else 39
        for cut in range(periods+1):
            for replay in range(2):
                rows.extend([('PREPLAY', dict(empty=int(empty), cut=cut, index=replay)), endpoint(7, empty)])
    rows.append(('SUMMARY', dict(periods=4729, baselines=16, replays=132, retired_empty=7, retired_send=13)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated original peripheral trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original peripheral field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original native write/load/pull, queue or fresh continuation')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('PREPLAY empty=0 cut=39 index=1', 'PREPLAY empty=0 cut=38 index=1'))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed original peripheral corruption')
    return len(invalid)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original peripheral cycles: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    command = [str(p.resolve()) for p in (binary, supplied, trace)] + ['--peripheral']
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original peripheral cycles preserve actual native stores, indexed loads and PULH.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original peripheral cycles failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original peripheral inputs changed')
    report = dict(passed=True, foreground_contexts=16, checkpoint_cuts=66, fresh_joint_replays=132,
        checked_endpoints=148, successful_bus_advance_calls=4729, refused_instruction_attempts=0,
        total_bus_advance_calls=4729, native_instructions_empty=7, native_instructions_send=13,
        actual_SPI_control_and_data_writes=True, actual_TX_cursor_and_indexed_buffer=True,
        actual_saved_H_pull=True, data_reads_empty=4, data_reads_send=6,
        stopped_before_RTI=True, collector_controls=negative,
        reference_policy='native write at w; captured indexed/stack r consumed one period later',
        physical_program_addresses_qualified=False, physical_register_phase_qualified=False,
        SPI_automatic_launch_or_physical_write_phase_qualified=False,
        host_injected_handler_PC_frame_instruction_or_SPI_byte=False,
        full_controller_boot=False, general_pipeline_qualified=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original native peripheral stores, indexed TX load and actual saved-H pull.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
