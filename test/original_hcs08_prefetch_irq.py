# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original SPI promotion and retained program queue through a second native IRQ."""

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
    def endpoint(seed):
        low = 255-seed
        return 'HANDOFF', dict(seed=seed, pc=0xe000, sp=0x47f, a=low, hx=(seed << 8) | low,
            ccr=0x61, known=0xef, retired=28, irq=2, ack=2, ready=2, inflight=1, next=0xe003,
            q0=image[0xe000], q1=image[0xe001], q2=image[0xe002], tx=0x90, queued=0x3c,
            active=1, edges=0, cursor=2, reads=74, data_accesses=34,
            old_pc=0xe000, old_next=0xe003, old_ready=2, old_inflight=1,
            old_retired=14, old_periods=48)

    rows = [endpoint(seed) for seed in range(8)]
    for cut in range(60):
        for replay in range(2):
            rows.extend([('HREPLAY', dict(cut=cut, index=replay)), endpoint(7)])
    rows.append(('SUMMARY', dict(periods=9794, baselines=8, replays=120, retired=28)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated original queue IRQ trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original queue IRQ field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original SPI source reentry, archived queue or native second frame')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('HREPLAY cut=59 index=1', 'HREPLAY cut=58 index=1'))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed original queue IRQ corruption')
    return len(invalid)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original queue IRQ cycles: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    command = [str(p.resolve()) for p in (binary, supplied, trace)] + ['--handoff']
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original queued SPI byte raises the next IRQ through retained native queues and frames.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original queue IRQ failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original queue IRQ inputs changed')
    report = dict(passed=True, foreground_contexts=8, checkpoint_cuts=60, fresh_joint_replays=120,
        checked_endpoints=128, successful_bus_advance_calls=9794, native_instructions=28,
        actual_queued_byte_promotion=True, real_SPI_request_reentry=True,
        complete_prior_queue_retained=True, pending_arrival_retained=True,
        actual_second_IRQ_frame=True, native_second_RTI=True,
        RTI_IRQ_contract='reference_immediate', program_periods_per_ISR=48,
        SPI_automatic_launch_or_physical_promotion_phase_qualified=False,
        collector_controls=negative, physical_program_addresses_qualified=False,
        physical_register_phase_qualified=False, general_pipeline_qualified=False,
        host_injected_handler_PC_frame_instruction_or_SPI_byte=False,
        full_controller_boot=False, shared_CPU_SPI_scheduler=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original queued SPI promotion and native queue-to-IRQ reentry.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
