# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original controller vector/program bytes through all eleven IRQ bus periods."""

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
    if (image[0xffe0] << 8) | image[0xffe1] != 0xea29:
        raise ValueError('Changed pinned original IRQ vector')
    queue = [image[0xea29+i] for i in range(3)]

    def endpoint(seed):
        return 'END', dict(seed=seed, cycles=11, pc=0xea29, sp=0x47a, ccr=0x69,
                           known=0xef, ready=2, inflight=1, q0=queue[0], q1=queue[1],
                           q2=queue[2], retired=0)

    rows = [endpoint(seed) for seed in range(8)]
    for cut in range(12):
        for replay in range(2):
            rows.append(('REPLAY', dict(cut=cut, index=replay)))
            rows.append(endpoint(7))
    rows.append(('SUMMARY', dict(periods=286, baseline=8, replays=24, retired=0)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated original IRQ bus trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original IRQ bus field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original IRQ period/frame/queue availability or joint restore')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    examples = [text[:-1], text+'EXTRA value=1\n', ''.join(lines[1:]), text+lines[-1]]
    changes = [('END ', 'cycles=11', 'cycles=9'), ('END ', 'pc=59945', 'pc=57344'),
               ('END ', 'sp=1146', 'sp=1151'), ('END ', 'ccr=105', 'ccr=97'),
               ('END ', 'known=239', 'known=255'), ('END ', 'ready=2', 'ready=3'),
               ('END ', 'inflight=1', 'inflight=0'), ('END ', 'retired=0', 'retired=1'),
               ('REPLAY ', 'cut=0', 'cut=1'), ('SUMMARY ', 'periods=286', 'periods=285')]
    for prefix, before, after in changes:
        changed = list(lines)
        at = next(i for i, line in enumerate(lines) if line.startswith(prefix) and before in line)
        changed[at] = changed[at].replace(before, after, 1)
        examples.append(''.join(changed))
    # Reference-derived queue bytes stay local; this mutation never targets physical hardware.
    changed = list(lines)
    first = changed[0].split()
    at = next(i for i, field in enumerate(first) if field.startswith('q0='))
    first[at] = 'q0='+str((int(first[at].split('=')[1])+1) & 255)
    changed[0] = ' '.join(first)+'\n'
    examples.append(''.join(changed))
    for changed in examples:
        try:
            validate(changed, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed original IRQ bus corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original IRQ bus: local pinned Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary, supplied, trace)]+['--irq-bus'],
                         capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original HCS08 IRQ bus sequence retains actual program bytes and joint state.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original IRQ bus execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original IRQ bus inputs changed during execution')
    report = dict(passed=True, declared_foreground_contexts=8, IRQ_bus_periods_per_endpoint=11,
        fresh_joint_replays=24, checkpoint_cuts=12, actual_bus_advance_calls=286,
        native_instruction_retirements=0, captured_program_bytes=3,
        program_bytes_ready_at_period11=2, program_bytes_in_flight_at_period11=1,
        collector_controls=negative, actual_original_vector_and_program_reads=True,
        handler_PC_or_frame_injected=False, instruction_queue_consumption_qualified=False,
        physical_clock_or_bus_phase_qualified=False, full_controller_boot_qualified=False,
        full_update_recovery_qualified=False, native_DSP_firmware_executed=False,
        firmware_modified=False, fuzz_execution=False, physical_traffic=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): sha for p, sha in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original eleven-period IRQ vector/program reads and fresh joint continuations.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
