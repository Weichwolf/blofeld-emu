# SPDX-License-Identifier: GPL-3.0-only
"""Original IRQ queue consumption: real PSHH s/p cycles, strict next BRCLR boundary."""

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
        raise ValueError('Changed original SPI vector')

    def endpoint(seed):
        return 'END', dict(seed=seed, pc=0xea2a, sp=0x479, ccr=0x69, known=0xef,
                           pushed=seed, retired=1, ready=2, inflight=1, next=0xea2d,
                           q0=image[0xea2a], q1=image[0xea2b], q2=image[0xea2c])

    rows = [endpoint(seed) for seed in range(8)]
    for cut in range(3):
        for replay in range(2):
            rows.extend([('REPLAY', dict(cut=cut, index=replay)), endpoint(7)])
    rows.append(('SUMMARY', dict(periods=146, baselines=8, replays=6, retired=1)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated prefetch trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate prefetch field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original queued instruction, stack, phase or fresh continuation')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:])]
    for field in ('pc', 'sp', 'ccr', 'known', 'pushed', 'retired', 'ready', 'inflight', 'next',
                  'q0', 'q1', 'q2'):
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed prefetch corruption')
    return len(invalid)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original prefetch: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    result = subprocess.run([str(p.resolve()) for p in (binary, supplied, trace)],
                            capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original IRQ-prefetched PSHH executes native stack/program cycles and fresh continuations.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original prefetch failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original prefetch inputs changed')
    report = dict(passed=True, foreground_contexts=8, checkpoint_cuts=3, fresh_joint_replays=6,
        checked_endpoints=14, successful_bus_advance_calls=146,
        refused_instruction_attempts=8, total_bus_advance_calls=154, native_instructions_per_endpoint=1,
        original_PSHH_s_then_p=True, strict_next_BRCLR=True, collector_controls=negative,
        host_injected_handler_PC_frame_or_instruction=False, full_controller_boot=False,
        general_pipeline_qualified=False, physical_register_phase_qualified=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified native original prefetched PSHH, actual s/p phases and strict BRCLR boundary.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
