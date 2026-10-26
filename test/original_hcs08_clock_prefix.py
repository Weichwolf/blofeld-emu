# SPDX-License-Identifier: GPL-3.0-only
"""Conditional original reset with actual CPU/ADC/TPM/COP bus-period coupling."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def expected_lines():
    return [
        'BEGIN phase=0',
        'PROFILE phase=0 periods=4022751 retired=1149066 services=285714 pc=60040',
        'BEGIN phase=127',
        'PROFILE phase=127 periods=4022625 retired=1149030 services=285705 pc=60040',
        'Qualified conditional original reset with real coupled ADC/TPM/COP bus periods.',
    ]


def validate(text):
    if text.splitlines() != expected_lines():
        raise ValueError('Changed original coupled prefix, phase/period/retirement/service frontier')


def controls(text):
    examples = [text + 'EXTRA\n', '\n'.join(text.splitlines()[:-1]) + '\n',
                '\n'.join(text.splitlines()[1:]) + '\n']
    changes = [('phase=127', 'phase=126'), ('periods=4022751', 'periods=4022750'),
               ('retired=1149066', 'retired=1149067'), ('services=285714', 'services=0'),
               ('pc=60040', 'pc=57344'), ('periods=4022625', 'periods=4022624'),
               ('retired=1149030', 'retired=1149066'), ('services=285705', 'services=285714')]
    for before, after in changes:
        if before not in text:
            raise ValueError('Invalid fixed coupled-prefix collector control')
        examples.append(text.replace(before, after, 1))
    for changed in examples:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed wrong-state control')
    return len(examples)


def main():
    if len(sys.argv) != 5:
        raise ValueError('Expected binary, original archive, specification and private trace')
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original coupled prefix: pinned local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    pins = {path: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (binary, archive, specification, pages)}
    result = subprocess.run([str(binary.resolve()), str(pages.resolve())], capture_output=True,
                            text=True, timeout=900, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr:
        raise ValueError('Original coupled prefix failed: ' + result.stdout + result.stderr)
    validate(result.stdout)
    rejected = controls(result.stdout)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in pins.items()):
        raise ValueError('Original coupled-prefix input changed during execution')
    trace.write_text(result.stdout)
    report = dict(passed=True, full_boot=False, hardware_equivalence=False,
                  diagnostic_ADC_reference=dict(bus_hz=12000000, supply_uv=3300000, channel0_uv=0),
                  declared_timer_phases=[0, 127], missing_reference_controls=2,
                  fresh_bounded_joint_replays=2, collector_controls=rejected,
                  whole_original_resets=4, original_code_modified=False,
                  actual_ICG_rate_established=False, physical_timing_qualified=False,
                  fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(path): digest for path, digest in pins.items()},
                  trace_sha256=hashlib.sha256(result.stdout.encode()).hexdigest())
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified conditional original reset, actual device clocks and strict input/NVM frontiers.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
