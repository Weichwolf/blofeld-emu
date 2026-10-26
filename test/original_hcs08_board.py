# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original Keyboard reset through the production reference board."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def validate(text):
    expected = [
        'MISSING kind=0 period=22690 retired=6193 pc=57788',
        'MISSING kind=1 period=22702 retired=6196 pc=57797',
        'MISSING kind=2 period=22690 retired=6193 pc=57788',
        'BEGIN phase=0',
        'PROFILE phase=0 periods=4022751 retired=1149066 services=285714 pc=60040',
        'BEGIN phase=127',
        'PROFILE phase=127 periods=4022625 retired=1149030 services=285705 pc=60040',
        'Verified original reset through production memory,CPU,BDC and device clock owners.',
    ]
    if text.splitlines() != expected:
        raise ValueError('Changed original reference-board input gate,phase,period or native state')


def controls(text):
    variants = [text + 'EXTRA\n', '\n'.join(text.splitlines()[1:]) + '\n',
                '\n'.join(text.splitlines()[:-1]) + '\n']
    changes = [('kind=2', 'kind=3'), ('period=22690', 'period=22691'),
               ('period=22702', 'period=22701'), ('retired=6193', 'retired=6194'),
               ('periods=4022751', 'periods=4022750'), ('periods=4022625', 'periods=4022624'),
               ('retired=1149066', 'retired=1149065'), ('retired=1149030', 'retired=1149031'),
               ('services=285714', 'services=0'), ('services=285705', 'services=285714'),
               ('pc=60040', 'pc=57344'), ('phase=127', 'phase=126')]
    for before, after in changes:
        variants.append(text.replace(before, after, 1))
    for changed in variants:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed wrong-state board control')
    return len(variants)


def main():
    if len(sys.argv) != 5:
        raise ValueError('Expected binary,original archive,specification and private trace')
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original reference board:pinned local Keyboard archive absent')
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
        raise ValueError('Original reference board failed:' + result.stdout + result.stderr)
    validate(result.stdout)
    rejected = controls(result.stdout)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in pins.items()):
        raise ValueError('Original reference-board input changed during execution')
    trace.write_text(result.stdout)
    report = dict(passed=True, full_boot=False, hardware_equivalence=False,
                  original_normal_resets=5, missing_reference_controls=3,
                  fresh_bounded_joint_continuations=2, collector_controls=rejected,
                  declared_ADC=dict(bus_hz=12000000, supply_uv=3300000, channel0_uv=0),
                  declared_timer_phases=[0, 127], actual_ICG_rate_established=False,
                  physical_timing_qualified=False, original_code_modified=False,
                  fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(path): digest for path, digest in pins.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified conditional native reference board with strict physical-input and NVM limits.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
