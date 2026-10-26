# SPDX-License-Identifier: GPL-3.0-only
"""Original Keyboard whole-board warm/cold reset cuts and strict input recovery."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def validate(text):
    expected = [
        'RESET cause=1 cut=0 periods=22700 retired=6195 pc=57794',
        'RESET cause=1 cut=1 periods=22700 retired=6195 pc=57794',
        'RESET cause=1 cut=22671 periods=22700 retired=6195 pc=57794',
        'RESET cause=1 cut=22700 periods=22700 retired=6195 pc=57794',
        'RESET cause=2 cut=0 periods=22700 retired=6195 pc=57794',
        'RESET cause=2 cut=1 periods=22700 retired=6195 pc=57794',
        'RESET cause=2 cut=22671 periods=22700 retired=6195 pc=57794',
        'RESET cause=2 cut=22700 periods=22700 retired=6195 pc=57794',
        'RECOVERY periods=22700 retired=6195 pc=57794',
        'Verified original HCS08 warm/cold reset cuts and missing-input recovery.',
    ]
    if text.splitlines() != expected:
        raise ValueError('Changed original reset cause,cut,period or native state')


def controls(text):
    variants = [text + 'EXTRA\n', '\n'.join(text.splitlines()[1:]) + '\n',
                '\n'.join(text.splitlines()[:-1]) + '\n']
    changes = [('cause=1', 'cause=2'), ('cause=2', 'cause=1'),
               ('cut=0', 'cut=2'), ('cut=1', 'cut=0'), ('cut=22671', 'cut=22670'),
               ('cut=22700', 'cut=22701'), ('periods=22700', 'periods=22699'),
               ('retired=6195', 'retired=6196'), ('pc=57794', 'pc=57344'),
               ('RECOVERY', 'RESET')]
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
                            text=True, timeout=120, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr:
        raise ValueError('Original reference board failed:' + result.stdout + result.stderr)
    validate(result.stdout)
    rejected = controls(result.stdout)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in pins.items()):
        raise ValueError('Original reference-board input changed during execution')
    trace.write_text(result.stdout)
    report = dict(passed=True, full_boot=False, hardware_equivalence=False,
                  original_initial_power_on_starts=11, new_reset_constructions=12,
                  directed_reset_cuts=8, missing_ADC_reference_rejections=2,
                  collector_controls=rejected, bounded_periods=22700,
                  observed_retired=6195, observed_pc=57794, ADC_remaining=165,
                  declared_ADC=dict(bus_hz=12000000, supply_uv=3300000, channel0_uv=0),
                  declared_timer_phase=0, actual_ICG_rate_established=False,
                  physical_reset_timing_qualified=False, original_code_modified=False,
                  fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(path): digest for path, digest in pins.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified original warm/cold board reset cuts with strict physical-input limits.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
