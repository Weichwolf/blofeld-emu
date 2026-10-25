# SPDX-License-Identifier: GPL-3.0-only
"""Original timer loop through real SPI/flash setup under declared BUSCLK histories."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row

CASES = ((0, 4000013, 1142861, 285714, 9), (127, 3999887, 1142825, 285705, 10))


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original timer trace')
    lines = iter(text.splitlines())
    for phase, periods, retired, services, divider in CASES:
        if row(next(lines)) != ('ENTRY', dict(phase=phase, **{'hx-known': 65535 if phase == 0 else 0,
                                                            'a-known': 255, 'ccr-known': 239})):
            raise ValueError('Changed declared entry register knowledge')
        if row(next(lines)) != ('EXIT', dict(phase=phase, periods=periods, retired=retired,
                pc=0xe1d1, hx=31250, services=services, counter=31250, divider=divider, **{'cop-age': 10})):
            raise ValueError('Changed original timer exit, service or period oracle')
        if row(next(lines)) != ('SPI', dict(phase=phase, periods=periods+37, retired=retired+10,
                control=0x74, baud=0x12, divisor=16, irq=1, **{'cpu-i': 1}, edges=0, samples=0,
                **{'flash-divider': 0x3d, 'stack-bytes': 2})):
            raise ValueError('Changed original SPI/flash setup, stack or IRQ masking')
        error = next(lines)
        if (not error.startswith('ERROR HCS08 execution pc=0xea88: ') or
                'register write address=0x1825 pc=0xea88 ccr=0x006c' not in error or
                'unqualified idle FCBEF launch without a staged command' not in error):
            raise ValueError('Original idle FSTAT write must keep its actual unsupported boundary')
        if row(next(lines)) != ('FAULT', dict(phase=phase, periods=periods+41,
                                              retired=retired+10, pc=0xea88)):
            raise ValueError('Changed original FSTAT fault retirement or elapsed budget')
        for cut in range(2):
            for replay in range(2):
                if row(next(lines)) != ('REPLAY', dict(phase=phase, cut=cut, index=replay, identical=1)):
                    raise ValueError('Missing original joint partial-instruction replay')
    if row(next(lines)) != ('DEADLINE', dict(periods=20, offered=19, retired=5,
                                            pc=0xe1c7, remaining=2, services=0)):
        raise ValueError('An expired watchdog must precede the first original service retirement')
    if next(lines) != ('ERROR HCS08 COP timeout address=0x1800 pc=0xe1c7 ccr=0x006d: '
                       'coordinated reset required'):
        raise ValueError('Missing actual COP timeout operation/address/PC/context')
    if next(lines, None) is not None:
        raise ValueError('Extra original timer trace suffix')


def controls(text):
    changes = [text.replace('periods=4000013', 'periods=4000012', 1),
               text.replace('hx-known=0', 'hx-known=65535', 1),
               text.replace('counter=31250', 'counter=31249', 1),
               text.replace('services=285714', 'services=285713', 1),
               text.replace('divider=9', 'divider=0', 1),
               text.replace('retired=1142861', 'retired=1142862', 1),
               text.replace('address=0x1825', 'address=0x1824', 1),
               text.replace('divisor=16', 'divisor=8', 1),
               text.replace('cpu-i=1', 'cpu-i=0', 1),
               text.replace('edges=0', 'edges=16', 1),
               text.replace('stack-bytes=2', 'stack-bytes=0', 1),
               text.replace('identical=1', 'identical=0', 1),
               text.replace('offered=19', 'offered=20', 1), text.rstrip('\n'), text+'EXTRA value=1\n']
    for changed in changes:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Directed timer collector corruption was accepted')
    return len(changes)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 timer wait: pinned local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, pages)}
    run = subprocess.run([str(p.resolve()) for p in (binary, pages, trace)],
                         capture_output=True, text=True, timeout=220, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original timer wait reaches its threshold through real TPM counts and COP services.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original timer-loop execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    rejected = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original timer inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, original_cases=2,
        early_COP_reset_cases=1, fresh_joint_replays=8, collector_controls=rejected,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(), physical_frequency_supplied=False,
        physical_divider_phase_qualified=False, per_cycle_bus_phase_qualified=False,
        timeout_service_race_qualified=False, original_clock_ADC_prefix_executed=False,
        original_SPI_and_flash_setup_executed=True, actual_CPU_IRQ_admission=False,
        original_idle_FSTAT_launch_qualified=False, serial_edges_supplied=False,
        full_boot=False, physical_traffic=False, fuzz_execution=False), indent=2)+'\n')
    print('Verified original timer threshold, real watchdog services and deadline admission.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
