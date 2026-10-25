# SPDX-License-Identifier: GPL-3.0-only
"""Original ADC execution on an explicit ADC-before-retirement reference clock."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original ADC common-clock trace')
    lines = iter(text.splitlines())
    for sample in (0, 1, 511, 512, 1023):
        for phase in (170, 171, 172, 175, 176):
            if row(next(lines)) != ('CASE', dict(sample=sample, phase=phase, setup=13, known=239)):
                raise ValueError('Changed declared ADC input, phase, setup or unknown H')
            for channel in range(3):
                if phase == 170:
                    if row(next(lines)) != ('MISS', dict(channel=channel, periods=9,
                                                        instructions=2, pending=1)):
                        raise ValueError('Late conversion must remain pending after original JMP')
                expected = dict(channel=channel, periods=156+(channel == 2), instructions=53,
                                sum=0x12f0+sample, counter=2, remaining=135, known=255)
                if row(next(lines)) != ('READY', expected):
                    raise ValueError('Changed native timing, ADC overlap or channel arithmetic')
            executing = 482+(27 if phase == 170 else 0)
            held = 3*phase-82
            expected = dict(edges=executing+held, executing=executing, held=held,
                            retired=162+(6 if phase == 170 else 0), conversions=3,
                            remaining=135, **{'flash-unchanged': 1})
            if row(next(lines)) != ('END', expected):
                raise ValueError('Changed independent common-clock/CPU/ADC totals')
    for cut in (0, 1, 4, 5, 8, 9, 30, 31, 65, 100, 115, 155):
        for index in range(2):
            if row(next(lines)) != ('REPLAY', dict(cut=cut, index=index, identical=1)):
                raise ValueError('Missing fresh CPU/ADC/BDC/clock mid-budget continuation')
    error = next(lines)
    if (not error.startswith('UNKNOWN HCS08 ADC sample address=0x0051 pc=0xe1bc ccr=0x006a ') or
            'remaining=176 channel=0: unknown enabled analog pin=0' not in error):
        raise ValueError('Unknown analog input must fail before offering a CPU period')
    if next(lines, None) is not None:
        raise ValueError('Extra common-clock ADC trace suffix')


def controls(text):
    changes = [('phase=170', 'phase=169'), ('setup=13', 'setup=12'),
               ('known=239', 'known=255'), ('periods=9', 'periods=8'),
               ('pending=1', 'pending=0'), ('periods=156', 'periods=155'),
               ('periods=157', 'periods=156'), ('instructions=53', 'instructions=52'),
               ('sum=4848', 'sum=4849'), ('remaining=135', 'remaining=176'),
               ('executing=509', 'executing=508'), ('held=428', 'held=427'),
               ('retired=168', 'retired=167'), ('identical=1', 'identical=0'),
               ('flash-unchanged=1', 'flash-unchanged=0'), ('address=0x0051', 'address=0x0052')]
    examples = [text.rstrip('\n'), text+'EXTRA value=1\n']
    for before, after in changes:
        if before not in text:
            raise ValueError('Invalid fixed common-clock collector control')
        examples.append(text.replace(before, after, 1))
    for changed in examples:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Collector accepted a fixed common-clock ADC corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original ADC common clock: pinned local Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary, supplied, trace)],
                         capture_output=True, text=True, timeout=75, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original ADC shared-clock retirement boundaries and joint snapshots pass.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original ADC common-clock execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original ADC common-clock inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True,
        declared_component_cases=25, original_ready_paths=75, genuine_busy_paths=15,
        fresh_joint_replays=24, checkpoint_cuts=12, collector_controls=negative, unknown_input_cases=1,
        reference_tie='ADC edge before native instruction retirement',
        physical_bus_phase_or_tie_qualified=False, boot_origin_CCR_or_clock_supplied=False,
        full_analog_event_path=False, physical_ADC_accuracy_qualified=False,
        native_DSP_firmware_executed=False, full_boot=False, physical_traffic=False,
        fuzz_execution=False, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()}), indent=2)+'\n')
    print('Verified original ADC common-clock boundaries, native arithmetic and joint snapshots.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
