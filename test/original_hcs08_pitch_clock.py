# SPDX-License-Identifier: GPL-3.0-only
"""Original pitch arithmetic/ring and overlapping ADC work on a declared reference clock."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row
from original_hcs08_pitch_event import cases, expected

CUTS = (0, 1, 114, 115, 290, 291, 65535)


def validate_clock(values, case):
    keys = {'periods', 'instructions', 'retired', 'partial', 'executing', 'offered', 'start',
            'complete', 'adc', 'ccf', 'conversions', 'remaining', 'tx_read', 'control', 'masked'}
    if set(values) != keys:
        raise ValueError('Changed common-clock observation schema')
    periods, instructions = values['periods'], values['instructions']
    event = expected(case)['emitted']
    fixed = dict(retired=instructions+3, partial=0, executing=periods+13, offered=periods+189,
                 start=115, complete=291, adc=1, ccf=1, conversions=2, remaining=0,
                 tx_read=case['cursor'], control=116 if event else 84, masked=1)
    if (any(values[k] != v for k,v in fixed.items()) or not 53 <= instructions < 2048 or
            not 291 <= periods <= 11*instructions):
        raise ValueError('Changed CPU budget, ADC overlap, retained ring cursor or masked SPI source')


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated clocked pitch trace')
    lines = iter(text.splitlines())
    for case in cases():
        wanted = expected(case)
        wanted.update(rx=0, irqs=0, foreground=0, pc=0xe7ae)
        if row(next(lines)) != ('CASE', wanted):
            raise ValueError('Changed original clocked pitch/filter/ring output')
        tag, values = row(next(lines))
        if tag != 'CLOCK':
            raise ValueError('Missing actual CPU/ADC reference-clock observation')
        validate_clock(values, case)
    for cut in CUTS:
        previous = None
        for index in range(2):
            tag, values = row(next(lines))
            if (tag != 'CLOCK_REPLAY' or set(values) != {'cut','phase','remaining','index','identical'} or
                    values['cut'] != cut or values['index'] != index or values['identical'] != 1 or
                    not 0 <= values['remaining'] <= 10):
                raise ValueError('Missing joint CPU/RAM/ADC/BDC/clock continuation')
            if cut == 65535:
                if not 1 <= values['phase'] < 2048 or values['remaining'] != 3:
                    raise ValueError('Joint snapshot must actually interrupt a five-period MUL')
            elif values['phase'] != cut:
                raise ValueError('Joint snapshot moved away from its declared reference edge')
            if cut in (0,1,114,115) and values['remaining'] != {0:0,1:4,114:1,115:0}[cut]:
                raise ValueError('BRSET/next-channel start snapshots lost independent remaining budgets')
            phase = (values['phase'], values['remaining'])
            if previous is not None and phase != previous:
                raise ValueError('Fresh repetitions changed the selected joint cut')
            previous = phase
    if next(lines, None) is not None:
        raise ValueError('Extra clocked pitch trace suffix')


def controls(text):
    examples = [text.rstrip('\n'), text+'EXTRA value=1\n']
    lines = text.splitlines()
    # Finite directed collector controls, never randomized parser/fuzz traffic.
    for tag, field in [('CASE','packet'), ('CASE','tx'), ('CASE','rx'), ('CASE','pc'),
                       ('CASE','filter'), ('CASE','last'), ('CASE','offset'), ('CASE','emitted'),
                       ('CLOCK','periods'), ('CLOCK','instructions'), ('CLOCK','retired'),
                       ('CLOCK','partial'), ('CLOCK','executing'), ('CLOCK','offered'),
                       ('CLOCK','start'), ('CLOCK','complete'), ('CLOCK','ccf'), ('CLOCK','conversions'),
                       ('CLOCK','remaining'), ('CLOCK','tx_read'), ('CLOCK','control'),
                       ('CLOCK','masked'), ('CLOCK_REPLAY','identical'), ('CLOCK_REPLAY','phase')]:
        index = next(n for n,line in enumerate(lines) if line.startswith(tag+' '))
        parsed_tag, values = row(lines[index])
        values[field] += 1
        changed = list(lines)
        changed[index] = parsed_tag+' '+' '.join(f'{k}={v}' for k,v in values.items())
        examples.append('\n'.join(changed)+'\n')
    for changed in examples:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Collector accepted a directed clocked-pitch corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path,sys.argv[1:])
    if not archive.is_file():
        print('SKIP original clocked pitch: pinned local Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True,exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p:hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary,archive,specification,supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary,supplied,trace)]+['--clocked'],
                         capture_output=True,text=True,timeout=75,check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    if (run.returncode or run.stderr or run.stdout.strip() !=
            'Original pitch filter/ring with masked IRQ and common ADC/CPU clock pass.'):
        raise ValueError('Original clocked pitch execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != d for p,d in frozen.items()):
        raise ValueError('Clocked pitch inputs changed during native execution')
    clocks = [row(line)[1] for line in text.splitlines() if line.startswith('CLOCK ')]
    report = dict(passed=True,declared_component_contexts=68,reference_gain_contexts=66,
                  synthetic_coefficient_width_controls=2,dynamic_filter_contexts=8,
                  queued_packets=sum(expected(c)['emitted'] for c in cases()),
                  literal_mapping_controls=17,setup_periods=13,held_conversion_periods=176,next_start_period=115,
                  next_complete_period=291,conversions_per_context=2,actual_IRQ_entries=0,
                  fresh_joint_replays=14,checkpoint_cuts=7,partial_MUL_remaining=3,
                  collector_controls=negative,observed_poll_period_range=[min(c['periods'] for c in clocks),
                  max(c['periods'] for c in clocks)],physical_clock_or_SPI_timing_qualified=False,
                  boot_origin_calibration=False,native_DSP_firmware_executed=False,full_boot=False,
                  physical_traffic=False,fuzz_execution=False,
                  trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  input_sha256={str(p):d for p,d in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified original pitch arithmetic/ring and common ADC/CPU reference-clock overlap.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
