# SPDX-License-Identifier: GPL-3.0-only
"""Independent calibrated pitch arithmetic and original IRQ/SPI/SHI output oracle."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row


def mapped(working, gain):
    if working <= 2048:
        return 0 if working <= 64 else min(8192, ((working-64)*gain) & 65535)
    if working <= 2112:
        return 8192
    product = ((working-2112)*gain) & 65535
    return 8192 if product <= 600 else min(16383, (product+7592) & 65535)


def cases():
    working = (0, 63, 64, 65, 256, 1000, 1702, 1703, 2048, 2112, 2113, 2232, 2233, 3000, 4092)
    # Literal boundary controls independently constrain the high-level integer oracle.
    literal = (0, 0, 0, 5, 960, 4680, 8190, 8192, 8192, 8192, 8192, 8192, 8197, 12032, 16383)
    if (tuple(mapped(w, 5) for w in working) != literal or
            mapped(325, 252) != 236 or mapped(2352, 252) != 2536):
        raise ValueError('Independent literal pitch mapping controls changed')
    values = [(w, previous, 5) for w in working for previous in (0, 8192, 16383)]
    values += [(1000, p, 5) for p in (4680, 4711, 4712, 4648, 4647)]
    values += [(3000, p, 5) for p in (12032, 12063, 12064, 12000, 11999)]
    values += [(0, 1, 5), (2048, 8191, 5), (4092, 16382, 5), (325, 0, 252), (2352, 0, 252)]
    declared = [(w,p,g,0) for w,p,g in values]
    declared += [(w,0,5,offset) for w in (1000,3000) for offset in (-64,64,-63,63)]
    return [dict(working=w, previous=p, gain=g, cursor=(0,253,254,255)[n % 4], offset=offset)
            for n, (w,p,g,offset) in enumerate(declared)]


def expected(case):
    old_filter = case['working']*4+case['offset']
    accumulated = (old_filter//16-case['offset'])*16
    filtered = (old_filter+accumulated//16-old_filter//16) & 65535
    if filtered != case['working']*4:
        raise ValueError('Own initial filter/sum must imply its independently specified new value')
    value = mapped(filtered//4, case['gain'])
    previous = case['previous']
    difference = (previous+32-value) & 65535
    event = difference > 63 or (previous != value and value in (0,8192,16383))
    packet = (0xe0 << 16) | ((value & 127) << 8) | ((value >> 7) & 127)
    return dict(**case, last=value if event else previous, filter=filtered,
                sum=0, counter=17, emitted=int(event), packet=packet if event else 0,
                tx=(case['cursor']+(3 if event else 0)) & 255, rx=3 if event else 0,
                irqs=4 if event else 0, foreground=3 if event else 0,
                pc=0xe211 if event else 0xe7ae, sp=0x475)


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original pitch event trace')
    lines = iter(text.splitlines())
    for case in cases():
        if row(next(lines)) != ('CASE', expected(case)):
            raise ValueError('Changed original filter, width, threshold, ring or actual SPI packet')
    for cut in (0, 1, 24, 60, 100, 140):
        for index in range(2):
            if row(next(lines)) != ('REPLAY', dict(cut=cut, index=index, identical=1)):
                raise ValueError('Missing original pitch producer/IRQ/SHI fresh continuation')
    if next(lines, None) is not None:
        raise ValueError('Extra original pitch trace suffix')


def controls(text):
    changes = [('filter=252', 'filter=251'), ('offset=-64', 'offset=-63'), ('sum=0', 'sum=1'), ('counter=17', 'counter=16'),
               ('last=4680', 'last=4681'), ('gain=252', 'gain=251'),
               ('emitted=1', 'emitted=0'), ('packet=14680064', 'packet=14680065'),
               ('tx=0', 'tx=1'), ('rx=3', 'rx=2'), ('irqs=4', 'irqs=3'),
               ('foreground=3', 'foreground=0'), ('pc=57873', 'pc=59310'),
               ('sp=1141', 'sp=1140'), ('identical=1', 'identical=0')]
    examples = [text.rstrip('\n'), text+'EXTRA value=1\n']
    for before, after in changes:
        if before not in text:
            raise ValueError('Invalid fixed pitch collector control: '+before)
        examples.append(text.replace(before, after, 1))
    for changed in examples:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Collector accepted a fixed original pitch corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original pitch event: pinned local Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary,supplied,trace)],
                         capture_output=True, text=True, timeout=75, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original pitch filter, integer mapping, event ring and SPI IRQ/SHI delivery pass.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original pitch execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p,digest in frozen.items()):
        raise ValueError('Original pitch inputs changed during execution')
    outputs = [expected(case) for case in cases()]
    packets = sum(case['emitted'] for case in outputs)
    report = dict(passed=True, declared_component_contexts=len(outputs), reference_gain_contexts=66, dynamic_filter_contexts=8,
        synthetic_coefficient_width_controls=2, emitted_packets=packets,
        baseline_IRQ_entries=packets*4, baseline_SPSCK_edges=packets*48,
        actual_IRQ_entries=(packets+13)*4, actual_SPSCK_edges=(packets+13)*48,
        reference_analog_input_code=0, literal_mapping_controls=17,
        fresh_joint_replays=12, checkpoint_cuts=6, collector_controls=negative,
        physical_ADC_or_SPI_timing_qualified=False, boot_origin_calibration=False,
        native_DSP_firmware_executed=False, full_boot=False, physical_traffic=False,
        fuzz_execution=False, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p):digest for p,digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified original pitch arithmetic, change suppression and native SPI event delivery.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
