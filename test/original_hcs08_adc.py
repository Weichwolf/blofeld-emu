# SPDX-License-Identifier: GPL-3.0-only
"""Original ADC paths with declared stable analog inputs and untimed CPU entry."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original ADC trace')
    lines = iter(text.splitlines())
    for sample in (0, 1, 511, 512, 1023):
        for flags, known in ((0x68, 0xff), (0x78, 0xff), (0x68, 0xef)):
            if row(next(lines)) != ('CASE', dict(code=sample, flags=flags, known=known,
                                               prefix=3, remaining=176)):
                raise ValueError('Changed declared ADC sample, foreground or original setup')
            for cut in (0, 1, 175):
                for index in range(2):
                    if row(next(lines)) != ('REPLAY', dict(cut=cut, index=index, identical=1)):
                        raise ValueError('Missing independent joint ADC continuation')
            for channel in range(3):
                expected = dict(channel=channel, next=(channel+1) % 3, code=sample,
                                sum=0x12f0+sample, counter=2, instructions=55,
                                conversions=channel+1, remaining=176, pc=0xe7ae,
                                sp=0x475, hx=0xb8+channel*2, a=1, ccr=0x68,
                                **{'initial-known': known if channel == 0 else 0xff,
                                   'ccr-known': 0xff, 'h-defined': 1})
                if row(next(lines)) != ('SAMPLE', expected):
                    raise ValueError('Changed native ADC dispatch, sum, channel, stack or period')
            if row(next(lines)) != ('END', dict(retired=168, conversions=3, next=0,
                                               **{'flash-unchanged': 1})):
                raise ValueError('Changed total original ADC execution or FLASH identity')
    error = next(lines)
    if (not error.startswith('UNKNOWN HCS08 ADC sample address=0x0051 pc=0xe1bc ccr=0x006a ') or
            'remaining=176 channel=0: unknown enabled analog pin=0' not in error):
        raise ValueError('Unknown real analog pin must fail with actual context before progress')
    if next(lines, None) is not None:
        raise ValueError('Extra original ADC trace suffix')


def controls(text):
    changes = [('remaining=176', 'remaining=175'), ('sum=4848', 'sum=4849'),
               ('counter=2', 'counter=1'), ('instructions=55', 'instructions=54'),
               ('next=1', 'next=2'), ('hx=184', 'hx=186'), ('ccr=104', 'ccr=120'),
               ('identical=1', 'identical=0'), ('conversions=3', 'conversions=4'),
               ('address=0x0051', 'address=0x0052'),
               ('known=239', 'known=255'), ('initial-known=239', 'initial-known=255'),
               ('ccr-known=255', 'ccr-known=239'), ('h-defined=1', 'h-defined=0'),
               ('flash-unchanged=1', 'flash-unchanged=0')]
    examples = [text.rstrip('\n'), text+'EXTRA value=1\n']
    for before, after in changes:
        if before not in text:
            raise ValueError('Invalid fixed ADC collector control')
        examples.append(text.replace(before, after, 1))
    for changed in examples:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Fixed ADC collector corruption was accepted')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original ADC: pinned local Keyboard archive absent')
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
    allowed = 'Original ADC setup, polling, dispatch and three native accumulators pass.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original ADC execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original ADC inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True,
        declared_component_cases=15, original_channel_paths=45, fresh_joint_replays=90,
        reference_suffixes=15, actual_native_instructions=8298,
        unknown_half_flag_cases=5, first_half_flag_definition_PC=0xe152, collector_controls=negative,
        unknown_input_cases=1, ideal_stable_inputs=True, CPU_instruction_timing_qualified=False,
        boot_origin_CCR_or_clock_supplied=False, physical_ADC_accuracy_qualified=False,
        native_DSP_firmware_executed=False, full_boot=False, physical_traffic=False,
        fuzz_execution=False, trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()}), indent=2)+'\n')
    print('Verified original ADC startup, busy/ready polling and native channel arithmetic.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
