# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original GPIO startup with unknown or explicitly supplied strap inputs."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def row(text):
    tag, *items = text.split()
    fields = {}
    for item in items:
        key, value = item.split('=')
        if key in fields:
            raise ValueError('Duplicate GPIO trace field')
        fields[key] = int(value)
    return tag, fields


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 GPIO: pinned local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, pages)}
    run = subprocess.run([str(p.resolve()) for p in (binary, pages, trace)],
                         capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(run.stdout + run.stderr)
    allowed = 'Original HCS08 GPIO startup preserves unknown pads and actual strap branches.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original GPIO execution failed: ' + run.stdout + run.stderr)
    lines = trace.read_text().splitlines()
    observations = [-1] + [bits << 2 for bits in range(16)]
    if len(lines) != 40:
        raise ValueError('Incomplete/extra original GPIO case chronology')
    at = 0
    for observation in observations:
        if row(lines[at]) != ('CONFIG', dict(observation=observation, retired=6176,
                                             pc=0xe04e, writes=11)):
            raise ValueError('Changed original GPIO configuration boundary')
        end = lines[at+1]
        at += 2
        if observation < 0:
            if not (end.startswith('UNKNOWN HCS08 execution pc=0xe04e: HCS08 GPIO read ') and
                    'address=0x0010 pc=0xe04e ccr=0x006c' in end and
                    'Detached resolved GPIO pad source mask=0x00003c' in end):
                raise ValueError('Unknown straps did not retain the genuine pad-read failure')
        elif observation & 0x38:
            if row(end) != ('CLOCK', dict(observation=observation, retired=6185,
                                         pc=0xe1a8, address=0x4e)):
                raise ValueError('Nonzero straps did not reach original clock trim')
        elif row(end) != ('BACKGROUND', dict(observation=observation, retired=6181)):
            raise ValueError('Zero strap mask did not take the real background branch')
        if observation in (-1, 0, 8):
            for replay in range(2):
                if row(lines[at]) != ('REPLAY', dict(observation=observation, index=replay, identical=1)):
                    raise ValueError('Missing/changed fresh GPIO joint replay')
                at += 1
    if at != len(lines):
        raise ValueError('Extra GPIO trace suffix')
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original GPIO input changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, cases=17,
        fresh_joint_replays=6, unknown_pad_cases=1, zero_strap_BGND_cases=2, original_clock_trim_failures=14,
        original_GPIO_writes=187, known_strap_patterns=16, initial_RAM_unknown=True,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, physical_straps_inferred=False, target_part_qualified=False,
        clock_timing_qualified=False, physical_traffic=False, fuzz_execution=False), indent=2)+'\n')
    print('Verified original GPIO writes and every explicit Port E input pattern.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
