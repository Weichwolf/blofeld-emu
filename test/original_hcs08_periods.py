# SPDX-License-Identifier: GPL-3.0-only
"""Original controller startup on independently documented bus-period budgets."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row


def validate(text):
    lines = iter(text.splitlines())
    if row(next(lines)) != ('TIMED', dict(gpio=22630, trim=22662, controls=22670)):
        raise ValueError('Changed independent original startup period totals')
    error = next(lines)
    if (not error.startswith('ERROR HCS08 execution pc=0xe1b0: ') or
            'ICG read address=0x004a pc=0xe1b0 ccr=0x0069' not in error or
            'live clock/status evolution is unqualified' not in error):
        raise ValueError('Missing actual timed ICG status failure')
    if row(next(lines)) != ('FAULT', dict(periods=22673, offered=22673,
                                         retired=6188, pc=0xe1b0, remaining=0)):
        raise ValueError('Failed status read changed its elapsed budget/retirement boundary')
    for cut, periods in enumerate((1, 22664, 22671)):
        for replay in range(2):
            if row(next(lines)) != ('REPLAY', dict(cut=cut, index=replay,
                                                  periods=periods, identical=1)):
                raise ValueError('Missing partial-instruction joint continuation')
    if next(lines, None) is not None:
        raise ValueError('Extra original timed suffix')


def controls(text):
    changes = [text.replace('gpio=22630', 'gpio=22629', 1),
               text.replace('trim=22662', 'trim=22661', 1),
               text.replace('controls=22670', 'controls=22671', 1),
               text.replace('address=0x004a', 'address=0x004b', 1),
               text.replace('retired=6188', 'retired=6189', 1),
               text.replace('periods=22673', 'periods=22672', 1),
               text.replace('identical=1', 'identical=0', 1),
               '\n'.join(text.splitlines()[:-1])+'\n', text+'EXTRA value=1\n']
    for changed in changes:
        try:
            validate(changed)
        except (ValueError, StopIteration):
            continue
        raise ValueError('Directed trace corruption was accepted')
    return len(changes)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 periods: pinned local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, pages)}
    run = subprocess.run([str(p.resolve()) for p in (binary, pages, trace)]+['periods'],
                         capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original HCS08 FLASH/GPIO/ICG setup retains unknown status and actual trim history.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original bus-period execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    rejected = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original timed inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True,
        periods=22673, retired=6188, fresh_joint_replays=6, collector_controls=rejected,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        physical_frequency_supplied=False, ICG_status_supplied=False,
        full_boot=False, physical_traffic=False, fuzz_execution=False), indent=2)+'\n')
    print('Verified original controller bus-period budgets and strict ICG status frontier.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
