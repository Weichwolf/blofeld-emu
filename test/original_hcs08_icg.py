# SPDX-License-Identifier: GPL-3.0-only
"""Execute original normal startup through candidate FLASH/GPIO/ICG owners."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row


def validate(text):
    lines = iter(text.splitlines())
    if row(next(lines)) != ('FETCH_CONTROLS', dict(count=3, retired=0)):
        raise ValueError('Missing strict original fetch controls')
    for seed, observation in ((-1, -1), (-1, 8), (0x5a, 8)):
        if row(next(lines)) != ('CASE', dict(seed=seed, observation=observation)):
            raise ValueError('Changed original ICG input chronology')
        if observation >= 0:
            if row(next(lines)) != ('RESET', dict(trim=0x80, pc=0xe000, retired=6188)):
                raise ValueError('Missing/changed actual coordinated warm reset')
        error = next(lines)
        expected = ('GPIO read address=0x0010 pc=0xe04e ccr=0x006c' if observation < 0 else
                    'ICG read address=0x004a pc=0xe1b0 ccr=0x0069')
        reason = 'Detached resolved GPIO' if observation < 0 else 'live clock/status evolution is unqualified'
        if not error.startswith('ERROR HCS08 execution ') or expected not in error or reason not in error:
            raise ValueError('Actual input/status read did not retain its strict failure')
        if row(next(lines)) != ('FAULT', dict(retired=6176 if observation < 0 else 6188,
                                            pc=0xe04e if observation < 0 else 0xe1b0)):
            raise ValueError('Changed original failure endpoint')
        for cut in range(1 if observation < 0 else 3):
            for replay in range(2):
                if row(next(lines)) != ('REPLAY', dict(cut=cut, index=replay, identical=1)):
                    raise ValueError('Incomplete/changed original joint restoration')
    if next(lines, None) is not None:
        raise ValueError('Extra original ICG suffix')


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 ICG: pinned local Keyboard archive absent')
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
    allowed = 'Original HCS08 FLASH/GPIO/ICG setup retains unknown status and actual trim history.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original ICG execution failed: ' + run.stdout + run.stderr)
    validate(trace.read_text())
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original ICG inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, cases=3,
        unknown_pad_failures=1, original_status_failures=2, fresh_joint_replays=14,
        coordinated_warm_reset_checks=2, unsupported_fetch_controls=3, initial_RAM_unknown=True, live_trim_guessed=False,
        NVM_unchanged=True, NVM_operating_rate_supplied=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, clock_status_qualified=False, physical_timing_qualified=False,
        physical_traffic=False, fuzz_execution=False), indent=2)+'\n')
    print('Verified original FLASH/GPIO/ICG startup and strict clock-status boundary.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
