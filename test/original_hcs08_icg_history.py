# SPDX-License-Identifier: GPL-3.0-only
"""Original untimed reset with conservative ICG bits and a strict live-timer gate."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def validate(text):
    expected = ('Original reset retains unknown ICG status through native ADC setup; '
                'strict TPM frontier.')
    if text.splitlines() != [expected]:
        raise ValueError('Changed original conservative-status/timer frontier')


def main():
    if len(sys.argv) != 5:
        raise ValueError('Expected binary, original archive, specification and private trace')
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original ICG history: pinned local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    pins = {path: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (binary, archive, specification, pages)}
    result = subprocess.run([str(binary.resolve()), str(pages.resolve())], capture_output=True,
                            text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr:
        raise ValueError('Original ICG history failed: ' + result.stdout + result.stderr)
    validate(result.stdout)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in pins.items()):
        raise ValueError('Original ICG-history input changed during execution')
    trace.write_text(result.stdout)
    report = dict(passed=True, retired=6197, endpoint_pc=0xe1ca, untimed=True,
                  full_boot=False, hardware_equivalence=False, original_code_modified=False,
                  ICG_ready_or_rate_supplied=False, fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(path): digest for path, digest in pins.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified conservative original ICG status and strict live timer frontier.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
