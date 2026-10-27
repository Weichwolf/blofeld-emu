# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original POR through the candidate production memory owner."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def validate(text):
    expected = [
        'MISSING trim pc=57765 retired=6184',
        'REFERENCE pc=57802 retired=6197',
        'PARTIAL pc=57802 retired=6197',
        'UNKNOWN guard pc=57426 retired=6178',
        'Verified original POR through independent production board memory.',
    ]
    if text.splitlines() != expected:
        raise ValueError('Changed original reset,missing calibration or strict live-timer frontier')


def controls(text):
    variants = [text + 'EXTRA\n', '\n'.join(text.splitlines()[1:]) + '\n']
    for before, after in [('57765', '57764'), ('6184', '6185'),
                          ('57802', '57803'), ('6197', '6196'),
                          ('57426', '57425'), ('6178', '6179'), ('PARTIAL', 'REFERENCE'),
                          ('UNKNOWN', 'ACCEPTED')]:
        variants.append(text.replace(before, after, 1))
    for changed in variants:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed wrong-frontier control')
    return len(variants)


def main():
    if len(sys.argv) != 5:
        raise ValueError('Expected binary,original archive,specification and private trace')
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original board memory:pinned local Keyboard archive absent')
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
        raise ValueError('Original board-memory reset failed:' + result.stdout + result.stderr)
    validate(result.stdout)
    rejected = controls(result.stdout)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in pins.items()):
        raise ValueError('Original board-memory input changed during execution')
    trace.write_text(result.stdout)
    report = dict(passed=True, original_normal_resets=5, missing_pad_control=True,
                  missing_factory_calibration_control=True, declared_port_e=8, declared_trim=128,
                  partial_gpio_known_mask=8, deciding_unknown_guard=True,
                  full_native_ram_peripheral_endpoint_match=True,
                  reference_retired=6197, endpoint_pc=0xe1ca, untimed=True,
                  collector_controls=rejected, original_code_modified=False, full_boot=False,
                  hardware_equivalence=False, fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(path): digest for path, digest in pins.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified original POR with strict board memory,pad/calibration controls and live-timer gate.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
