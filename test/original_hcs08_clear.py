# SPDX-License-Identifier: GPL-3.0-only
"""Check original receive/return periods against fixed ISA/source expectations."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_spi_stores import validate, controls
from original_clear import expected_rows


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original CLR cycles: pinned local Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    expected = expected_rows(pages)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: digest(p) for p in (binary, archive, specification, supplied, Path(__file__),
                                    Path(__file__).with_name('original_clear.py'))}
    result = subprocess.run([str(p.resolve()) for p in (binary, supplied, trace)],
                            capture_output=True, text=True, check=False, timeout=270)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    allowed = ('Original native CLR:16 contexts,24 cuts,48 fresh suffixes. '
               'Declared cycle/clock references; full boot and physical phase remain open.')
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original CLR cycle execution failed: ' + result.stdout + result.stderr)
    text = trace.read_text()
    validate(text, expected)
    negative = controls(text, expected)
    if any(digest(p) != value for p, value in frozen.items()):
        raise ValueError('Original CLR cycle inputs changed')
    report = dict(passed=True, **expected[-1][1], collector_fixed_negative_controls=negative,
                  input_sha256={str(p): value for p, value in frozen.items()},
                  trace_sha256=digest(trace), original_instruction_bytes_modified=False,
                  host_instruction_or_PC_substitution=False, full_controller_boot=False,
                  physical_matrix_or_clock_qualified=False, host_MIDI_audio=False,
                  hardware_equivalence=False, fuzz_execution=False, physical_traffic=False)
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified original CLR read/write cycles and fresh checkpoint suffixes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
