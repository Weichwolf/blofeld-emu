# SPDX-License-Identifier: GPL-3.0-only
"""Original scanner stores with independent periods, fresh restores and strict GPIO failures."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
import original_hcs08_spi_word
import original_hcs08_spi_move

PROFILES = {'word': (original_hcs08_spi_word, 300), 'move': (original_hcs08_spi_move, 600)}


def validate(text, expected):
    if not text.endswith('\n'):
        raise ValueError('Truncated original scanner store trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original scanner store field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected:
        raise ValueError('Changed original scanner store periods, state or checkpoint matrix')


def controls(text, expected):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    seen = set()
    for index, (tag, fields) in enumerate(expected):
        if tag in seen:
            continue
        seen.add(tag)
        for field, value in fields.items():
            columns = lines[index].split()
            at = next(i for i, item in enumerate(columns) if item.startswith(field+'='))
            columns[at] = field+'='+str(value+1)
            invalid.append(''.join(lines[:index])+' '.join(columns)+'\n'+''.join(lines[index+1:]))
    for candidate in invalid:
        try:
            validate(candidate, expected)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed scanner store corruption')
    return len(invalid)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:5])
    profile = sys.argv[5]
    module, timeout = PROFILES[profile]
    if not archive.is_file():
        print('SKIP original scanner stores: pinned local Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    expected = module.expected_rows(pages)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: digest(p) for p in (binary, archive, specification, supplied,
                                    Path(__file__), Path(module.__file__))}
    command = [str(p.resolve()) for p in (binary, supplied, trace)]
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=timeout)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = ('Original CPU/SPI/ADC periods preserve genuine serial prefixes and the strict '
               'foreground frontier.' if profile == 'word' else
               'Original native MOV joins real SPI/ADC completion under declared pad observations; '
               'missing and detached pads still fail. Full boot and physical matrix remain '
               'unqualified.')
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original scanner store execution failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, expected)
    negative = controls(text, expected)
    if any(digest(p) != value for p, value in frozen.items()):
        raise ValueError('Original scanner store inputs changed')
    summary = expected[-1][1]
    report = dict(passed=True, profile=profile, **summary, collector_controls=negative,
                  input_sha256={str(p): value for p, value in frozen.items()},
                  trace_sha256=digest(trace), original_instruction_bytes_modified=False,
                  host_instruction_or_PC_substitution=False, full_controller_boot=False,
                  physical_matrix_or_clock_qualified=False, host_MIDI_audio=False,
                  hardware_equivalence=False, fuzz_execution=False, physical_traffic=False)
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f'Verified original {profile} stores and fresh CPU/SPI/ADC checkpoint suffixes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
