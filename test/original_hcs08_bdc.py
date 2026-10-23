# SPDX-License-Identifier: GPL-3.0-only
"""Original HCS08 execution reached through explicit byte-level BDC commands."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def validate(path, records):
    steps, pages, strict, retired = 0, 0, 0, 0
    ended = False
    with path.open() as stream:
        for line in stream:
            tag, *fields = line.split()
            row = {key: int(value) for key, value in (field.split('=') for field in fields)}
            if ended:
                raise ValueError('Data after original BDC completion')
            if tag == 'STRICT':
                if pages or steps or row != dict(replay=strict, pc=0xb2, retired=8, transfers=0):
                    raise ValueError('Changed native strict BDC frontier')
                strict += 1
            elif tag == 'STEP':
                steps += 1
                if strict != 2 or row['n'] != steps or row['mode'] not in (0, 1):
                    raise ValueError('Native BDC execution trace is not complete and sequential')
            elif tag == 'PAGE':
                address = int.from_bytes(records[pages * 514:pages * 514 + 2], 'big')
                if (pages >= 7 or steps <= retired or row != dict(index=pages, address=address,
                        retired=steps, serviced=165 + (pages + 1) * 520,
                        status=0xc8, trim=0xa5, transfers=(pages + 1) * 513)):
                    raise ValueError('Changed native page, BDC mode, command count or trim')
                retired = steps
                pages += 1
            elif tag == 'END':
                if pages != 7 or row != dict(pages=7, serviced=3805, retired=steps, trim=0xa5):
                    raise ValueError('Missing complete original BDC page profile')
                ended = True
            else:
                raise ValueError('Unknown original BDC trace row')
    if not ended:
        raise ValueError('Incomplete original BDC trace')
    return dict(original_instructions=steps, pages=pages, BDC_commands=3805,
                actual_erase_commands=7, actual_program_commands=3584,
                strict_frontier_replays=strict)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 BDC: pinned local Keyboard archive absent')
        return 77
    programmer, records = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    code, pages = trace.with_suffix('.programmer.bin'), trace.with_suffix('.pages.bin')
    code.write_bytes(programmer)
    pages.write_bytes(records)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, archive, specification, code, pages)}
    result = subprocess.run([str(p.resolve()) for p in (binary, code, pages, trace)],
                            capture_output=True, text=True, timeout=240, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    allowed = 'Original HCS08 execution through BDC transactions verified under explicit reference contracts.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original BDC execution failed: ' + result.stdout + result.stderr)
    totals = validate(trace, records)
    expected = bytearray([0xa5] * 8192)
    expected[0xffbd - 0xe000], expected[0xffbf - 0xe000] = 0xff, 0xfe
    for index in range(7):
        record = records[index * 514:(index + 1) * 514]
        address = int.from_bytes(record[:2], 'big') - 0xe000
        expected[address:address + 512] = record[2:]
    expected[0xffbe - 0xe000] = 0xa5
    if Path(str(trace) + '.bin').read_bytes() != expected:
        raise ValueError('BDC-launched native image differs from independent page overlay and retained trim')
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != sha for path, sha in frozen.items()):
        raise ValueError('Original BDC inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **totals,
        final_image_sha256=hashlib.sha256(expected).hexdigest(),
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        physical_wire=False, original_HC12_helper_executed=False,
        physical_timing_qualified=False, complete_update=False, full_boot=False,
        hardware_equivalence=False, physical_traffic=False, fuzz_execution=False,
        scope='Logical byte transactions, actual native HCS08 and explicit candidate/reference FLASH; '
        'own reset/debug register context and preimage. Saved trim is read and substituted through '
        'BDC test commands; actual HC12 bit transport, arbitration and complete update remain open.'), indent=2) + '\n')
    print('Verified native original HCS08 programmer through BDC commands, held modes and saved trim.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
