# SPDX-License-Identifier: GPL-3.0-only
"""Execute original local HCS08 bytes; no hardware success or missing bytes are supplied."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from keyboard_update_reference import verified_blocks
from inspect_firmware import decode_block
from extract_runtime import hc12_object

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 execution: pinned local Keyboard archive absent')
        return 77
    blocks, _, _ = verified_blocks(archive)
    data = decode_block(blocks[2])[1]
    helper, consumed = hc12_object(data)
    reference = json.loads((ROOT / 'doc/hcs08-reference.json').read_text())
    programmer = helper[0x1546 - 0x1200:0x15ca - 0x1200]
    if len(programmer) != 132 or hashlib.sha256(programmer).hexdigest() != reference['programmer_sha256']:
        raise ValueError('Changed original HCS08 RAM programmer identity')
    stream = data[consumed:]
    if len(stream) != 3603 or stream[3598:] != bytes.fromhex('fffff7f7f7'):
        raise ValueError('Changed original supplied-page stream extent/end/trailer')
    for n, page in enumerate(reference['pages']):
        record = stream[n * 514:(n + 1) * 514]
        if (int.from_bytes(record[:2], 'big') != page['address'] or
                hashlib.sha256(record[2:]).hexdigest() != page['sha256']):
            raise ValueError('Changed original HCS08 page identity')
    trace.parent.mkdir(parents=True, exist_ok=True)
    code = trace.with_suffix('.programmer.bin')
    pages = trace.with_suffix('.pages.bin')
    code.write_bytes(programmer)
    pages.write_bytes(stream[:3598])
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, code, pages)}
    run = subprocess.run([str(p.resolve()) for p in (binary, code, pages, trace)],
                         capture_output=True, text=True, check=False, timeout=60)
    trace.with_suffix('.log').write_text(run.stdout + run.stderr)
    allowed = 'Original HCS08 programmer and startup execute to strict peripheral boundaries.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original HCS08 execution failed: ' + run.stdout + run.stderr)
    rows = []
    for line in trace.read_text().splitlines():
        tag, *items = line.split()
        rows.append((tag, {key: int(value) for key, value in (x.split('=') for x in items)}))
    expected = []
    for seed in (0, 0x1200):
        expected.extend([
            ('PROGRAMMER', dict(seed=seed, retired=3, pc=0xa5, sp=seed | 0x77,
                               hx=seed | 0x78, a=30, address=0x1820, fault=1)),
            ('STARTUP', dict(seed=seed, retired=6165, pc=0xe030, sp=0x47d, hx=300,
                            a=3, writes=3084, reads=2048, address=0x10, fault=1))])
    if rows != expected:
        raise ValueError('Original HCS08 native prefix state/counts differ')
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original HCS08 inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, cases=4,
        original_startup_instructions=12330, original_programmer_instructions=6,
        checked_ram_bytes=2048, writes=6168, reads=4096,
        strict_peripheral_failures=4, full_boot=False, controller_link=False,
        flash_programming=False, physical_traffic=False, fuzz_execution=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        specification_sha256=frozen[specification]), indent=2) + '\n')
    print('Verified original HCS08 programmer and startup prefixes with explicit unknown peripherals.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
