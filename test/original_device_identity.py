# SPDX-License-Identifier: GPL-3.0-only
"""Original boot checksum and resident serial check; no successful DSP status is injected."""
import hashlib
from pathlib import Path
import subprocess
import sys

binary, archive, specification = map(Path, sys.argv[1:])
if not archive.is_file():
    print('SKIP original unit identity: pinned local firmware absent')
    raise SystemExit(77)
expected_archive = '095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == expected_archive
frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in [binary, archive, specification]}
run = subprocess.run([str(binary), str(archive)], capture_output=True, text=True, timeout=60, check=False)
expected = ['BOOT provisioned=0 instructions=90314 pc=33280 sp=9728 d=0',
            'SERIAL provisioned=0 first=255 status=128 pc=34041 ccr=216 dsp=165',
            'BOOT provisioned=1 instructions=90314 pc=33280 sp=9728 d=0',
            'SERIAL provisioned=1 first=69 status=0 pc=34041 ccr=212 dsp=165',
            'Original boot and serial warning use persistent factory data; full DSP boot remains open.']
if run.returncode or run.stderr or run.stdout.splitlines() != expected:
    raise ValueError('Original device identity check failed: ' + run.stdout + run.stderr)
assert all(hashlib.sha256(p.read_bytes()).hexdigest() == sha for p, sha in frozen.items())
print('Verified unchanged boot/program bytes and native serial-warning decisions for erased/virtual unit data.')
