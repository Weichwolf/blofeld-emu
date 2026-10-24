# SPDX-License-Identifier: GPL-3.0-only
"""Unredirected cold MCU startup consumes persistent serial and draws its own warnings."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

binary, archive, specification, trace = map(Path, sys.argv[1:])
if not archive.is_file():
    print('SKIP original startup identity: pinned local firmware absent')
    raise SystemExit(77)
assert hashlib.sha256(archive.read_bytes()).hexdigest() == (
    '095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72')
frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, archive, specification)}
trace.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='blofeld-startup-identity-') as temporary:
    run = subprocess.run([str(binary), str(archive), str(trace),
                          str(Path(temporary) / 'unit.bfnvm')],
                         capture_output=True, text=True, timeout=170, check=False)
expected = ['Cold native startup unavailable=0: DSP warning retained, serial warning=0, NVM preserved.',
            'Cold native startup unavailable=0: DSP warning retained, serial warning=0, NVM preserved.',
            'Cold native startup unavailable=1: DSP warning retained, serial warning=1, NVM preserved.',
            'Conditional original reset/diagnostic rendering passed; full DSP boot, normal runtime '
            'and physical board equivalence remain unqualified.']
if run.returncode or run.stderr or run.stdout.splitlines() != expected:
    raise ValueError('Original cold startup identity check failed: ' + run.stdout + run.stderr)
assert all(hashlib.sha256(p.read_bytes()).hexdigest() == sha for p, sha in frozen.items())
panels = [Path(str(trace) + f'.{n}.pbm').read_bytes() for n in range(3)]
assert all(panel.startswith(b'P4\n128 64\n') for panel in panels)
assert panels[0] == panels[1] and panels[2] != panels[0]
print('Three original cold startups retain persistent/damaged identity and genuine diagnostic rendering.')
