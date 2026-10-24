# SPDX-License-Identifier: GPL-3.0-only
"""Reference startup preserves the legacy baseline and consumes real persistent unit state."""
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

binary, archive, specification, trace = map(Path, sys.argv[1:5])
mode = sys.argv[5]
assert mode in ('baseline', 'factory')
if not archive.is_file():
    print('SKIP original identity main: pinned local firmware absent')
    raise SystemExit(77)
assert hashlib.sha256(archive.read_bytes()).hexdigest() == (
    '095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72')
frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, archive, specification)}
trace.parent.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='blofeld-identity-main-') as temporary:
    run = subprocess.run([str(binary), str(archive), str(trace), temporary, mode],
                         capture_output=True, text=True, timeout=230, check=False)
factory = int(mode == 'factory')
expected = [f'Original identity main factory={factory}: native WAI, strict input recovery, '
            'LCD publication and retained NVM passed.']
if factory:
    expected.append('Original SCI:264 fixed waveform bytes use actual WAI/ISR/foreground paths.')
expected.append('Conditional MCU/LCD/SCI experiment only; complete DSP/profile/audio/update/'
                'recovery and physical hardware equivalence remain unqualified.')
if run.returncode or run.stderr or run.stdout.splitlines() != expected:
    raise ValueError('Original identity main failed: ' + run.stdout + run.stderr)
assert all(hashlib.sha256(p.read_bytes()).hexdigest() == sha for p, sha in frozen.items())
assert Path(str(trace) + '.pbm').read_bytes().startswith(b'P4\n128 64\n')
print(f'Original {mode} main startup and native persistence/publication contract verified.')
