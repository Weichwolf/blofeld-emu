# SPDX-License-Identifier: GPL-3.0-only
"""Run one explicit startup observation with a 32-second external deadline."""
import hashlib
import json
import shlex
import os
import signal
from pathlib import Path
import subprocess
import sys
import time


def main():
    profiling = len(sys.argv) > 1 and sys.argv[1] == '--perf'
    if profiling:
        del sys.argv[1]
    if len(sys.argv) != 4:
        raise ValueError(
            'Usage: startup_probe.py [--perf] binary profile new-output-dir')
    inputs = [Path(value).resolve(strict=True) for value in sys.argv[1:3]]
    if not all(path.is_file() for path in inputs):
        raise ValueError('Every startup input must be a regular file')
    directory = Path(sys.argv[3]).resolve()
    directory.mkdir(mode=0o700, parents=True, exist_ok=False)
    assets = []
    profile = inputs[1]
    for line in profile.read_text().splitlines():
        fields = shlex.split(line, comments=True)
        if fields and fields[0] in {'firmware', 'boot-rom', 'device-state', 'nand'}:
            path = Path(fields[1])
            assets.append((path if path.is_absolute() else profile.parent / path).resolve(strict=True))
    identities = {}
    for path in inputs + assets:
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        identities[str(path)] = {'bytes': path.stat().st_size, 'sha256': digest}
    (directory / 'inputs.json').write_text(json.dumps(identities, indent=2) + '\n')
    command = [str(path) for path in inputs] + [str(directory)]
    if profiling:
        command = ['perf', 'record', '--user-callchains', '-e', 'cycles:u',
                   '-F', '499', '--call-graph', 'dwarf,8192',
                   '-o', str(directory / 'perf.data'), '--'] + command
    started = time.monotonic()
    timed_out = False
    with (directory / 'output.log').open('wb') as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            code = child.wait(timeout=32)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(child.pid, signal.SIGKILL)
            code = child.wait()
        finally:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
    elapsed = time.monotonic() - started
    result = {'command': command, 'wall_seconds': elapsed, 'exit_code': code,
              'external_timeout': timed_out, 'playability': 'unproven',
              'startup_probe': 'failed'}
    (directory / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print((directory / 'output.log').read_text(), end='')
    print(json.dumps(result, indent=2))
    if timed_out:
        return 124
    return code or 2


if __name__ == '__main__':
    raise SystemExit(main())
