# SPDX-License-Identifier: GPL-3.0-only
"""Run one explicit startup observation with a nine-second external deadline."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    if len(sys.argv) != 7:
        raise ValueError(
            'Usage: startup_probe.py binary firmware.zip board.bfboard '
            'boot.bin unit.bfnvm new-output-dir')
    inputs = [Path(value).resolve(strict=True) for value in sys.argv[1:6]]
    if not all(path.is_file() for path in inputs):
        raise ValueError('Every startup input must be a regular file')
    directory = Path(sys.argv[6]).resolve()
    directory.mkdir(mode=0o700, parents=True, exist_ok=False)
    identities = {}
    for path in inputs:
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        identities[str(path)] = {'bytes': path.stat().st_size, 'sha256': digest}
    (directory / 'inputs.json').write_text(json.dumps(identities, indent=2) + '\n')
    command = [str(path) for path in inputs] + [str(directory)]
    started = time.monotonic()
    timed_out = False
    with (directory / 'output.log').open('wb') as output:
        child = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        try:
            code = child.wait(timeout=9)
        except subprocess.TimeoutExpired:
            timed_out = True
            child.kill()
            code = child.wait()
        finally:
            if child.poll() is None:
                child.kill()
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
