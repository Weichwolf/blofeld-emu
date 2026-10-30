# SPDX-License-Identifier: GPL-3.0-only
"""Run the real Board own-ROM flash controls and verify complete independent wire images."""
from pathlib import Path
import subprocess
import sys
import tempfile
import mcu_flash_live_wire as wire

with tempfile.TemporaryDirectory(prefix='blofeld-board-flash-') as name:
    directory = Path(name)
    result = subprocess.run([sys.argv[1], directory], capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    assert result.stderr.count('Own Board live-publication failure') == 2, result.stderr
    for filename, state in [('board-complete', 'program'), ('board-active', 'unknown'),
                            ('board-fault-transfer', 'unknown'), ('board-fault-complete', 'program')]:
        wire.verify(directory / (filename + '.bfnvm'), wire.transition(wire.initial(), state))
    print(result.stdout, end='')
    print('Four complete Board unit files match independent cells,known bits,history and CRC.')
