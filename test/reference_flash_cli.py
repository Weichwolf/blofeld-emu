# SPDX-License-Identifier: GPL-3.0-only
"""Real CLI device-state binding with own ROM and independent complete NVM images."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import mcu_flash_live_wire as wire

binary, firmware = map(Path, sys.argv[1:3])
if not firmware.is_file():
    print('SKIP CLI container: local original archive absent; own Board tests remain unconditional')
    raise SystemExit(77)
manifest = json.loads((Path(__file__).resolve().parents[1] / 'doc/downloads.json').read_text())
expected = next(row['sha256'] for row in manifest['files'] if row['file'] == firmware.name)
assert hashlib.sha256(firmware.read_bytes()).hexdigest() == expected
config = '''blofeld-reference-board 1
mcu-flash-timing uf32-formula 1 zero-gap words
clock 12000000 0 12000000 540000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 18446744073709551615 low
operations 1000
initial drive B 16 02 00
initial drive E 03 03 00
'''
with tempfile.TemporaryDirectory(prefix='blofeld-live-flash-cli-') as name:
    directory = Path(name)
    board, rom = directory / 'own.bfboard', directory / 'own-rom.bin'
    board.write_text(config)
    rom.write_bytes(bytes(3))  # Own DSP NOP; DSP is never released in this MCU control.
    for budget, state in [(1, 'initial'), (100, 'unknown'), (200, 'program')]:
        path = directory / f'unit-{budget}.bfnvm'
        path.write_bytes(wire.encoded(wire.initial()))
        path.chmod(0o600)
        result = subprocess.run([binary, '--firmware', firmware, '--reference-board', board,
                                 '--dsp-rom', rom, '--device-state', path,
                                 '--instructions', str(budget)], capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        assert f'instructions={budget} ' in result.stdout and 'boot completion is not established' in result.stdout
        wire.verify(path, wire.transition(wire.initial(), state))
print('CLI:own native reset/guest code publishes initial,active and complete unit files without ZIP fallback.')
