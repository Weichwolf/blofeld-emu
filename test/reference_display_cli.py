# SPDX-License-Identifier: GPL-3.0-only
"""Ordinary CLI publication from authored native LCD writes, without guest changes."""
import binascii
import fcntl
import hashlib
import json
from pathlib import Path
import stat
import struct
import subprocess
import sys

from test_reference_cli import image

BINARY, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:4])
ROOT = Path(__file__).resolve().parents[1]
HEADER = b'P4\n128 64\n'
PATTERN = 0xa5
# MC9S12UF32 MEBI register addresses; E4 is the declared LCD enable signal.
PORT_A, PORT_B, DDR_A, DDR_B, PORT_E, DDR_E = 0, 1, 2, 3, 8, 9
CONFIG = '''blofeld-reference-board 2
dsp-execution reference-effects
clock 12000000 0 12000000 540000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 18446744073709551615 low
operations 9000
port-b-pulls 82 7d
lcd-parallel st7565 reset-release 3000
initial drive E 03 03 00
'''


def store(address, value):
    # Authored HC12 LDAA immediate / STAA extended, not manufacturer bytes.
    return bytes((0x86, value, 0x7a, address >> 8, address & 255))


def lcd(value, data=False):
    return (store(PORT_A, value) + store(PORT_B, 0x41 if data else 0x40) +
            store(PORT_E, 16) + store(PORT_E, 0))


def program(complete):
    code = store(PORT_E, 0) + store(DDR_E, 16) + store(PORT_B, 0x80) + store(DDR_B, 0xe1)
    code += store(PORT_A, 0) + store(DDR_A, 255) + bytes((0xa7,)) * 32
    # Common reversal cancels the explicit viewer panel's reversed COM wiring.
    code += lcd(0xc8)
    if complete:
        for page in range(8):
            code += lcd(0xb0 + page) + lcd(0) + lcd(0x10)
            code += lcd(0, True) * 128
    code += lcd(0xb0) + lcd(0) + lcd(0x10) + lcd(PATTERN, True) + lcd(0xaf)
    # Authored BRA self-loop keeps the completed display stable for the remaining budget.
    return code + bytes.fromhex('20fe')


def mcu_image(code):
    result = bytearray(image(code))
    # The unrolled display program crosses the helper's optional IRQ handler.
    # IRQs stay masked; preserve every authored program byte at the real reset entry.
    start = 16 + 0x1000
    assert len(code) < 0x6f00
    result[start:start + len(code)] = code
    result[-4:] = struct.pack('<I', binascii.crc32(result[:-4]))
    return result


def prepare(name, complete=True, config=CONFIG, code=None):
    root = ARTIFACTS / name
    root.mkdir(parents=True, exist_ok=True)
    board, rom, nvm = root / 'own.bfboard', root / 'rom.bin', root / 'mcu.bfnvm'
    board.write_text(config)
    rom.write_bytes(bytes(3))  # Authored DSP NOP; this fixture genuinely holds DSP reset.
    nvm.write_bytes(mcu_image(program(complete) if code is None else code))
    display = root / 'display'
    display.mkdir(exist_ok=True)
    for name in ('display.pbm', 'display.txt'):
        (display / name).unlink(missing_ok=True)
    command = [str(BINARY), '--firmware', str(FIRMWARE), '--reference-board', str(board),
               '--dsp-rom', str(rom), '--flash-in', str(nvm), '--instructions', '9000']
    return root, display, command


def invoke(command, log):
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=30, check=False)
    log.write_text(result.stdout)
    return result


def complete_frame():
    root, display, command = prepare('complete')
    baseline = invoke([*command, '--trace', str(root / 'plain.trace')], root / 'plain.log')
    observed = invoke([*command, '--trace', str(root / 'display.trace'), '--display-dir', str(display)],
                      root / 'display.log')
    assert baseline.returncode == observed.returncode == 0, baseline.stdout + observed.stdout
    assert baseline.stdout == observed.stdout
    assert (root / 'plain.trace').read_bytes() == (root / 'display.trace').read_bytes()
    expected = bytearray(1024)
    for y in (0, 2, 5, 7):
        expected[y * 16] = 0x80
    bitmap = display / 'display.pbm'
    assert bitmap.read_bytes() == HEADER + expected
    text = (display / 'display.txt').read_text().splitlines()
    assert len(text) == 64 and all(len(row) == 128 for row in text)
    assert [(x, y) for y, row in enumerate(text) for x, pixel in enumerate(row) if pixel == '#'] == [
        (0, 0), (0, 2), (0, 5), (0, 7)]
    assert stat.S_IMODE(bitmap.stat().st_mode) == 0o600
    assert 'value=0xaf known=0xff' in (root / 'display.trace').read_text()


def coexistence():
    root, display, command = prepare('midi-display', config=CONFIG +
                                     'midi-sci PR2 6000000 after 0\n')
    midi = root / 'clock.bin'
    midi.write_bytes(bytes((0xf8,)))
    result = invoke([*command, '--midi-file', str(midi), '--display-dir', str(display)],
                    root / 'run.log')
    assert result.returncode == 0 and 'MIDI accepted=1 ' in result.stdout, result.stdout
    expected = bytearray(1024)
    for y in (0, 2, 5, 7):
        expected[y * 16] = 0x80
    assert (display / 'display.pbm').read_bytes() == HEADER + expected


def unknown_and_faulted():
    root, display, command = prepare('unknown', complete=False)
    result = invoke([*command, '--trace', str(root / 'trace'), '--display-dir', str(display)],
                    root / 'run.log')
    assert result.returncode == 0, result.stdout
    # Only the earlier known reset/off image is published; unknown RAM is never filled with zero.
    assert (display / 'display.pbm').read_bytes() == HEADER + bytes(1024)
    assert 'data=1 value=0xa5 known=0xff' in (root / 'trace').read_text()
    root, display, command = prepare('native-fault', code=bytes.fromhex('b63000'))
    result = invoke([*command, '--display-dir', str(display)], root / 'run.log')
    assert result.returncode == 2 and 'pc=' in result.stdout, result.stdout


def setup_guards():
    root, display, command = prepare('missing-route', config=CONFIG.replace(
        'lcd-parallel st7565 reset-release 3000\n', ''))
    result = invoke([*command, '--display-dir', str(display)], root / 'run.log')
    assert result.returncode == 1 and 'requires lcd-parallel' in result.stdout
    assert not (display / 'display.pbm').exists()
    root, display, command = prepare('writer-lease')
    with (display / '.display.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = invoke([*command, '--display-dir', str(display)], root / 'run.log')
        assert result.returncode == 1 and 'lease' in result.stdout, result.stdout
    assert not (display / 'display.pbm').exists()


def aliases():
    for kind in ('input', 'hardlink', 'trace', 'nand', 'nand-store'):
        root, display, command = prepare('alias-' + kind)
        if kind == 'trace':
            command += ['--trace', str(display / 'display.txt')]
        elif kind in ('nand', 'nand-store'):
            body = b'BLFNAND\n' + struct.pack('<IIIIII', 1, 1, 0, 0, 528, 0)
            nand = root / 'input.bfnand'
            nand.write_bytes(body + struct.pack('<I', binascii.crc32(body)))
            (display / 'display.pbm').hardlink_to(nand)
            with (root / 'own.bfboard').open('a') as source:
                source.write(f'{"nand-image" if kind == "nand" else "nand-store"} j "{nand}"\n')
        else:
            original = root / 'rom.bin'
            if kind == 'input':
                original.rename(display / 'display.pbm')
                command[command.index('--dsp-rom') + 1] = str(display / 'display.pbm')
            else:
                (display / 'display.txt').hardlink_to(original)
        retained = {p: p.read_bytes() for p in display.iterdir() if p.is_file()}
        result = invoke([*command, '--display-dir', str(display)], root / 'run.log')
        assert result.returncode == 1 and 'collision' in result.stdout, result.stdout
        assert all(p.read_bytes() == data for p, data in retained.items())


if not FIRMWARE.is_file():
    print('SKIP: local original archive absent; native Board LCD tests remain unconditional')
    raise SystemExit(77)
manifest = json.loads((ROOT / 'doc/downloads.json').read_text())
expected_hash = next(row['sha256'] for row in manifest['files'] if row['file'] == FIRMWARE.name)
assert hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() == expected_hash
ARTIFACTS.mkdir(parents=True, exist_ok=True)
complete_frame()
coexistence()
unknown_and_faulted()
setup_guards()
aliases()
print('CLI LCD: literal native pixels, identical CPU/DSP trace, unknown/native faults and output guards passed')
