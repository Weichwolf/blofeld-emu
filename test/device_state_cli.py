# SPDX-License-Identifier: GPL-3.0-only
"""Persistent virtual units through actual MCU/reference-board process paths."""
import hashlib
from pathlib import Path
import stat
import sys
import tempfile
import unittest

from flash_cli import BINARY, FIRMWARE, encoded, invoke
from test_reference_cli import CONFIG, ROM, WAIT, image


class Options(unittest.TestCase):
    def test_conflicts_and_aliases_fail_before_loading(self):
        for arguments in (("--inspect",), ("--flash-in", "x.bfnvm"),
                          ("--flash-out", "x.bfnvm")):
            result = invoke("absent.zip", "--device-state", "unit.bfnvm", *arguments)
            self.assertEqual(result.returncode, 1)
            self.assertIn(b"conflicts", result.stderr)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "input.bfnvm"
            path.write_bytes(b"own protected input")
            link = path.parent / "alias.bfnvm"
            link.hardlink_to(path)
            for arguments in (("--trace", link), ("--audio-out", link)):
                result = invoke(path, "--device-state", path, *arguments)
                self.assertEqual(result.returncode, 1)
                self.assertEqual(path.read_bytes(), b"own protected input")


class Execution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not FIRMWARE.is_file():
            raise unittest.SkipTest("Pinned manufacturer firmware remains a private input")
        assert hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() == (
            '095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72')
        cls.binary_hash = hashlib.sha256(BINARY.read_bytes()).hexdigest()

    @classmethod
    def tearDownClass(cls):
        assert hashlib.sha256(BINARY.read_bytes()).hexdigest() == cls.binary_hash

    def test_new_unit_identity_survives_process_restart(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "unit.bfnvm"
            for restart in range(2):
                result = invoke(FIRMWARE, "--device-state", path, "--instructions", 0)
                self.assertEqual(result.returncode, 0, result.stderr.decode())
                current = path.read_bytes()
                serial = current[16 + 0x1e0:16 + 0x200].split(b"\0", 1)[0]
                self.assertRegex(serial, rb"^EMU[0-9A-HJKMNP-TV-Z]{9}$")
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
                if restart:
                    self.assertEqual(current, first)
                else:
                    first = current

    def test_existing_and_failed_nvm_are_never_repaired(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "unit.bfnvm"
            # Own NOP program, independent reset vector and unknown factory byte.
            original = encoded(unknown=(0x1e0,))
            path.write_bytes(original)
            result = invoke(FIRMWARE, "--device-state", path, "--instructions", 1)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(b"pc=0x9001", result.stdout)
            self.assertEqual(path.read_bytes(), original)
            unknown_instruction = encoded(unknown=(0x1000,))
            path.write_bytes(unknown_instruction)
            result = invoke(FIRMWARE, "--device-state", path, "--instructions", 1)
            self.assertEqual(result.returncode, 2, result.stderr.decode())
            self.assertIn(b"0x9000", result.stderr)
            self.assertEqual(path.read_bytes(), unknown_instruction)
            path.write_bytes(b"own corrupt state")
            result = invoke(FIRMWARE, "--device-state", path, "--instructions", 0)
            self.assertEqual(result.returncode, 1)
            self.assertIn(b"Invalid MCU NVM file length", result.stderr)
            self.assertEqual(path.read_bytes(), b"own corrupt state")

    def test_native_coupled_board_persists_success_and_strict_fault(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            board, rom, state = root / "board.bfboard", root / "rom.bin", root / "unit.bfnvm"
            rom.write_bytes(ROM)
            # Independent own native WAI/DSP loop, followed by unknown GPIO refusal.
            cases = ((CONFIG, WAIT, 7, 0),
                     (CONFIG + "loopback U 01 P 01\ninitial drive P fe 00 00\n",
                      bytes.fromhex("8601 7a025c b60258"), 3, 2))
            for config, code, limit, expected in cases:
                board.write_text(config)
                original = image(code)
                state.write_bytes(original)
                result = invoke(FIRMWARE, "--device-state", state,
                                "--reference-board", board, "--dsp-rom", rom,
                                "--instructions", limit)
                self.assertEqual(result.returncode, expected, result.stderr.decode())
                self.assertEqual(state.read_bytes(), original)
                if expected:
                    self.assertIn(b"Unknown MCU GPIO input at 0x0258 mask=0x01", result.stderr)
                else:
                    self.assertIn(b"completed-steps=27 units=66 ticks=2970", result.stdout)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
