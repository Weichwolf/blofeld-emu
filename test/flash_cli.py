# SPDX-License-Identifier: GPL-3.0-only
"""Exercise private NVM files through the real standalone process."""

import binascii
import hashlib
import json
from pathlib import Path
import stat
import struct
import subprocess
import sys
import tempfile
import unittest

BINARY = Path(sys.argv[1]).resolve()
FIRMWARE = Path(sys.argv[2]).resolve()
ROOT = Path(__file__).resolve().parents[1]


def encoded(unknown=(), code=b""):
    # Own synthetic HC12 NOP image with an independent reset vector/options.
    data = bytearray([0xa7]) * 32768
    data[0x7f0d], data[0x7f0f] = 0xff, 0xfe
    data[-2:] = bytes.fromhex("90 00")
    data[0x1000:0x1000 + len(code)] = code
    known = bytearray([255]) * 4096
    for address in unknown:
        data[address] = 0
        known[address // 8] &= ~(1 << (address % 8))
    body = b"BLFNVM\r\n" + struct.pack("<II", 1, 38912) + data + known + bytes(2048)
    return body + struct.pack("<I", binascii.crc32(body))


def invoke(firmware, *arguments):
    return subprocess.run([BINARY, "--firmware", firmware, *map(str, arguments)],
                          capture_output=True, timeout=30, check=False)


class FileGuards(unittest.TestCase):
    def test_colliding_trace_paths_preserve_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            firmware = root / "input.zip"
            firmware.write_bytes(b"own dummy firmware input")
            state = root / "state.bfnvm"
            original = encoded()
            state.write_bytes(original)
            hardlink = root / "hardlink.bfnvm"
            hardlink.hardlink_to(state)
            symlink = root / "symlink.bfnvm"
            symlink.symlink_to(state)
            for trace in (firmware, state, hardlink, symlink):
                with self.subTest(trace=trace):
                    result = invoke(firmware, "--trace", trace, "--flash-in", state)
                    self.assertEqual(result.returncode, 1)
                    self.assertIn(b"collision", result.stderr)
                    self.assertEqual(state.read_bytes(), original)
                    self.assertEqual(firmware.read_bytes(), b"own dummy firmware input")
            linked_output = root / "firmware.bfnvm"
            linked_output.hardlink_to(firmware)
            result = invoke(firmware, "--flash-out", linked_output)
            self.assertEqual(result.returncode, 1)
            self.assertIn(b"collision", result.stderr)
            self.assertEqual(firmware.read_bytes(), b"own dummy firmware input")

    def test_invalid_options_fail_before_opening_files(self):
        for arguments in (("--flash-in", "wrong.bin"),
                          ("--flash-out", "wrong.bin"),
                          ("--inspect", "--flash-in", "state.bfnvm"),
                          ("--inspect", "--flash-out", "state.bfnvm")):
            with self.subTest(arguments=arguments):
                result = invoke("absent.zip", *arguments)
                self.assertEqual(result.returncode, 1)
                self.assertNotIn(b"Cannot open firmware", result.stderr)
                self.assertRegex(result.stderr.decode(), "extension|--inspect")


class PersistentExecution(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not FIRMWARE.is_file():
            raise unittest.SkipTest("Official 1.25 archive is a private local input")
        manifest = json.loads((ROOT / "doc/downloads.json").read_text())
        expected = next(row["sha256"] for row in manifest["files"]
                        if row["file"] == "Blofeld_Firmware_V1.25.zip")
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Original 1.25 archive identity does not match")

    def test_persisted_image_drives_execution_and_same_path_save(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.bfnvm"
            original = encoded()
            path.write_bytes(original)
            for instructions in (0, 1):
                result = invoke(FIRMWARE, "--flash-in", path, "--flash-out", path,
                                "--instructions", instructions)
                self.assertEqual(result.returncode, 0, result.stderr.decode())
                self.assertIn(f"pc=0x{0x9000 + instructions:04x}".encode(), result.stdout)
                self.assertEqual(path.read_bytes(), original)
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_wai_stops_without_scheduler_and_exports_cold_nvm(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "wai.bfnvm"
            output = root / "saved.bfnvm"
            trace = root / "wai.trace"
            # Own code: LDS #2000; WAI; NOP. No external scheduler or wake input.
            original = encoded(code=bytes.fromhex("cf 20 00 3e a7"))
            path.write_bytes(original)
            result = invoke(FIRMWARE, "--flash-in", path, "--flash-out", output,
                            "--trace", trace, "--instructions", 5)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertIn(b"MCU waiting for an external interrupt", result.stdout)
            self.assertIn(b"pc=0x9004", result.stdout)
            self.assertIn(b"sp=0x1ff7", result.stdout)
            self.assertIn("WAI", trace.read_text())
            self.assertNotIn("pc=0x9004", trace.read_text())
            self.assertEqual(output.read_bytes(), original)

    def test_unknown_code_and_vector_remain_unknown_without_fallback(self):
        for unknown in ((0x1000,), (0x7ffe, 0x7fff)):
            with self.subTest(unknown=unknown), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "state.bfnvm"
                output = Path(directory) / "result.bfnvm"
                original = encoded(unknown)
                path.write_bytes(original)
                result = invoke(FIRMWARE, "--flash-in", path, "--flash-out", output,
                                "--instructions", 1)
                self.assertEqual(result.returncode, 2, result.stderr.decode())
                self.assertIn(b"uninitialized or unmapped", result.stderr.lower())
                if unknown == (0x7ffe, 0x7fff):
                    self.assertIn(b"CPU state unavailable", result.stderr)
                self.assertEqual(output.read_bytes(), original)
                self.assertEqual(path.read_bytes(), original)

    def test_unreadable_reset_options_and_malformed_files_do_not_boot(self):
        valid = encoded()
        damaged = bytearray(valid)
        damaged[100] ^= 1
        for original in (encoded((0x7f0f,)), bytes(damaged), valid[:-1], valid + b"x"):
            with self.subTest(digest=hashlib.sha256(original).hexdigest()), \
                    tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "state.bfnvm"
                output = Path(directory) / "result.bfnvm"
                path.write_bytes(original)
                result = invoke(FIRMWARE, "--flash-in", path, "--flash-out", output,
                                "--instructions", 0)
                self.assertEqual(result.returncode, 1, result.stderr.decode())
                self.assertFalse(output.exists())
                self.assertEqual(path.read_bytes(), original)

    def test_initial_export_and_fault_export_are_private(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for instructions, exit_code in ((0, 0), (100000, 2)):
                path = root / f"original-{instructions}.bfnvm"
                result = invoke(FIRMWARE, "--flash-out", path, "--instructions", instructions)
                self.assertEqual(result.returncode, exit_code, result.stderr.decode())
                data = path.read_bytes()
                self.assertEqual(len(data), 38932)
                self.assertEqual(data[32784:36880], bytes([255]) * 4096)
                self.assertEqual(data[36880:38928], bytes(2048))
                self.assertEqual(binascii.crc32(data[:-4]), struct.unpack("<I", data[-4:])[0])
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertEqual((root / "original-0.bfnvm").read_bytes(),
                             (root / "original-100000.bfnvm").read_bytes())

    def test_missing_input_and_failed_save_are_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state = root / "state.bfnvm"
            output = root / "result.bfnvm"
            result = invoke(FIRMWARE, "--flash-in", state, "--flash-out", output,
                            "--instructions", 0)
            self.assertEqual(result.returncode, 1)
            self.assertFalse(output.exists())
            original = encoded()
            state.write_bytes(original)
            output.mkdir()
            result = invoke(FIRMWARE, "--flash-in", state, "--flash-out", output,
                            "--instructions", 0)
            self.assertEqual(result.returncode, 1)
            self.assertIn(b"Cannot publish MCU NVM file", result.stderr)
            self.assertTrue(output.is_dir())
            self.assertEqual(state.read_bytes(), original)
            self.assertEqual(list(root.glob("*.tmp.*")), [])


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
