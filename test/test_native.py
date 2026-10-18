# SPDX-License-Identifier: GPL-3.0-only
"""Exercise the native CLI and its rejection boundaries in separate processes."""

import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]


class NativeCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binary = os.environ.get("BLOFELD_TEST_BINARY")
        if not binary:
            raise unittest.SkipTest("Run through CTest to select the current native binary")
        cls.binary = Path(binary)

    def inspect(self, data):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "input.bin"
            path.write_bytes(data)
            return subprocess.run(
                [self.binary, "--firmware", path, "--inspect"],
                capture_output=True, text=True, timeout=15, check=False,
            )


class NativeParserTest(NativeCli):
    def test_no_arguments_reports_usage(self):
        result = subprocess.run([self.binary], capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 1)
        self.assertIn("Usage:", result.stderr)

    def test_truncated_headers_are_rejected(self):
        for data in (b"", b"MThd", b"PK\x03\x04", b"PK\x05\x06" + bytes(17)):
            with self.subTest(data=data):
                result = self.inspect(data)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertTrue(result.stderr.strip())

    def test_empty_smf_does_not_count_as_firmware(self):
        data = b"MThd" + struct.pack(">IHHH", 6, 0, 1, 96)
        data += b"MTrk" + struct.pack(">I", 4) + b"\x00\xff\x2f\x00"
        result = self.inspect(data)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("six-block", result.stderr)

    def test_invalid_gpio_fixture_syntax_is_rejected(self):
        for fixture in ("", "0", "zz:00", "ff-00", "fff:00"):
            with self.subTest(fixture=fixture):
                result = subprocess.run(
                    [self.binary, "--firmware", "unused", "--port-e-input", fixture],
                    capture_output=True, text=True, timeout=15, check=False,
                )
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("GPIO", result.stderr)


class NativeReferenceTest(NativeCli):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.reference = ROOT / "doc/Blofeld_Firmware_V1.25.zip"
        if not cls.reference.exists():
            raise unittest.SkipTest("Official firmware is a private local reference")
        with zipfile.ZipFile(cls.reference) as archive:
            name, = (name for name in archive.namelist() if name.lower().endswith(".mid"))
            cls.midi = archive.read(name)

    def test_reference_zip_and_midi_are_accepted(self):
        for data in (self.reference.read_bytes(), self.midi):
            with self.subTest(size=len(data)):
                result = self.inspect(data)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("32768 MCU bytes, 127 DSP records, 4841 checked data packets", result.stdout)

    def test_repacked_zip_and_crc_rejection(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "reference.zip"
            with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as archive:
                archive.writestr("firmware.mid", self.midi)
            data = path.read_bytes()
        self.assertEqual(self.inspect(data).returncode, 0)
        damaged = bytearray(data)
        # Stored ZIP data starts after the 30-byte local header and this exact filename.
        damaged[30 + len("firmware.mid") + 10] ^= 1
        result = self.inspect(damaged)
        self.assertEqual(result.returncode, 1)
        self.assertIn("CRC", result.stderr)

    def test_incomplete_midi_is_rejected(self):
        for cut in (10, 21, len(self.midi) - 1):
            with self.subTest(cut=cut):
                result = self.inspect(self.midi[:cut])
                self.assertEqual(result.returncode, 1, result.stderr)

    def test_real_boot_prefix_stops_at_unknown_pin(self):
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "mcu.trace"
            result = subprocess.run(
                [self.binary, "--firmware", self.reference, "--instructions", "100", "--trace", trace],
                capture_output=True, text=True, timeout=15, check=False,
            )
            listing = trace.read_text()
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("18 completed MCU instructions", result.stderr)
        self.assertIn("read at 0x0008", result.stderr)
        self.assertIn("pc=0x8029 d=0xff90", result.stderr)
        self.assertIn("io-write 0x0012=0x21 INITEE", listing)
        self.assertIn("io-write 0x0046=0x90 TSCR1", listing)
        self.assertIn("LDAB 0x8", listing)

    @unittest.skipUnless(Path("/dev/full").exists(), "Linux write-failure fixture is unavailable")
    def test_trace_write_failure_is_reported(self):
        result = subprocess.run(
            [self.binary, "--firmware", self.reference, "--instructions", "100", "--trace", "/dev/full"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertTrue(result.stderr.strip())

    def test_explicit_gpio_fixtures_reach_the_next_unsupported_peripheral(self):
        for fixture, completed in (("fb:00", 35), ("fb:08", 33)):
            with self.subTest(fixture=fixture), tempfile.TemporaryDirectory() as temporary:
                trace = Path(temporary) / "mcu.trace"
                result = subprocess.run(
                    [self.binary, "--firmware", self.reference, "--instructions", "100",
                     "--port-e-input", fixture, "--trace", trace],
                    capture_output=True, text=True, timeout=15, check=False,
                )
                listing = trace.read_text()
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(f"{completed} completed MCU instructions", result.stderr)
                self.assertIn("MCU read at 0x0037", result.stderr)
                self.assertIn("clock source is required", result.stderr)
                self.assertIn("pc=0x8049", result.stderr)
                self.assertIn("Explicit simulated", result.stdout)
                self.assertIn("board wiring is unverified", result.stdout)
                self.assertIn("reserved/write ignored", listing)
                self.assertEqual("CMPB #0x0" in listing, fixture == "fb:00")

    def test_inconsistent_gpio_drive_mask_is_rejected(self):
        result = subprocess.run(
            [self.binary, "--firmware", self.reference, "--port-e-input", "01:ff"],
            capture_output=True, text=True, timeout=15, check=False,
        )
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("Invalid MCU GPIO external drive", result.stderr)
