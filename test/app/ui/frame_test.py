# SPDX-License-Identifier: GPL-3.0-only
"""Unit tests for the independent display viewer's bounded PBM reader."""

import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/app"))

from ui.frame import HEADER, PAYLOAD_BYTES, Frame, read_frame, runtime_directory


class FrameTest(unittest.TestCase):
    def test_runtime_directory(self):
        with patch.dict(os.environ, {"XDG_RUNTIME_DIR": "/selected/runtime"}):
            self.assertEqual(runtime_directory(), Path("/selected/runtime"))
        with patch.dict(os.environ, {"XDG_RUNTIME_DIR": "relative"}):
            with self.assertRaises(ValueError):
                runtime_directory()
        with patch.dict(os.environ, {}, clear=True), patch("sys.platform", "linux"):
            self.assertEqual(runtime_directory(), Path(f"/run/user/{os.getuid()}"))
        with patch.dict(os.environ, {}, clear=True), patch("sys.platform", "darwin"):
            with patch("subprocess.check_output", return_value="/native/user/tmp/\n") as query:
                self.assertEqual(runtime_directory(), Path("/native/user/tmp"))
                query.assert_called_once_with(
                    ["/usr/bin/getconf", "DARWIN_USER_TEMP_DIR"], text=True, timeout=1)
            with patch("subprocess.check_output", side_effect=OSError("unavailable")):
                with self.assertRaises(ValueError):
                    runtime_directory()

    def test_pixel_order(self):
        bits = bytearray(PAYLOAD_BYTES)
        bits[0], bits[16], bits[-1] = 0x81, 0x40, 0x01
        frame = Frame(bytes(bits))
        for x, y in ((0, 0), (7, 0), (1, 1), (127, 63)):
            self.assertTrue(frame.on(x, y))
        for x, y in ((1, 0), (0, 1), (126, 63)):
            self.assertFalse(frame.on(x, y))

    def test_payload_size(self):
        for size in (0, PAYLOAD_BYTES - 1, PAYLOAD_BYTES + 1):
            with self.assertRaises(ValueError):
                Frame(bytes(size))

    def test_files(self):
        with tempfile.TemporaryDirectory(dir=sys.argv[1]) as temporary:
            path = Path(temporary) / "display.pbm"
            payload = bytes(PAYLOAD_BYTES)
            path.write_bytes(HEADER + payload)
            self.assertEqual(read_frame(path), Frame(payload))
            for malformed in (b"", HEADER + payload[:-1], HEADER + payload + b"\0",
                              b"P1\n128 64\n" + payload):
                path.write_bytes(malformed)
                with self.assertRaises(ValueError):
                    read_frame(path)
            path.unlink()
            os.mkfifo(path)
            with self.assertRaises(ValueError):
                read_frame(path)
            path.unlink()
            path.symlink_to("missing")
            with self.assertRaises(OSError):
                read_frame(path)


if __name__ == "__main__":
    Path(sys.argv[1]).mkdir(parents=True, exist_ok=True)
    unittest.main(argv=[sys.argv[0]])
