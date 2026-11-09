# SPDX-License-Identifier: GPL-3.0-only
"""Display painting and publisher lifecycle remain independent of firmware execution."""

from pathlib import Path
import os
import sys
import tempfile
import unittest
import time

import cairo

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/app"))

from ui.display import paint
from ui.frame import Frame, PAYLOAD_BYTES
from ui.frame import HEADER
from ui.watch import DisplayWatch
from gi.repository import GLib


class DisplayTest(unittest.TestCase):
    def test_publisher_lifecycle(self):
        with tempfile.TemporaryDirectory(dir=sys.argv[1]) as temporary:
            directory = Path(temporary) / "blofeld-emu"
            directory.mkdir()
            bitmap = directory / "display.pbm"
            bitmap.write_bytes(HEADER + bytes(PAYLOAD_BYTES))
            changes = []
            watch = DisplayWatch(temporary, lambda frame, message: changes.append((frame, message)))
            writer = None
            try:
                self.assertEqual(changes[-1], (None, "Emulator off"))
                path = directory / "display.live"
                os.mkfifo(path, 0o600)
                reader = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                writer = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
                os.close(reader)
                watch.schedule()

                def await_message(message):
                    deadline = time.monotonic() + 2.5
                    context = GLib.MainContext.default()
                    while changes[-1][1] != message and time.monotonic() < deadline:
                        while context.pending():
                            context.iteration(False)
                        time.sleep(0.005)
                    self.assertEqual(changes[-1][1], message)

                await_message("")
                self.assertEqual(changes[-1][0], Frame(bytes(PAYLOAD_BYTES)))
                before = bitmap.stat().st_mtime_ns
                os.close(writer)
                writer = None
                await_message("Emulator off")
                self.assertIsNone(changes[-1][0])
                self.assertEqual(bitmap.stat().st_mtime_ns, before)
            finally:
                if writer is not None:
                    os.close(writer)
                watch.close()

    def test_pixel_painting(self):
        bits = bytearray(PAYLOAD_BYTES)
        bits[0] = 0x80
        image = cairo.ImageSurface(cairo.FORMAT_ARGB32, 512, 256)
        paint(cairo.Context(image), 512, 256, Frame(bytes(bits)))
        image.flush()
        data = image.get_data()

        def pixel(x, y):
            offset = y * image.get_stride() + x * 4
            word = int.from_bytes(data[offset:offset + 4], sys.byteorder)
            return word >> 24, (word >> 16) & 255, (word >> 8) & 255, word & 255

        for x, y in ((0, 0), (3, 0), (0, 3), (3, 3)):
            alpha, *color = pixel(x, y)
            self.assertEqual(alpha, 255)
            self.assertTrue(all(value < 30 for value in color))
        for x, y in ((4, 0), (0, 4), (511, 255)):
            alpha, *color = pixel(x, y)
            self.assertEqual(alpha, 255)
            self.assertTrue(all(value > 230 for value in color))


if __name__ == "__main__":
    Path(sys.argv[1]).mkdir(parents=True, exist_ok=True)
    unittest.main(argv=[sys.argv[0]])
