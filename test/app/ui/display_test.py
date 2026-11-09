# SPDX-License-Identifier: GPL-3.0-only
"""PNG export uses the same exact-pixel painter as the native LCD window."""

from pathlib import Path
import sys
import tempfile
import unittest

import cairo

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/app"))

from ui.display import save_png
from ui.frame import Frame, PAYLOAD_BYTES


class ScreenshotTest(unittest.TestCase):
    def test_snapshot_png(self):
        bits = bytearray(PAYLOAD_BYTES)
        bits[0] = 0x80
        with tempfile.TemporaryDirectory(dir=sys.argv[1]) as directory:
            path = Path(directory) / "display.png"
            save_png(Frame(bytes(bits)), path)
            image = cairo.ImageSurface.create_from_png(str(path))
            self.assertEqual((image.get_width(), image.get_height()), (512, 256))
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
