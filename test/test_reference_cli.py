# SPDX-License-Identifier: GPL-3.0-only
"""Real CLI reference-board inputs, native WAI time and strict failure boundaries."""

import binascii
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONFIG = """blofeld-reference-board 1
clock 12000000 0 12000000 540000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 450 low
operations 23
initial drive B 16 02 00
initial drive E 03 03 00
"""
# Own DSP ROM: NOP; long absolute JMP ff0000. Independent FM costs: 1 + 4.
ROM = bytes.fromhex("000000 0af080 ff0000")
# Own LDS/LDD/LDX/LDY/ANDCC/WAI/NOP, using independently specified native costs.
WAIT = bytes.fromhex("cf2000 cc1234 ce4567 cd6789 10ef 3e a7")


def image(code):
    data = bytearray([0xa7]) * 32768
    data[0x7f0d], data[0x7f0f] = 255, 254
    data[-2:] = bytes.fromhex("9000")
    data[0x1000:0x1000 + len(code)] = code
    # Own RTI acknowledge/return handler and independent vector.
    data[0x2000:0x2006] = bytes.fromhex("8680 7a0037 0b")
    data[0x7ff0:0x7ff2] = bytes.fromhex("a000")
    body = b"BLFNVM\r\n" + struct.pack("<II", 1, 38912) + data + bytes([255]) * 4096 + bytes(2048)
    return body + struct.pack("<I", binascii.crc32(body))


class ReferenceCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        binary = os.environ.get("BLOFELD_TEST_BINARY")
        if not binary:
            raise unittest.SkipTest("CTest selects the current native executable")
        cls.binary = Path(binary)
        cls.firmware = ROOT / "doc/Blofeld_Firmware_V1.25.zip"
        if cls.firmware.is_file():
            manifest = json.loads((ROOT / "doc/downloads.json").read_text())
            expected = next(row["sha256"] for row in manifest["files"]
                            if row["file"] == cls.firmware.name)
            if hashlib.sha256(cls.firmware.read_bytes()).hexdigest() != expected:
                raise ValueError("Original firmware input identity mismatch")

    def invoke(self, *args, firmware=None):
        return subprocess.run([self.binary, "--firmware", firmware or self.firmware, *map(str, args)],
                              capture_output=True, text=True, timeout=30, check=False)

    def prepared(self, root, config=CONFIG, rom=ROM, code=WAIT):
        board, bootstrap, state = root / "scenario.bfboard", root / "rom.bin", root / "mcu.bfnvm"
        board.write_text(config)
        bootstrap.write_bytes(rom)
        state.write_bytes(image(code))
        return board, bootstrap, state

    def test_conflicting_options_and_output_aliases_preserve_inputs(self):
        for args in (("--reference-board", "absent"), ("--dsp-rom", "absent"),
                     ("--reference-board", "x", "--dsp-rom", "y", "--inspect"),
                     ("--reference-board", "x", "--dsp-rom", "y", "--port-e-input", "fb:03")):
            result = self.invoke(*args, firmware="absent.zip")
            self.assertEqual(result.returncode, 1)
            self.assertRegex(result.stderr, "together|conflicts")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            board, rom, state = self.prepared(root)
            for source in (board, rom):
                before = source.read_bytes()
                for link in (source, root / "hard.bfnvm", root / "sym.bfnvm"):
                    if link.name == "hard.bfnvm":
                        link.hardlink_to(source)
                    elif link.name == "sym.bfnvm":
                        link.symlink_to(source)
                    for output in (("--trace", "--flash-out") if link.suffix == ".bfnvm"
                                   else ("--trace",)):
                        result = self.invoke("--reference-board", board, "--dsp-rom", rom,
                                             output, link, firmware="absent.zip")
                        self.assertEqual(result.returncode, 1)
                        self.assertIn("collision", result.stderr)
                        self.assertEqual(source.read_bytes(), before)
                    if link != source:
                        link.unlink()

    def test_bounded_config_and_rom_fail_before_output_creation(self):
        cases = ["", CONFIG.replace(" 1\n", " 2\n", 1), CONFIG + "clock 1 0 1 1 x\n",
                 CONFIG.replace("operations 23", "operations 0"),
                 CONFIG.replace("initial drive B 16 02", "initial drive B 16 ff"),
                 CONFIG.replace("spi-b 02 04", "spi-b 02 02"),
                 CONFIG + "at 900 drive U 01 00 00\nat 800 drive U 01 01 00\n",
                 CONFIG + "at -1 phy-window 15\n", CONFIG + "initial usb-decoder ff\n",
                 CONFIG + "initial drive Z ff 00 00\n", CONFIG + "unknown 0\n",
                 CONFIG + "operations 2 surplus\n", CONFIG + "\0", "#" * 65537]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            board, rom, _ = self.prepared(root)
            trace = root / "out.trace"
            for text in cases:
                board.write_text(text)
                result = self.invoke("--reference-board", board, "--dsp-rom", rom,
                                     "--trace", trace)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertRegex(result.stderr, "[Rr]eference|[Pp]rivate state")
                self.assertFalse(trace.exists())
            board.write_text(CONFIG)
            for content in (b"", b"x", b"1234", b"x" * (65536 * 3 + 3)):
                rom.write_bytes(content)
                result = self.invoke("--reference-board", board, "--dsp-rom", rom,
                                     "--trace", trace)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertFalse(trace.exists())

    def run_native(self, config=CONFIG, rom=ROM, code=WAIT, instructions=7):
        if not self.firmware.is_file():
            self.skipTest("Original firmware archive is a private local loader input")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            board, bootstrap, state = self.prepared(root, config, rom, code)
            trace, saved = root / "run.trace", root / "saved.bfnvm"
            result = self.invoke("--reference-board", board, "--dsp-rom", bootstrap,
                                 "--flash-in", state, "--flash-out", saved,
                                 "--trace", trace, "--instructions", instructions)
            return result, trace.read_text(), saved.read_bytes() if saved.exists() else None

    def test_actual_native_wai_time_drives_dsp_and_exports_only_mcu_nvm(self):
        result, trace, saved = self.run_native()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("unverified", result.stdout)
        self.assertIn("boot completion is not established", result.stdout)
        self.assertIn("pc=0x900f", result.stdout)
        self.assertIn("instructions=6 operations=23 interrupts=0 cycles=38 idle=17 ticks=380", result.stdout)
        self.assertIn("completed-steps=27 units=66 ticks=2970", result.stdout)
        self.assertIn("DSP-RELEASE requested=450 actual=450 pc=0xff0000", trace)
        steps = re.findall(r"^DSP end=(\d+) pc=(0x\w+) start=(\d+) units=(\d+) ticks=(\d+)$", trace, re.M)
        self.assertEqual(len(steps), 27)
        elapsed = 0
        for index, (end, pc, start, units, ticks) in enumerate(steps):
            cost = (1, 4)[index % 2]
            self.assertEqual(int(pc, 16), (0xff0000, 0xff0001)[index % 2])
            self.assertEqual((int(start), int(units), int(ticks)), (elapsed, cost, cost * 45))
            elapsed += cost * 45
            self.assertEqual(int(end), elapsed + 450)
        self.assertEqual(saved, image(WAIT))

    def test_quantized_release_and_external_events_are_visible(self):
        config = CONFIG.replace("dsp-release 450", "dsp-release 451")
        config += "at 451 drive U 01 01 00\n"
        result, trace, _ = self.run_native(config)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("INPUT requested=451 actual=630", trace)
        self.assertIn("DSP-RELEASE requested=451 actual=630", trace)
        self.assertIn("completed-steps=25 units=61 ticks=2745", result.stdout)

    def test_real_rti_wake_reuses_native_wai_frame(self):
        code = bytes.fromhex("cf2000 8610 7a003b 8680 7a0038 10ef 3e a7 20fd")
        result, trace, saved = self.run_native(CONFIG.replace("operations 23", "operations 600"),
                                             code=code, instructions=12)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("interrupts=1", result.stdout)
        self.assertIn("sp=0x2000", result.stdout)
        self.assertIn("IRQ vector=0xfff0 stack-reused=1", trace)
        self.assertEqual(saved, image(code))

    def test_explicit_shared_tie_and_pad_order_rejections(self):
        config = CONFIG.replace("operations 23", "operations 10")
        config = config.replace("dsp-release 450", "dsp-release 1170")
        config = config.replace("initial drive B 16 02", "initial drive B 16 06")
        config += "at 1170 drive B 00 00 00\n"
        # Own data/direction writes establish the pads, then release external drives.
        code = bytes.fromhex("8606 7a0001 8616 7a0003 8600 a7 a7 7a0001")
        for replacement, message in ((('ordering dsp-before-mcu', 'ordering reject'),
                                      "Unqualified"),
                                     (('clock-before-select', 'reject'), "Unqualified simultaneous")):
            result, _, _ = self.run_native(config.replace(*replacement), code=code, instructions=10)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn(message, result.stderr)
            self.assertIn("mcu-pc=", result.stderr)
            self.assertIn("dsp-pc=", result.stderr)
        result, _, _ = self.run_native(config, code=code, instructions=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("ordered-pairs=1 ties=1", result.stdout)

    def test_receiver_pull_does_not_validate_an_unknown_loopback_source(self):
        config = CONFIG + "loopback U 01 P 01\ninitial drive P fe 00 00\n"
        # Own LDAA #1; STAA PERP (025c); LDAA PTP (0258). Other P bits are low.
        code = bytes.fromhex("8601 7a025c b60258")
        result, _, saved = self.run_native(config, code=code, instructions=3)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown MCU GPIO input at 0x0258 mask=0x01", result.stderr)
        self.assertEqual(saved, image(code))
        for level in (0, 1):
            result, _, _ = self.run_native(config + f"initial drive U 01 {level:02x} 00\n",
                                          code=code, instructions=3)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(f"d=0x{level << 8:04x}", result.stdout)

    def test_unknown_pad_and_missing_rom_word_fail_visibly(self):
        result, _, saved = self.run_native(CONFIG.replace("initial drive B 16 02 00\n", ""))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown reference SPI pad read at 0x0001", result.stderr)
        self.assertIn("mcu-pc=0x9000 dsp-pc=0xff0000", result.stderr)
        self.assertEqual(saved, image(WAIT))
        result, _, _ = self.run_native(rom=bytes(3))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("0xff0001", result.stderr)
        self.assertIn("DSP pc=", result.stderr)
        self.assertIn("MCU pc=", result.stderr)


if __name__ == "__main__":
    unittest.main()
