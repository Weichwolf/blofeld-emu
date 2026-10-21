# SPDX-License-Identifier: GPL-3.0-only
"""Host file bytes through the actual original DSP TIO/IRQ/ring/parser path."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

from original_dsp_voice import original_records
from reference_original_midi_cli import BIT_TICKS, MCU, OPERATIONS, START, events
from test_reference_cli import CONFIG

BINARY, GENERATOR, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:])
ROOT = Path(__file__).resolve().parents[1]


def configuration(reference=True):
    text = CONFIG.replace("operations 23", f"operations {OPERATIONS}")
    text += ("dsp-interrupts native-vector-estimates prepare-boundary\n"
             "dsp-input-order input-before-dsp 4\n"
             "dsp-initial tio 0 high synchronized\n"
             "shi-hreq U 01 direct\ninitial drive U 3e 00 00\n"
             f"midi-dsp-tio 0 after {START} synchronized\n")
    if reference:
        text += f"midi-dsp-reference {BIT_TICKS} 1\n"
    return text


class DspHostMidiCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
        original_records(FIRMWARE, reference)
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        cls.rom = ARTIFACTS / "private-original-midi.bin"
        result = subprocess.run([GENERATOR, FIRMWARE, cls.rom], capture_output=True,
                                text=True, timeout=30, check=False)
        if result.returncode:
            raise ValueError(result.stdout + result.stderr)
        cls.rom_digest = hashlib.sha256(cls.rom.read_bytes()).hexdigest()

    def invoke(self, name, msb=0x40, text=None):
        from test_reference_cli import image
        folder = ARTIFACTS / name
        folder.mkdir(exist_ok=True)
        board, flash, trace, source = [folder / value for value in
                                       ("board.bfboard", "mcu.bfnvm", "run.trace", "midi.bytes")]
        board.write_text(configuration() if text is None else text)
        flash.write_bytes(image(MCU))
        message = bytes((0xe0, 0, msb))
        source.write_bytes(message)
        trace.unlink(missing_ok=True)
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                  (BINARY, FIRMWARE, self.rom, board, flash, source)}
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", self.rom, "--flash-in", flash, "--trace", trace,
                                 "--instructions", str(OPERATIONS), "--midi-file", source],
                                capture_output=True, text=True, timeout=40, check=False)
        (folder / "run.log").write_text(result.stdout + result.stderr)
        for path, digest in before.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), digest)
        return result, trace.read_text() if trace.is_file() else "", message

    def verify_reference_frames(self, trace, msb):
        bits = re.findall(r"^DSP-MIDI-BIT requested=(\d+) actual=(\d+) tio=0 "
                          r"level=([01]) bit=(\d+) synchronized=1 ", trace, re.M)
        payload = [(int(wanted), bool(int(level))) for wanted, actual, level, bit in bits
                   if int(bit) < 10 and wanted == actual]
        self.assertEqual(payload, events(msb))
        self.assertTrue(all(wanted == actual for wanted, actual, _, _ in bits))
        for pc, count in (("002d03", 3), ("002cf8", 21), ("002d10", 3),
                          ("000cbb", 1), ("000c05", 1)):
            self.assertEqual(len(re.findall(r"^DSP end=\d+ pc=0x" + pc + " ", trace, re.M)), count)
        self.assertIn(f"DSP-MIDI-SOURCE first={START} baud=0 "
                      f"reference-bit-ticks={BIT_TICKS} reference-idle-bits=1", trace)

    def test_original_center_pitch_reaches_actual_mcu_marker(self):
        result, trace, message = self.invoke("center")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("MCU pc=0x9013 d=0xbeef", result.stdout)
        self.assertRegex(result.stdout, r"idle=[1-9][0-9]* ")
        self.assertRegex(result.stdout, r"MIDI accepted=3 queued=0 started=1 dsp-events=\d+ dsp-frames=3")
        self.assertIn(f"bytes={message.hex()}\n", trace)
        self.verify_reference_frames(trace, 0x40)
        marker = re.findall(r"^SHI-RETURN end=(\d+) signal=hreq driven=1 level=0 ", trace, re.M)
        self.assertEqual(len(marker), 1)
        self.assertGreater(int(marker[0]), events()[-2][0])
        self.assertLess(int(marker[0]), events()[-1][0])

    def test_noncenter_bytes_cannot_reach_the_center_marker(self):
        result, trace, _ = self.invoke("noncenter", msb=0x41)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("MCU pc=0x9008 d=0x0100", result.stdout)
        self.assertNotIn("signal=hreq driven=1", trace)
        self.verify_reference_frames(trace, 0x41)

    def test_standard_baud_is_not_changed_to_match_guest_timer_constants(self):
        result, trace, message = self.invoke("standard", text=configuration(reference=False))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"DSP-MIDI-SOURCE first={START} baud=31250 reference-bit-ticks=0 "
                      "reference-idle-bits=0", trace)
        bits = re.findall(r"^DSP-MIDI-BIT requested=(\d+) actual=(\d+) tio=0 "
                          r"level=([01]) bit=(\d+) synchronized=1 ", trace, re.M)
        expected = []
        for index, byte in enumerate(message):
            levels = [False] + [bool(byte & (1 << bit)) for bit in range(8)] + [True]
            expected.extend((START + (index * 10 + bit) * 17280, level)
                            for bit, level in enumerate(levels))
        self.assertEqual([(int(at), bool(int(level))) for at, actual, level, _ in bits
                          if at == actual], expected)

    def test_conflicting_route_or_incomplete_order_rejects_before_execution(self):
        for index, text in enumerate((
                configuration() + "midi-sci PR2 6000000 after 0\n",
                configuration() + f"dsp-at {START + 1} tio 0 high synchronized\n",
                configuration().replace("after 10000000", "after 0"),
                configuration().replace("dsp-input-order input-before-dsp 4\n", ""),
                configuration().replace(f"midi-dsp-reference {BIT_TICKS} 1", "midi-dsp-reference 0 1"))):
            result, trace, _ = self.invoke(f"invalid-{index}", text=text)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")

    def test_unqualified_simultaneous_sources_fail_with_both_cpu_contexts(self):
        text = configuration() + f"dsp-at {START} tio 1 high synchronized\n"
        result, _, _ = self.invoke("collision", text=text)
        self.assertEqual(result.returncode, 2)
        self.assertIn(f"Simultaneous reference DSP input sources at={START}", result.stderr)
        self.assertIn("mcu-pc=", result.stderr)
        self.assertIn("dsp-pc=", result.stderr)


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP original DSP host MIDI: private original firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
