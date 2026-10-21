# SPDX-License-Identifier: GPL-3.0-only
"""Unchanged original MIDI components through the real headless reference driver."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

from original_dsp_voice import original_records
from test_reference_cli import CONFIG, image, replay_sections

BINARY, GENERATOR, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:5])
ROOT = Path(__file__).resolve().parents[1]
BIT_TICKS = 3249 * 2 * 45
START = 10000000
OPERATIONS = 120000
# Own MCU: LDS; enable PU0 pull; LDAA PORTU/CMPA/BNE poll; LDD #beef; WAI.
# BNE at900d targets9008. Only an actual low GPIO read reaches the marker.
MCU = bytes.fromhex("cf2000 8601 7a027c b60278 8100 26f9 ccbeef 3e")


def events(msb=0x40):
    result = []
    for index, byte in enumerate((0xe0, 0, msb)):
        at = START + index * 11 * BIT_TICKS
        levels = [False] + [bool((byte >> bit) & 1) for bit in range(8)] + [True]
        result.extend((at + bit * BIT_TICKS, level) for bit, level in enumerate(levels))
    return result


def config(msb=0x40, unknown=False):
    text = CONFIG.replace("operations 23", f"operations {OPERATIONS}")
    text += ("dsp-interrupts native-vector-estimates prepare-boundary\n"
             "dsp-input-order input-before-dsp 4\n"
             "dsp-initial tio 0 high synchronized\n"
             "shi-hreq U 01 direct\ninitial drive U 3e 00 00\n")
    for index, (ticks, level) in enumerate(events(msb)):
        value = "unknown raw" if unknown and index == 28 else (
            ("high" if level else "low") + " synchronized")
        text += f"dsp-at {ticks} tio 0 {value}\n"
    return text


class OriginalMidiCli(unittest.TestCase):
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
        cls.identities = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (BINARY, GENERATOR, FIRMWARE, cls.rom)}

    @classmethod
    def tearDownClass(cls):
        for name, expected in cls.identities.items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
                raise ValueError("Changed test executable or original input: " + name)

    def invoke(self, name, text):
        folder = ARTIFACTS / name
        folder.mkdir(exist_ok=True)
        board, flash, trace = (folder / value for value in ("board.bfboard", "mcu.bfnvm", "run.trace"))
        board.write_text(text)
        flash.write_bytes(image(MCU))
        trace.unlink(missing_ok=True)
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", self.rom, "--flash-in", flash, "--trace", trace,
                                 "--instructions", str(OPERATIONS)], capture_output=True,
                                text=True, timeout=40, check=False)
        (folder / "run.log").write_text(result.stdout + result.stderr)
        self.assertTrue(trace.is_file(), result.stderr)
        return result, trace.read_text()

    def verify_events(self, trace, expected):
        observed = re.findall(r"^DSP-INPUT requested=(\d+) actual=(\d+) tio=0 "
                              r"level=(high|low|unknown) synchronized=([01]) ", trace, re.M)
        values = [(int(requested), level, sync) for requested, actual, level, sync in observed
                  if requested == actual]
        self.assertEqual(len(values), len(observed), "An input was observed at the wrong timestamp")
        self.assertEqual(values, [(ticks, "high" if level else "low", "1")
                                  for ticks, level in expected])

    def verify_original_returns(self, trace):
        # Original complete-record boundaries, pinned before native execution.
        # Per byte: one capture return, seven ordinary samples, one final sample.
        for pc, count in (("002d03", 3), ("002cf8", 21), ("002d10", 3),
                          ("000cbb", 1), ("000c05", 1)):
            self.assertEqual(len(re.findall(r"^DSP end=\d+ pc=0x" + pc + " ", trace, re.M)), count)

    def verify_center(self, result, trace):
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("MCU pc=0x9013 d=0xbeef", result.stdout)
        self.assertIn("sp=0x1ff7", result.stdout)
        self.assertRegex(result.stdout, r"idle=[1-9][0-9]* ")
        self.assertIn("DSP-INPUTS observed=30 configured=30", result.stdout)
        self.verify_original_returns(trace)
        outputs = re.findall(r"^SHI-RETURN end=(\d+) signal=hreq driven=1 level=0 ", trace, re.M)
        self.assertEqual(len(outputs), 1)
        # Center pitch is (0 | 64<<7)-8192 ==0. The guest compares this result.
        # Marker must follow the final data bit's sample and precede the stop bit.
        self.assertGreater(int(outputs[0]), events()[-2][0])
        self.assertLess(int(outputs[0]), events()[-1][0])

    def test_original_receiver_parser_and_actual_mcu_return(self):
        result, trace = self.invoke("center", config())
        self.verify_center(result, trace)
        self.verify_events(trace, events())

    def test_noncenter_pitch_must_not_reach_the_center_marker(self):
        result, trace = self.invoke("noncenter", config(0x41))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("MCU pc=0x9008 d=0x0100", result.stdout)
        self.assertIn("sp=0x2000", result.stdout)
        self.assertIn("idle=0", result.stdout)
        self.assertNotIn("signal=hreq driven=1", trace)
        self.assertIn("DSP-INPUTS observed=30 configured=30", result.stdout)
        self.verify_events(trace, events(0x41))
        self.verify_original_returns(trace)

    def test_joint_snapshot_mid_byte_repeats_the_original_result(self):
        result, trace = self.invoke("mid-byte-replay", config() + "checkpoint 50000 2\n")
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.verify_events(prefix, events()[:2])
        self.verify_events(suffixes[0], events()[2:])
        self.assertNotIn("signal=hreq driven=1", prefix)
        self.verify_center(result, prefix + suffixes[0])
        boundary = re.findall(r"^MCU end=(\d+).*operations=50000 ", prefix, re.M)
        self.assertEqual(len(boundary), 1)
        self.assertGreater(int(boundary[0]), events()[1][0])
        self.assertLess(int(boundary[0]), events()[2][0])

    def test_joint_restore_repeats_the_original_unknown_di_fault(self):
        result, trace = self.invoke("unknown-replay", config(unknown=True) + "checkpoint 50000 2\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP timer DI read at X:0xffff8f", result.stderr)
        self.assertIn("pc=0x002ce4 omr=0x600085", result.stderr)
        self.assertIn("DSP-INPUTS observed=29 configured=30", result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.verify_events(prefix, events()[:2])
        self.assertNotIn("signal=hreq driven=1", trace)
        for suffix in suffixes:
            self.assertEqual(suffix.count("DSP-INPUT requested="), 27)
            self.assertEqual(suffix.count("level=unknown synchronized=0"), 1)


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP original MIDI reference CLI: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
