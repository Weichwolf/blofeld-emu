# SPDX-License-Identifier: GPL-3.0-only
"""Exact DSP TIO observations through the real common-time headless driver."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

from test_reference_cli import CONFIG, image, replay_sections

BINARY, FIXTURE, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:5])
ROOT = Path(__file__).resolve().parents[1]
CONTRACT = "dsp-interrupts native-vector-estimates prepare-boundary\n"
INPUTS = ("dsp-input-order input-before-dsp 4\n"
          "dsp-initial tio 0 high synchronized\n"
          "dsp-at 1200 tio 0 low synchronized\n")


def store(address, value):
    return bytes((0x86, value, 0x7a, address >> 8, address & 255))


def check_pin(expected):
    # Own LDAA PORTU / CMPA / BEQ. A wrong value takes an actual unmapped read.
    return bytes((0xb6, 0x02, 0x78, 0x81, expected, 0x27, 3, 0xb6, 0x30, 0))


class DspInputsCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not FIRMWARE.is_file():
            raise unittest.SkipTest("Private original firmware input is absent")
        manifest = json.loads((ROOT / "doc/downloads.json").read_text())
        expected = next(v["sha256"] for v in manifest["files"] if v["file"] == FIRMWARE.name)
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Changed official firmware archive")
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        cls.rom = ARTIFACTS / "own-capture-irq.bin"
        subprocess.run([FIXTURE, "capture-irq", cls.rom], capture_output=True, check=True, timeout=30)

    @staticmethod
    def config():
        return (CONFIG.replace("operations 23", "operations 100") + CONTRACT + INPUTS +
                "shi-hreq U 01 direct\ninitial drive U 3e 00 00\n")

    def invoke(self, name, config, code=None, instructions=8):
        board = ARTIFACTS / (name + ".bfboard")
        nvm = ARTIFACTS / (name + ".bfnvm")
        trace = ARTIFACTS / (name + ".trace")
        board.write_text(config)
        nvm.write_bytes(image(code if code is not None else
                              store(0x27c, 1) + check_pin(1) + check_pin(0)))
        trace.unlink(missing_ok=True)
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", self.rom, "--flash-in", nvm,
                                 "--trace", trace, "--instructions", str(instructions)],
                                capture_output=True, text=True, timeout=30, check=False)
        (ARTIFACTS / (name + ".log")).write_text(result.stdout + result.stderr)
        return result, trace.read_text() if trace.exists() else ""

    def test_mid_instruction_capture_executes_actual_vector_and_gpio_read(self):
        # Independent FM costs: seven 2-unit setup instructions, NOP1/JMP4,
        # then a two-word MOVEP fast vector2. Release450 gives HREQ1395.
        for early_contract in (False, True):
            config = self.config()
            if early_contract:
                config = config.replace(CONTRACT, "").replace("clock ", CONTRACT + "clock ", 1)
            result, trace = self.invoke("capture-early-" + str(early_contract), config)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("DSP-INPUT requested=1200 actual=1200 tio=0 level=low synchronized=1", trace)
            self.assertRegex(trace, r"RETURN end=1395 .*signal=hreq driven=1 level=0")
            self.assertIn("instructions=8 operations=8 interrupts=0 cycles=23 idle=0 ticks=230", result.stdout)
            self.assertIn("DSP-INPUTS observed=1 configured=1", result.stdout)

    def test_completion_ties_obey_both_explicit_orders(self):
        for order, input_first in (("input-before-dsp", True), ("dsp-before-input", False)):
            config = self.config().replace("dsp-at 1200", "dsp-at 1125").replace("input-before-dsp", order)
            result, trace = self.invoke("tie-" + order, config)
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-INPUT requested=1125 actual=1125")
            completed = trace.index("DSP end=1125 ")
            self.assertEqual(source < completed, input_first)
            self.assertRegex(trace, r"RETURN end=1215 .*signal=hreq driven=1 level=0")

    def test_shared_mcu_write_forwards_due_and_exhausted_dsp_input_sources(self):
        code = store(0x27c, 1) + check_pin(1) + store(1, 2) + check_pin(0)
        # S12CPUV2 Rev4 printed279: HCS12 STAA extended uses PwO. The
        # native write is cycle19/tick1710; the final O ends at1800.
        config = self.config() + "dsp-at 1710 tio 0 high synchronized\n"
        for order, input_first in (("dsp-before-mcu", True), ("mcu-before-dsp", False)):
            scenario = config.replace("ordering dsp-before-mcu", "ordering " + order)
            result, trace = self.invoke("shared-" + order, scenario, code, instructions=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-INPUT requested=1710 actual=1710")
            write = trace.index("WRITE cycle=19 address=0x0001")
            self.assertEqual(source < write, input_first)
            self.assertIn("cycles=27 idle=0 ticks=270", result.stdout)
            self.assertIn("spi-writes=1 frames=0 ordered-pairs=0 ties=1", result.stdout)
            self.assertIn("DSP-INPUTS observed=2 configured=2", result.stdout)
        result, trace = self.invoke("shared-reject", config.replace(
            "ordering dsp-before-mcu", "ordering reject"), code, instructions=10)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("simultaneous external event and MCU shared write at 0x0001", result.stderr)
        self.assertNotIn("DSP-INPUT requested=1710", trace)

    def test_unqualified_tie_and_live_admission_changes_reject(self):
        cases = [(self.config().replace("dsp-at 1200", "dsp-at 1125").replace(
                    "input-before-dsp", "reject"), "simultaneous"),
                 (self.config().replace("prepare-boundary", "reject-changes"), "changed before"),
                 (self.config().replace(CONTRACT, ""), "Unqualified DSP interrupt")]
        for index, (config, reason) in enumerate(cases):
            result, _ = self.invoke("unqualified-" + str(index), config)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn(reason, result.stderr)

    def test_checkpoint_restores_both_future_and_exhausted_input_sources(self):
        for operation, prefix_events, suffix_events in ((4, 0, 1), (5, 1, 0)):
            config = self.config() + f"checkpoint {operation} 2\n"
            result, trace = self.invoke("replay-" + str(operation), config)
            self.assertEqual(result.returncode, 0, result.stderr)
            prefix, suffixes = replay_sections(trace)
            self.assertEqual(prefix.count("DSP-INPUT requested="), prefix_events)
            self.assertEqual(len(suffixes), 3)
            self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
            self.assertEqual(suffixes[0].count("DSP-INPUT requested="), suffix_events)
            self.assertIn("DSP-INPUTS observed=1 configured=1", result.stdout)

    def test_native_wai_periods_still_dispatch_exact_dsp_inputs(self):
        config = self.config().replace("operations 100", "operations 200").replace(
            "dsp-at 1200", "dsp-at 10000")
        code = store(0x27c, 1) + bytes.fromhex("cf2000 3e")
        result, trace = self.invoke("during-wai", config, code, instructions=1000)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DSP-INPUT requested=10000 actual=10000", trace)
        self.assertRegex(result.stdout, r"idle=[1-9][0-9]* ")
        self.assertRegex(trace, r"RETURN end=[0-9]+ .*signal=hreq driven=1 level=0")

    def test_event_budget_preserves_delivered_prefix_and_unknown_input_fails(self):
        config = self.config().replace("input-before-dsp 4", "input-before-dsp 1")
        result, trace = self.invoke("exhausted-budget", config +
                                   "dsp-at 1210 tio 0 high synchronized\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("event budget exhausted", result.stderr)
        self.assertEqual(trace.count("DSP-INPUT requested="), 1)
        self.assertIn("DSP-INPUTS observed=1 configured=2", result.stderr)
        config = self.config().replace("low synchronized", "unknown raw")
        result, _ = self.invoke("unknown-raw", config)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP timer capture input during clock at X:0xffff8f", result.stderr)
        # Seven setup instructions consume14 units, followed by NOP1. The
        # failing JMP4 changes native PC but cannot charge its180 nominal ticks.
        self.assertIn("completed-reference-ticks=675 completed-native-cycle-units=15", result.stderr)
        self.assertIn("completed-steps=8 units=15 ticks=675", result.stderr)
        self.assertIn("dsp-pc=", result.stderr)

    def test_faulted_unknown_input_suffix_repeats_after_joint_restore(self):
        config = self.config().replace("low synchronized", "unknown raw") + "checkpoint 4 2\n"
        result, trace = self.invoke("replayed-unknown", config)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP timer capture input during clock at X:0xffff8f", result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertNotIn("DSP-INPUT requested=", prefix)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.assertEqual(suffixes[0].count("level=unknown synchronized=0"), 1)
        self.assertIn("DSP-INPUTS observed=1 configured=1", result.stderr)

    def test_invalid_input_contracts_fail_before_trace_creation(self):
        cases = [self.config().replace("input-before-dsp 4", "input-before-dsp 0"),
                 self.config().replace("input-before-dsp", "guess"),
                 self.config().replace("dsp-input-order input-before-dsp 4\n", ""),
                 self.config().replace("tio 0 low", "tio 2 low"),
                 self.config().replace("low synchronized", "unknown synchronized"),
                 self.config().replace("dsp-at 1200", "dsp-at 100"),
                 self.config() + "dsp-at 1100 tio 0 high synchronized\n",
                 self.config() + "dsp-at 1200 tio 0 high synchronized\n",
                 self.config() + CONTRACT,
                 self.config().replace("prepare-boundary", "guess")]
        for index, config in enumerate(cases):
            result, trace = self.invoke("invalid-" + str(index), config)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")
            self.assertRegex(result.stderr, "[Dd]sp|DSP|[Rr]eference")

    def test_actual_release_may_not_skip_a_due_input(self):
        # Native reset ends450; the first 1-cycle LDAA ends540. Requested451
        # therefore releases only at540; input500 must reject instead of backdating.
        config = self.config().replace("dsp-release 450", "dsp-release 451").replace(
            "dsp-at 1200", "dsp-at 500")
        result, trace = self.invoke("late-release", config)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("actual DSP release", result.stderr)
        self.assertNotIn("DSP-INPUT requested=", trace)


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP reference DSP input tests: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
