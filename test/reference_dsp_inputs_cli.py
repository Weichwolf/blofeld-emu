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
CONTRACT = "dsp-execution reference-effects\n"
INPUTS = ("dsp-input-order input-before-dsp 4\n"
          "dsp-initial tio 0 high synchronized\n"
          "dsp-at 3000 tio 0 low synchronized\n")


def store(address, value):
    return bytes((0x86, value, 0x7a, address >> 8, address & 255))


def check_pin(expected):
    # Authored LDAA PORTU / CMPA / BEQ. A wrong value takes an actual unmapped read.
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
        return (CONFIG.replace("operations 23", "operations 100") + INPUTS +
                "shi-hreq U 01 direct\ninitial drive U 3e 00 00\n")

    def invoke(self, name, config, code=None, instructions=28):
        board = ARTIFACTS / (name + ".bfboard")
        nvm = ARTIFACTS / (name + ".bfnvm")
        trace = ARTIFACTS / (name + ".trace")
        board.write_text(config)
        nvm.write_bytes(image(code if code is not None else
                              store(0x27c, 1) + bytes((0xa7,)) * 20 + check_pin(1) + check_pin(0)))
        trace.unlink(missing_ok=True)
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", self.rom, "--flash-in", nvm,
                                 "--trace", trace, "--instructions", str(instructions)],
                                capture_output=True, text=True, timeout=30, check=False)
        (ARTIFACTS / (name + ".log")).write_text(result.stdout + result.stderr)
        return result, trace.read_text() if trace.exists() else ""

    def test_mid_instruction_capture_executes_actual_vector_and_gpio_read(self):
        # Authored MCU: setup4 + NOP20 + two seven-period pin checks, plus
        # five reset periods. The20 NOPs let the actual DSP setup retire before
        # the selected capture edge; they do not replace a native DSP effect.
        for early_contract in (False, True):
            config = self.config()
            if early_contract:
                config = config.replace(CONTRACT, "").replace("clock ", CONTRACT + "clock ", 1)
            result, trace = self.invoke("capture-early-" + str(early_contract), config)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("DSP-INPUT requested=3000 actual=3000 tio=0 level=low synchronized=1", trace)
            self.assertRegex(trace, r"RETURN end=3105 .*signal=hreq driven=1 level=0")
            self.assertIn("instructions=28 operations=28 interrupts=0 cycles=43 idle=0 ticks=430", result.stdout)
            self.assertIn("DSP-INPUTS observed=1 configured=1", result.stdout)

    def test_completion_ties_obey_both_explicit_orders(self):
        for order, input_first in (("input-before-dsp", True), ("dsp-before-input", False)):
            config = self.config().replace("dsp-at 3000", "dsp-at 3015").replace("input-before-dsp", order)
            result, trace = self.invoke("tie-" + order, config)
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-INPUT requested=3015 actual=3015")
            completed = trace.index("DSP-PIPELINE start=2970 end=3015 ")
            self.assertEqual(source < completed, input_first)
            # Input-first admits before NOP starts. DSP-first has already
            # advanced the NOP PCU effect and must retire it before admission,
            # delaying service by two45-tick periods. Two vector reads precede I/O.
            returned = 3105 if input_first else 3195
            self.assertRegex(trace, rf"RETURN end={returned} .*signal=hreq driven=1 level=0")

    def test_shared_mcu_write_forwards_due_and_exhausted_dsp_input_sources(self):
        code = store(0x27c, 1) + bytes((0xa7,)) * 20 + check_pin(1) + store(1, 2) + check_pin(0)
        # S12CPUV2 Rev4 printed279: HCS12 STAA extended uses PwO. The
        # native write is cycle39/tick3510; the final O ends at3600.
        config = self.config() + "dsp-at 3510 tio 0 high synchronized\n"
        for order, input_first in (("input-before-mcu", True), ("mcu-before-input", False)):
            scenario = config + "dsp-input-write-order " + order + "\n"
            if not input_first:
                scenario = scenario.replace("ordering dsp-before-mcu", "ordering mcu-before-dsp")
            result, trace = self.invoke("shared-" + order, scenario, code, instructions=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-INPUT requested=3510 actual=3510")
            write = trace.index("WRITE cycle=39 address=0x0001")
            self.assertEqual(source < write, input_first)
            self.assertIn("cycles=47 idle=0 ticks=470", result.stdout)
            self.assertIn("spi-writes=1 frames=0 ordered-pairs=0 ties=1", result.stdout)
            self.assertIn("DSP-INPUTS observed=2 configured=2", result.stdout)
        result, trace = self.invoke("shared-reject", config + "dsp-input-write-order reject\n", code, instructions=30)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("simultaneous external event and MCU shared write at 0x0001", result.stderr)
        self.assertNotIn("DSP-INPUT requested=3510", trace)

    def test_unqualified_input_native_tie_rejects_before_observation(self):
        config = self.config().replace("dsp-at 3000", "dsp-at 3015").replace(
            "input-before-dsp", "reject")
        result, trace = self.invoke("unqualified-tie", config)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("simultaneous", result.stderr)
        self.assertNotIn("DSP-INPUT requested=3015", trace)

    def test_checkpoint_restores_both_future_and_exhausted_input_sources(self):
        for operation, prefix_events, suffix_events in ((24, 0, 1), (25, 1, 0)):
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
            "dsp-at 3000", "dsp-at 10000")
        code = store(0x27c, 1) + bytes.fromhex("cf2000 3e")
        result, trace = self.invoke("during-wai", config, code, instructions=3000)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DSP-INPUT requested=10000 actual=10000", trace)
        self.assertRegex(result.stdout, r"idle=[1-9][0-9]* ")
        self.assertRegex(trace, r"RETURN end=[0-9]+ .*signal=hreq driven=1 level=0")

    def test_event_budget_preserves_delivered_prefix_and_unknown_input_fails(self):
        config = self.config().replace("input-before-dsp 4", "input-before-dsp 1")
        result, trace = self.invoke("exhausted-budget", config +
                                   "dsp-at 3010 tio 0 high synchronized\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("external source exceeded its period budget", result.stderr)
        self.assertEqual(trace.count("DSP-INPUT requested="), 1)
        self.assertIn("DSP-INPUTS observed=1 configured=2", result.stderr)
        config = self.config().replace("low synchronized", "unknown raw")
        result, _ = self.invoke("unknown-raw", config)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP timer capture input during clock at X:0xffff8f", result.stderr)
        # The raw observation at3000 is seen by the next real45-tick period.
        # DMA/F2/native publication precedes its timer clock, so the fault keeps
        # that accepted prefix instead of charging old nominal instruction costs.
        self.assertIn("ticks=3015 timebase=540000000", result.stderr)
        self.assertIn("DSP DMA clock finish pc=", result.stderr)
        self.assertIn("HC12 cycle execution pc=", result.stderr)

    def test_faulted_unknown_input_suffix_repeats_after_joint_restore(self):
        config = self.config().replace("low synchronized", "unknown raw") + "checkpoint 24 2\n"
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
                 self.config().replace("dsp-at 3000", "dsp-at 100"),
                 self.config() + "dsp-at 2900 tio 0 high synchronized\n",
                 self.config() + "dsp-at 3000 tio 0 high synchronized\n",
                 self.config() + CONTRACT,
                 self.config().replace(CONTRACT, ""),
                 self.config().replace("reference-effects", "guess"),
                 self.config() + "dsp-interrupts native-vector-estimates reject-changes\n"]
        for index, config in enumerate(cases):
            result, trace = self.invoke("invalid-" + str(index), config)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")
            self.assertRegex(result.stderr, "[Dd]sp|DSP|[Rr]eference")

    def test_due_input_observes_the_pad_while_dsp_reset_is_still_asserted(self):
        # The first native NOP ends540, so requested451 releases at540. Input500
        # is still a real external observation under reset, not a past native step.
        config = self.config().replace("dsp-release 450", "dsp-release 451").replace(
            "dsp-at 3000", "dsp-at 500")
        result, trace = self.invoke("late-release", config, bytes((0xa7,)), instructions=1)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DSP-INPUT requested=500 actual=500", trace)
        self.assertIn("DSP-RELEASE requested=451 actual=540", trace)
        self.assertLess(trace.index("DSP-INPUT requested=500"), trace.index("DSP-RELEASE"))


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP reference DSP input tests: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
