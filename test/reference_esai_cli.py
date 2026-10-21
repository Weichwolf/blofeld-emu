# SPDX-License-Identifier: GPL-3.0-only
"""Exact external ESAI clocks and actual native serial output through the real CLI."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

from test_reference_cli import CONFIG, image, replay_sections
from reference_dsp_inputs_cli import check_pin, store

BINARY, FIXTURE, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:5])
ROOT = Path(__file__).resolve().parents[1]
START = 2000
HALF = 23
CYCLES = 258
FIRST = "10100101110000111001111111111111"
SECOND = "00010010001101000101011000000000"


def config(index=1, start=START, cycles=CYCLES, unknown=None):
    text = CONFIG.replace("operations 23", "operations 200")
    text += ("dsp-input-order input-before-dsp 16\n"
             f"dsp-initial esai {index} unknown unknown low reference-reset-edges\n")
    for edge in range(cycles * 2):
        level = "unknown" if edge == unknown else "high" if edge % 2 == 0 else "low"
        text += (f"dsp-at {start + edge * HALF} esai {index} unknown unknown {level} "
                 "reference-reset-edges\n")
    return text


def periodic(index=1, start=START, edges=CYCLES * 2):
    text = config(index=index, start=start, cycles=0)
    return text + f"dsp-clock esai {index} hckt {start} {HALF} {edges} reference-reset-edges\n"


def rows(trace):
    return [dict(field.split("=", 1) for field in line.split()[1:])
            for line in trace.splitlines() if line.startswith("DSP-ESAI ")]


class EsaiCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        manifest = json.loads((ROOT / "doc/downloads.json").read_text())
        expected = next(row["sha256"] for row in manifest["files"] if row["file"] == FIRMWARE.name)
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Changed official firmware archive")
        ARTIFACTS.mkdir(parents=True, exist_ok=True)

    def invoke(self, name, text, mode="esai1-refill", code=None, instructions=1000):
        folder = ARTIFACTS / name
        folder.mkdir(exist_ok=True)
        board, rom, nvm, trace = (folder / value for value in (
            "board.bfboard", "own-rom.bin", "mcu.bfnvm", "run.trace"))
        generated = subprocess.run([FIXTURE, mode, rom], capture_output=True, text=True,
                                   check=False, timeout=30)
        self.assertEqual(generated.returncode, 0, generated.stdout + generated.stderr)
        board.write_text(text)
        nvm.write_bytes(image(code if code is not None else bytes.fromhex("cf2000 3e")))
        trace.unlink(missing_ok=True)
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", rom, "--flash-in", nvm, "--trace", trace,
                                 "--instructions", str(instructions)], capture_output=True,
                                text=True, check=False, timeout=40)
        (folder / "run.log").write_text(result.stdout + result.stderr)
        return result, trace.read_text() if trace.exists() else ""

    def verify_frame(self, trace, index=1, refill=True, start=START):
        samples = []
        previous = 0
        observations = [row for row in rows(trace) if int(row["instance"]) == index]
        self.assertEqual(len(observations), CYCLES * 2)
        for edge, row in enumerate(observations):
            at = start + edge * HALF
            self.assertEqual((int(row["requested"]), int(row["actual"])), (at, at))
            self.assertEqual(int(row["instance"]), index)
            self.assertEqual((row["clock"], row["fs"], row["reset-edges"]), ("unknown", "unknown", "1"))
            self.assertEqual(row["hckt"], "high" if edge % 2 == 0 else "low")
            pins = int(row["clocks-levels"], 16)
            self.assertEqual((int(row["clocks-driven"], 16), int(row["clocks-known"], 16)), (3, 3))
            if not previous & 1 and pins & 1 and int(row["tx-driven"], 16):
                self.assertEqual((int(row["tx-driven"], 16), int(row["tx-known"], 16)), (1, 1))
                # Independent /2 toggle divider, falling launch/rising sample,
                # 32-bit slots and two slots per word-length-FS frame.
                self.assertEqual(at, start + (5 + 4 * len(samples)) * 2 * HALF)
                self.assertEqual(bool(pins & 2), len(samples) < 32)
                samples.append(str(int(row["tx-levels"], 16) & 1))
            previous = pins
        self.assertEqual("".join(samples), FIRST + (SECOND if refill else FIRST))

    def test_external_bit_clock_and_frame_snapshot_decode_without_generator(self):
        text = config(cycles=0).replace("unknown unknown low reference-reset-edges",
                                        "low low unknown raw")
        for edge in range(98):
            clock = "high" if edge % 2 == 0 else "low"
            fs = "high" if edge < 2 else "low"
            text += f"dsp-at {START + edge * HALF} esai 1 {clock} {fs} unknown raw\n"
        result, trace = self.invoke("external-clock-fs", text, "esai1-external")
        self.assertEqual(result.returncode, 0, result.stderr)
        samples = []
        observed = rows(trace)
        self.assertEqual(len(observed), 98)
        for edge, row in enumerate(observed):
            self.assertEqual(int(row["requested"]), START + edge * HALF)
            self.assertEqual(row["actual"], row["requested"])
            self.assertEqual((row["clocks-driven"], row["clocks-known"], row["hckt"],
                              row["reset-edges"]), ("0x00", "0x00", "unknown", "0"))
            if row["clock"] == "low" and int(row["tx-driven"], 16):
                self.assertEqual(int(row["actual"]), START + (3 + 2 * len(samples)) * HALF)
                self.assertEqual((row["tx-driven"], row["tx-known"]), ("0x01", "0x01"))
                samples.append(str(int(row["tx-levels"], 16) & 1))
        self.assertEqual("".join(samples), FIRST[:24] * 2)

    def test_native_refill_preserves_current_word_on_both_maps(self):
        for index in (0, 1):
            result, trace = self.invoke("refill-" + str(index), config(index), f"esai{index}-refill")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.verify_frame(trace, index)
            self.assertIn("DSP-INPUTS observed=516 configured=516", result.stdout)
            self.assertRegex(result.stdout, r"idle=[1-9][0-9]* ")

    def test_underrun_retains_known_data_and_midword_snapshot_replays(self):
        result, trace = self.invoke("midword-replay", config() + "checkpoint 35 2\n", "esai1")
        self.assertEqual(result.returncode, 0, result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.assertGreater(len(rows(prefix)), 2)
        self.assertLess(len(rows(prefix)), CYCLES * 2)
        self.verify_frame(prefix + suffixes[0], refill=False)
        # The snapshot falls inside the first24-bit payload, not before its frame.
        at = int(rows(prefix)[-1]["actual"])
        self.assertGreater(at, START + 5 * 2 * HALF)
        self.assertLess(at, START + (5 + 4 * 24) * 2 * HALF)

    def test_periodic_clock_matches_literal_events_and_restores_its_cursor(self):
        result, literal = self.invoke("literal-reference", config())
        self.assertEqual(result.returncode, 0, result.stderr)
        result, generated = self.invoke("periodic-reference", periodic())
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(generated, literal)
        self.verify_frame(generated)
        self.assertIn("DSP-ESAI-CLOCK instance=1 observed=516 configured=516", result.stdout)
        for operation, events_remain in ((35, True), (180, False)):
            result, trace = self.invoke("periodic-replay-" + str(operation),
                                        periodic() + f"checkpoint {operation} 2\n")
            self.assertEqual(result.returncode, 0, result.stderr)
            prefix, suffixes = replay_sections(trace)
            self.assertEqual(len(suffixes), 3)
            self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
            self.assertEqual(bool(rows(suffixes[0])), events_remain)
            self.verify_frame(prefix + suffixes[0])

    def test_compact_long_clock_does_not_expand_the_event_file(self):
        text = periodic(edges=1000000)
        self.assertLess(len(text), 512)
        result, trace = self.invoke("compact-large-span", text)
        self.assertEqual(result.returncode, 0, result.stderr)
        observed = rows(trace)
        self.assertGreater(len(observed), CYCLES * 2)
        self.assertLess(len(observed), 1000)
        self.assertIn(f"DSP-ESAI-CLOCK instance=1 observed={len(observed)} configured=1000000",
                      result.stdout)
        self.assertEqual([int(row["actual"]) for row in observed],
                         [START + i * HALF for i in range(len(observed))])

    def test_periodic_collisions_fail_at_the_actual_due_boundary(self):
        for name, extra in (("static", f"dsp-at {START} tio 0 high raw\n"),
                            ("second-clock", f"dsp-clock esai 0 hckt {START} 31 2 reference-reset-edges\n")):
            result, trace = self.invoke("periodic-collision-" + name, periodic() + extra)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn(f"Simultaneous reference DSP input sources at={START}", result.stderr)
            self.assertNotIn("DSP-ESAI requested=", trace)
            self.assertNotIn("DSP-INPUT requested=", trace)
            self.assertIn("DSP-ESAI-CLOCK instance=1 observed=0 configured=516", result.stderr)
        # A static unknown HCKT snapshot follows151 real periodic observations.
        result, trace = self.invoke("periodic-fault-replay", periodic() +
                                    "dsp-at 5451 esai 1 unknown unknown unknown reference-reset-edges\n"
                                    "checkpoint 35 2\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP ESAI HCKT", result.stderr)
        self.assertIn("DSP-ESAI-CLOCK instance=1 observed=151 configured=516", result.stderr)
        self.assertIn("DSP-INPUTS observed=0 configured=1", result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.assertEqual(len(rows(prefix + suffixes[0])), 151)

    def test_both_periodic_instances_and_tio_share_one_ordered_source(self):
        text = periodic() + ("dsp-clock esai 0 hckt 2001 29 7 reference-reset-edges\n"
                             "dsp-initial esai 0 unknown unknown low reference-reset-edges\n"
                             "dsp-at 1950 tio 0 high raw\n")
        result, trace = self.invoke("two-periodic-sources", text)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.verify_frame(trace)
        inactive = [row for row in rows(trace) if row["instance"] == "0"]
        self.assertEqual([int(row["actual"]) for row in inactive], [2001 + i * 29 for i in range(7)])
        self.assertTrue(all(row["tx-driven"] == "0x00" and row["clocks-driven"] == "0x00"
                            for row in inactive))
        self.assertIn("DSP-ESAI-CLOCK instance=0 observed=7 configured=7", result.stdout)
        self.assertIn("DSP-ESAI-CLOCK instance=1 observed=516 configured=516", result.stdout)
        self.assertIn("DSP-INPUTS observed=1 configured=1", result.stdout)

    def test_periodic_span_validation_and_actual_release(self):
        valid = periodic()
        row = f"dsp-clock esai 1 hckt {START} {HALF} 516 reference-reset-edges\n"
        invalid = [row.replace(f"{HALF} 516", "0 516"), row.replace("516", "0"),
                   row.replace(str(START), "18446744073709551615"), row.replace("hckt", "sckt"),
                   row.replace("esai 1", "esai 2"), row.replace("reference-reset-edges", "raw"),
                   row.replace(str(START), "100")]
        for index, wrong in enumerate(invalid):
            result, trace = self.invoke("invalid-periodic-" + str(index), valid.replace(row, wrong))
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")
        result, trace = self.invoke("duplicate-periodic", valid + row)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(trace, "")
        # The last observation may equal UINT64_MAX; the next step must not overflow.
        for name, clock, observed in (("maximum-first", "18446744073709551615 1 1", 0),
                                      ("maximum-last", "2000 18446744073709549615 2", 1)):
            boundary = valid.replace(f"{START} {HALF} 516", clock)
            result, _ = self.invoke(name, boundary)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(f"DSP-ESAI-CLOCK instance=1 observed={observed} configured=" +
                          ("1" if observed == 0 else "2"), result.stdout)
        text = periodic(start=500, edges=2).replace("dsp-release 450", "dsp-release 451")
        result, trace = self.invoke("late-periodic", text)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("actual DSP release", result.stderr)
        self.assertNotIn("DSP-ESAI requested=", trace)

    def test_unknown_hckt_repeats_strict_failure_after_restore(self):
        result, trace = self.invoke("unknown-replay", config(unknown=150) + "checkpoint 35 2\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown DSP ESAI HCKT at Y:0xffff96", result.stderr)
        self.assertRegex(result.stderr, r"ESAI input write index=1 mcu-pc=0x[0-9a-f]+ dsp-pc=0x[0-9a-f]+")
        self.assertIn("DSP-INPUTS observed=150 configured=516", result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.assertEqual(len(rows(prefix + suffixes[0])), 150)

    def test_ties_keep_both_explicit_completion_orders_and_default_rejection(self):
        for order, before in (("input-before-dsp", True), ("dsp-before-input", False)):
            text = config(start=2115, cycles=2).replace("input-before-dsp", order)
            result, trace = self.invoke("tie-" + order, text, "esai1")
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-ESAI requested=2115 actual=2115")
            completed = trace.index("DSP end=2115 ")
            self.assertEqual(source < completed, before)
        text = config(start=2115, cycles=2).replace("input-before-dsp", "reject")
        result, trace = self.invoke("tie-reject", text, "esai1")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("simultaneous", result.stderr)
        self.assertNotIn("DSP-ESAI requested=2115", trace)

    def test_refill_tied_to_slot_start_changes_actual_serial_payload(self):
        # Six MOVEPs cost12 native units,154 NOPs cost154 and refill costs2.
        # Epoch450 +168*45 =8010. HCKT cycle132 starts slot1 at1984+131*46.
        for order, new_word in (("dsp-before-input", True), ("input-before-dsp", False)):
            text = config(start=1984).replace("input-before-dsp", order)
            result, trace = self.invoke("payload-tie-" + order, text, "esai1-tied-refill")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.verify_frame(trace, refill=new_word, start=1984)
            source = trace.index("DSP-ESAI requested=8010 actual=8010")
            refill = trace.index("DSP end=8010 ")
            self.assertEqual(refill < source, new_word)

    def test_shared_mcu_write_and_mixed_tio_cursor(self):
        code = store(0x27c, 1) + check_pin(1) + store(1, 2) + check_pin(1)
        text = config(start=1710, cycles=1) + "dsp-at 1760 tio 0 high raw\n"
        text += "shi-hreq U 01 direct\ninitial drive U 3e 00 00\n"
        for order, before in (("dsp-before-mcu", True), ("mcu-before-dsp", False)):
            result, trace = self.invoke("shared-" + order, text.replace(
                "ordering dsp-before-mcu", "ordering " + order), "esai1", code, 10)
            self.assertEqual(result.returncode, 0, result.stderr)
            source = trace.index("DSP-ESAI requested=1710 actual=1710")
            write = trace.index("WRITE cycle=19 address=0x0001")
            self.assertEqual(source < write, before)
            self.assertIn("DSP-INPUT requested=1760 actual=1760 tio=0 level=high", trace)
            self.assertIn("DSP-INPUTS observed=3 configured=3", result.stdout)
        result, trace = self.invoke("shared-reject", text.replace(
            "ordering dsp-before-mcu", "ordering reject"), "esai1", code, 10)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("simultaneous", result.stderr)
        self.assertNotIn("DSP-ESAI requested=1710", trace)

    def test_reset_mux_timing_and_budget_guards_remain_strict(self):
        cases = [(config(), "esai1-active", "Unsupported/active DSP ESAI configuration"),
                 (config(), "esai1-missing-hckt", "requires HCKT/FST peripheral mux"),
                 (config().replace("reference-reset-edges", "raw"), "esai1", "timing"),
                 (config().replace("input-before-dsp 16", "input-before-dsp 1"),
                  "esai1", "event budget exhausted")]
        for index, (text, mode, reason) in enumerate(cases):
            result, _ = self.invoke("guard-" + str(index), text, mode)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn(reason, result.stderr)

    def test_invalid_esai_inputs_fail_before_trace_creation(self):
        text = config(cycles=1)
        cases = [text.replace("esai 1", "esai 2"), text.replace("esai 1", "esai -1"),
                 text.replace("unknown unknown low", "unknown unknown guessed"),
                 text.replace("reference-reset-edges", "synchronized"),
                 text.replace("dsp-input-order input-before-dsp 16\n", ""),
                 text + "dsp-at 2000 tio 0 high raw\n",
                 text.replace("unknown unknown low", "unknown low"),
                 text.replace("dsp-initial esai", "dsp-initial guessed")]
        for index, scenario in enumerate(cases):
            result, trace = self.invoke("invalid-" + str(index), scenario)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP ESAI CLI regression: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
