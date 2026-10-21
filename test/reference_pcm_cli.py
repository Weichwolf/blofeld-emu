# SPDX-License-Identifier: GPL-3.0-only
"""Actual standalone ESAI pins to signed PCM, including native changes and replay."""

import sys
import unittest

import reference_esai_cli as esai
from test_reference_cli import replay_sections


def route(index=1, after=1200, edge="rising", slot=32, lane=0):
    return f"pcm-receive esai {index} tx {lane} after {after} {edge} {slot} 24 2 high\n"


def records(trace, kind):
    return [dict(part.split("=", 1) for part in line.split()[1:])
            for line in trace.splitlines() if line.startswith(kind + " ")]


class PcmCli(unittest.TestCase):
    invoke = esai.EsaiCli.invoke

    @classmethod
    def setUpClass(cls):
        esai.EsaiCli.setUpClass()

    def check_frame(self, trace, samples=(-5913697, 1193046), first=2230, end=13822, index=1):
        frames = records(trace, "PCM-FRAME")
        self.assertEqual(len(frames), 1)
        row = frames[0]
        self.assertEqual((int(row["index"]), int(row["first"]), int(row["end"]),
                          int(row["channels"])), (index, first, end, 2))
        self.assertEqual(tuple(map(int, row["samples"].split(","))), samples)

    def test_both_maps_decode_real_native_payload_without_changing_cpu_trace(self):
        for index in (0, 1):
            text = esai.periodic(index=index)
            plain, baseline = self.invoke(f"plain-{index}", text, f"esai{index}-refill")
            self.assertEqual(plain.returncode, 0, plain.stderr)
            result, trace = self.invoke(f"pcm-{index}", text + route(index), f"esai{index}-refill")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.check_frame(trace)
            self.assertIn("PCM frames=1 started=1 locked=1 faulted=0", result.stdout)
            filtered = "".join(line for line in trace.splitlines(keepends=True)
                               if not line.startswith("PCM-"))
            self.assertEqual(filtered, baseline)
            self.assertEqual(result.stdout.replace("PCM frames=1 started=1 locked=1 faulted=0\n", ""),
                             plain.stdout)
            pins = records(trace, "PCM-PINS")
            self.assertTrue(any(row["source"] == "native" for row in pins))
            self.assertEqual(sum(row["source"] == "external" for row in pins), 516)
            self.assertEqual((pins[0]["end"], pins[0]["clock"], pins[0]["fs"]), ("1215", "0", "0"))

    def test_joint_partial_and_completed_frame_snapshots_replay_exact_pcm(self):
        for operation in (35, 180):
            text = esai.periodic() + route() + f"checkpoint {operation} 2\n"
            result, trace = self.invoke(f"pcm-snapshot-{operation}", text)
            self.assertEqual(result.returncode, 0, result.stderr)
            prefix, suffixes = replay_sections(trace)
            self.assertEqual(len(suffixes), 3)
            self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
            self.check_frame(prefix + suffixes[0])
            self.assertEqual(bool(records(prefix, "PCM-FRAME")), operation == 180)

    def test_pcm_observation_does_not_change_existing_loopback_behavior(self):
        text = esai.periodic() + "loopback B 02 E 04\n"
        plain, baseline = self.invoke("pcm-loopback-plain", text)
        result, trace = self.invoke("pcm-loopback-enabled", text + route())
        self.assertEqual(plain.returncode, 0, plain.stderr)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.check_frame(trace)
        self.assertEqual("".join(line for line in trace.splitlines(keepends=True)
                                  if not line.startswith("PCM-")), baseline)

    def test_refill_ties_preserve_actual_payload_and_causal_order(self):
        for order, new in (("input-before-dsp", False), ("dsp-before-input", True)):
            text = esai.periodic(start=1984).replace("input-before-dsp", order) + route()
            result, trace = self.invoke("pcm-tie-" + order, text, "esai1-tied-refill")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.check_frame(trace, samples=(-5913697, 1193046 if new else -5913697),
                             first=2214, end=13806)
            tied = [row for row in records(trace, "PCM-PINS") if row["end"] == "8010"]
            self.assertEqual([row["order"] for row in tied], ["0", "1"])
            self.assertEqual([row["source"] for row in tied],
                             ["native", "external"] if new else ["external", "native"])

    def test_external_clock_and_generated_frame_use_actual_resolved_levels(self):
        text = esai.config(cycles=0).replace("unknown unknown low reference-reset-edges",
                                            "low unknown unknown reference-reset-edges")
        for edge in range(128):
            clock = "high" if edge % 2 == 0 else "low"
            text += (f"dsp-at {2000 + edge * 23} esai 1 {clock} unknown unknown "
                     "reference-reset-edges\n")
        result, trace = self.invoke("pcm-external", text + route(edge="falling"),
                                    "esai1-external-sckt")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.check_frame(trace, samples=(-5913697, -5913697), first=2023, end=4921)
        pins = records(trace, "PCM-PINS")
        self.assertEqual((pins[0]["clock"], pins[0]["fs"]), ("0", "0"))

    def test_unqualified_external_word_frame_sync_still_fails_in_native_hardware(self):
        text = esai.config(cycles=0).replace("unknown unknown low reference-reset-edges",
                                            "low low unknown raw")
        result, trace = self.invoke("pcm-unqualified-word-fs", text + route(edge="falling"),
                                    "esai1-external-word")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unqualified DSP ESAI word-length frame sync at Y:0xffff95", result.stderr)
        self.assertEqual(records(trace, "PCM-PINS"), [])
        self.assertEqual(records(trace, "PCM-FRAME"), [])

    def test_late_join_and_known_underrun_never_fill_a_partial_frame(self):
        text = esai.periodic(edges=1032).replace("operations 200", "operations 400")
        result, trace = self.invoke("pcm-two-frames", text + route())
        self.assertEqual(result.returncode, 0, result.stderr)
        frames = records(trace, "PCM-FRAME")
        self.assertEqual(len(frames), 2)
        self.check_frame("PCM-FRAME " + " ".join(f"{k}={v}" for k, v in frames[0].items()))
        self.check_frame("PCM-FRAME " + " ".join(f"{k}={v}" for k, v in frames[1].items()),
                         samples=(1193046, 1193046), first=14006, end=25598, index=2)
        result, trace = self.invoke("pcm-late-join", text + route(after=3000))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.check_frame(trace, samples=(1193046, 1193046), first=14006, end=25598)

    def test_missing_lane_faults_with_context_and_replays_from_a_trusted_prefix(self):
        text = esai.periodic() + route(lane=1) + "checkpoint 12 2\n"
        result, trace = self.invoke("pcm-unknown-lane", text)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown or undriven payload bit", result.stderr)
        self.assertIn("pin read clock=Y:0xffff96 data=Y:0xffff81 at=2230", result.stderr)
        self.assertRegex(result.stderr, r"mcu-pc=0x[0-9a-f]+ dsp-pc=0x[0-9a-f]+")
        self.assertIn("PCM frames=0 started=1 locked=1 faulted=1", result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.assertEqual(records(prefix + suffixes[0], "PCM-FRAME"), [])

    def test_native_mux_change_is_observed_before_the_next_external_edge(self):
        result, trace = self.invoke("pcm-native-drop-clock", esai.periodic() + route(),
                                    "esai1-drop-sckt")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("PCM SCKT/FST peripheral mux is absent", result.stderr)
        self.assertIn("at=3960 source=native", result.stderr)
        self.assertIn("DSP end=3960 ", trace)
        self.assertLess(int(records(trace, "DSP-ESAI")[-1]["actual"]), 3960)
        self.assertEqual(records(trace, "PCM-FRAME"), [])

    def test_truncated_or_future_capture_has_no_manufactured_frame(self):
        for name, text, started in (("truncated", esai.periodic(edges=128) + route(), 1),
                                    ("future", esai.periodic() + route(after=18446744073709551615), 0)):
            result, trace = self.invoke("pcm-" + name, text)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(records(trace, "PCM-FRAME"), [])
            self.assertIn(f"PCM frames=0 started={started}", result.stdout)

    def test_invalid_pcm_contracts_reject_before_any_execution_trace(self):
        valid = route()
        wrong = [valid.replace("esai 1", "esai 2"), valid.replace("tx 0", "tx 6"),
                 valid.replace("after 1200", "after 0"), valid.replace("rising", "guessed"),
                 valid.replace("32 24", "0 24"), valid.replace("32 24", "16 24"),
                 valid.replace("24 2 high", "24 1 high"), valid.replace("high", "unknown"),
                 valid + valid]
        for index, row in enumerate(wrong):
            result, trace = self.invoke("pcm-invalid-" + str(index), esai.periodic() + row)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")


if __name__ == "__main__":
    if not esai.FIRMWARE.is_file():
        print("SKIP PCM CLI regression: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
