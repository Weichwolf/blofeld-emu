# SPDX-License-Identifier: GPL-3.0-only
"""Explicit audio source parsing, native ESAI output, replay and visible refusals."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

from test_reference_cli import CONFIG, image, replay_sections

BINARY, FIXTURE, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:5])
ROOT = Path(__file__).resolve().parents[1]
EPOCH = 176400
HALF = 9375
FIRST = "10100101110000111001111111111111"


def config(index=1):
    text = CONFIG.replace("540000000", "211680000000")
    text = text.replace("dsp-release 450", f"dsp-release {EPOCH}")
    text = text.replace("operations 23", "operations 1200")
    return (text + "dsp-input-order input-before-dsp 64\n"
            f"dsp-audio-clock 11289600 {EPOCH} low {1 << index} reference-reset-edges\n")


def fields(line):
    return dict(field.split("=", 1) for field in line.split()[1:])


class AudioClockCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        manifest = json.loads((ROOT / "doc/downloads.json").read_text())
        expected = next(row["sha256"] for row in manifest["files"] if row["file"] == FIRMWARE.name)
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Changed official reference archive")
        ARTIFACTS.mkdir(parents=True, exist_ok=True)

    def invoke(self, name, text, index=1):
        directory = ARTIFACTS / name
        directory.mkdir(exist_ok=True)
        board, rom, nvm, trace = (directory / value for value in (
            "board.bfboard", "own-rom.bin", "mcu.bfnvm", "run.trace"))
        generated = subprocess.run([FIXTURE, f"esai{index}", rom], capture_output=True,
                                   text=True, check=False, timeout=30)
        self.assertEqual(generated.returncode, 0, generated.stdout + generated.stderr)
        board.write_text(text)
        nvm.write_bytes(image(bytes.fromhex("cf2000 3e")))
        trace.unlink(missing_ok=True)
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", rom, "--flash-in", nvm, "--trace", trace,
                                 "--instructions", "1000"], capture_output=True, text=True,
                                check=False, timeout=60)
        (directory / "run.log").write_text(result.stdout + result.stderr)
        return result, trace.read_text() if trace.exists() else ""

    def verify(self, trace, index):
        clocks = [fields(line) for line in trace.splitlines()
                  if line.startswith("DSP-AUDIO-CLOCK ")]
        self.assertGreater(len(clocks), 1024)
        for edge, row in enumerate(clocks):
            self.assertEqual(int(row["requested"]), EPOCH + edge * HALF)
            self.assertEqual(row["actual"], row["requested"])
            self.assertEqual(int(row["baseline"]), int(edge == 0))
            self.assertEqual(int(row["level"]), edge % 2)
            self.assertEqual(int(row["destinations"], 16), 1 << index)
        serial = [fields(line) for line in trace.splitlines() if line.startswith("DSP-ESAI ")]
        self.assertEqual(len(serial), len(clocks))
        samples = []
        previous = False
        for row in serial:
            self.assertEqual(int(row["instance"]), index)
            level = bool(int(row["clocks-levels"], 16) & 1)
            if previous and not level and int(row["tx-driven"], 16):
                self.assertEqual(int(row["tx-known"], 16), 1)
                samples.append(str(int(row["tx-levels"], 16) & 1))
            previous = level
        self.assertGreater(len(samples), 128)
        expected = (FIRST * ((len(samples) + len(FIRST) - 1) // len(FIRST)))[:len(samples)]
        self.assertEqual("".join(samples), expected)

    def test_native_maps_and_checkpoint_source_replay(self):
        for index in (0, 1):
            result, trace = self.invoke(f"native-{index}", config(index), index)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.verify(trace, index)
        result, trace = self.invoke("replay", config() + "checkpoint 300 2\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        prefix, suffixes = replay_sections(trace)
        self.assertEqual(len(suffixes), 3)
        self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
        self.verify(prefix + suffixes[0], 1)

    def test_invalid_declared_sources_are_visible(self):
        base = config()
        cases = {
            "zero-frequency": (base.replace("11289600", "0"), "must be nonzero"),
            "unknown-level": (base.replace(" low 2 reference", " unknown 2 reference"),
                              "must be known"),
            "empty-route": (base.replace(" low 2 reference", " low 0 reference"),
                            "must be nonzero"),
            "bad-route": (base.replace(" low 2 reference", " low 8 reference"), "number"),
            "inexact-clock": (base.replace("211680000000", "540000000")
                              .replace(f"dsp-release {EPOCH}", "dsp-release 450")
                              .replace(f"11289600 {EPOCH}", "11289600 450"), "exact"),
            "missing-order": (base.replace("dsp-input-order input-before-dsp 64\n", ""),
                              "explicit dsp-input-order"),
            "duplicate": (base + f"dsp-audio-clock 11289600 {EPOCH} low 2 reference-reset-edges\n",
                          "Duplicate"),
            "overlap": (base + f"dsp-clock esai 1 hckt {EPOCH} {HALF} 2 reference-reset-edges\n",
                        "Conflicting"),
        }
        for name, (text, error) in cases.items():
            with self.subTest(name=name):
                result, _ = self.invoke(name, text)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn(error, result.stderr)

    def test_unrelated_earlier_input_does_not_starve_the_free_running_source(self):
        at = EPOCH + 1
        result, trace = self.invoke("interleaved-tio", config() + f"dsp-at {at} tio 0 high raw\n")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"DSP-INPUT requested={at} actual={at} tio=0 level=high", trace)
        self.verify(trace, 1)

    def test_independent_source_collision_is_not_silently_ordered(self):
        result, trace = self.invoke("collision", config() + f"dsp-at {EPOCH} tio 0 high raw\n")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn(f"Simultaneous reference DSP input sources at={EPOCH}", result.stderr)
        self.assertIn("mcu-pc=", result.stderr)
        self.assertIn("dsp-pc=", result.stderr)
        self.assertNotIn("DSP-AUDIO-CLOCK requested=", trace)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
