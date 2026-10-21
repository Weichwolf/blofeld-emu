# SPDX-License-Identifier: GPL-3.0-only
"""Native standalone PCM histories to atomic WAV output, including replay/errors."""

import os
from pathlib import Path
import struct
import subprocess
import sys
import unittest
import wave

import reference_esai_cli as esai
from reference_pcm_cli import records
from test_reference_cli import image, replay_sections

OLD = b"previous complete audio reference"
PAYLOAD = bytes.fromhex("009fc3a5 00563412 00563412 00563412")


def config(index=1, edges=1032, half=5625, lane=0):
    return f"""blofeld-reference-board 1
clock 12000000 0 12000000 138240000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 115200 low
operations 400
initial drive B 16 02 00
initial drive E 03 03 00
dsp-input-order input-before-dsp 16
dsp-initial esai {index} unknown unknown low reference-reset-edges
dsp-clock esai {index} hckt 512000 {half} {edges} reference-reset-edges
pcm-receive esai {index} tx {lane} after 307200 rising 32 24 2 high
"""


class WaveCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        esai.EsaiCli.setUpClass()

    def invoke(self, name, text=None, index=1, rate=48000, maximum=4, extra=None, mode=None):
        folder = esai.ARTIFACTS / name
        folder.mkdir(exist_ok=True)
        board, rom, nvm, trace, audio = (folder / n for n in
                                       ("board.bfboard", "own-rom.bin", "mcu.bfnvm", "run.trace", "audio.wav"))
        made = subprocess.run([esai.FIXTURE, f"esai{index}-refill", rom], capture_output=True,
                              text=True, check=False, timeout=30)
        self.assertEqual(made.returncode, 0, made.stdout + made.stderr)
        board.write_text(text if text is not None else config(index=index))
        nvm.write_bytes(image(bytes.fromhex("cf2000 3e")))
        trace.unlink(missing_ok=True)
        if audio.is_symlink():
            audio.unlink()
        elif audio.exists():
            audio.unlink()
        audio.write_bytes(OLD)
        if mode == "symlink":
            target = folder / "target.bin"
            target.write_bytes(OLD)
            audio.unlink()
            audio.symlink_to(target.name)
        args = [esai.BINARY, "--firmware", esai.FIRMWARE, "--reference-board", board,
                "--dsp-rom", rom, "--flash-in", nvm, "--instructions", "1000"]
        if mode != "no-trace":
            args += ["--trace", "/dev/full" if mode == "trace-error" else trace]
        if extra is None:
            target = folder / "missing/audio.wav" if mode == "missing-parent" else audio
            if mode == "alias":
                audio.unlink()
                os.link(nvm, audio)
            args += ["--audio-out", target, "--audio-rate", str(rate),
                     "--audio-max-frames", str(maximum)]
        else:
            args += [arg.replace("{audio}", str(audio)) for arg in extra]
        result = subprocess.run(args, capture_output=True, text=True, check=False, timeout=45)
        (folder / "run.log").write_text(result.stdout + result.stderr)
        self.assertNotRegex(result.stdout + result.stderr,
                            r"AddressSanitizer|LeakSanitizer|UndefinedBehaviorSanitizer|runtime error:")
        return result, trace.read_text() if trace.exists() else "", audio

    def check_wave(self, path):
        data = path.read_bytes()
        self.assertEqual(len(data), 84)
        self.assertEqual(data[68:], PAYLOAD)
        self.assertEqual(struct.unpack_from("<H", data, 38)[0], 24)
        self.assertEqual(data[40:44], bytes(4))
        self.assertEqual(data[44:60], bytes.fromhex("01000000 0000 1000 8000 00aa00389b71"))
        with wave.open(str(path), "rb") as decoded:
            self.assertEqual((decoded.getnchannels(), decoded.getsampwidth(), decoded.getframerate(),
                              decoded.getnframes()), (2, 4, 48000, 2))
            self.assertEqual(decoded.readframes(3), PAYLOAD)
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)

    def test_both_native_instances_produce_exact_known_pcm_and_timing(self):
        for index in (0, 1):
            result, trace, audio = self.invoke("wave-map-" + str(index), index=index)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.check_wave(audio)
            self.assertIn("AUDIO recorded=2 limit=4", result.stdout)
            frames = records(trace, "PCM-FRAME")
            self.assertEqual([(int(r["first"]), int(r["end"])) for r in frames],
                             [(568250, 3403250), (3448250, 6283250)])

    def test_snapshots_keep_entire_history_without_recording_replay_suffixes_twice(self):
        result, _, baseline = self.invoke("wave-no-replay")
        self.assertEqual(result.returncode, 0, result.stderr)
        expected = baseline.read_bytes()
        for operation in (35, 200, 320):
            result, trace, audio = self.invoke("wave-replay-" + str(operation),
                                              config() + f"checkpoint {operation} 2\n")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(audio.read_bytes(), expected)
            prefix, suffixes = replay_sections(trace)
            self.assertEqual(len(suffixes), 3)
            self.assertTrue(all(suffix == suffixes[0] for suffix in suffixes))
            self.assertEqual(len(records(prefix + suffixes[0], "PCM-FRAME")), 2)
            self.assertIn("AUDIO recorded=2 limit=4", result.stdout)

    def test_capacity_failure_preserves_old_file_and_completed_prefix(self):
        result, trace, audio = self.invoke("wave-capacity", maximum=1)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("PCM recording frame budget exhausted", result.stderr)
        self.assertIn("AUDIO recorded=1 limit=1", result.stderr)
        self.assertEqual(audio.read_bytes(), OLD)
        self.assertEqual(len(records(trace, "PCM-FRAME")), 1)

    def test_wav_recording_and_replay_do_not_require_a_diagnostic_trace(self):
        result, trace, audio = self.invoke("wave-without-trace", config() + "checkpoint 200 2\n",
                                          mode="no-trace")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(trace, "")
        self.check_wave(audio)
        self.assertIn("AUDIO recorded=2 limit=4", result.stdout)

    def test_inconsistent_declared_rate_or_observed_clock_never_relabels_samples(self):
        for name, text, rate in (("wrong-rate", config(), 44100),
                                  ("wrong-clock", config(half=5626), 48000)):
            result, _, audio = self.invoke(name, text, rate=rate)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn("WAV rate", result.stderr)
            self.assertEqual(audio.read_bytes(), OLD)

    def test_partial_frame_and_unknown_data_do_not_publish_audio(self):
        for name, text, expected, reason in (("wave-truncated", config(edges=128), 1, "complete PCM frame"),
                                             ("wave-unknown", config(lane=1), 2, "Unknown or undriven payload")):
            result, _, audio = self.invoke(name, text)
            self.assertEqual(result.returncode, expected, result.stderr)
            self.assertIn(reason, result.stderr)
            self.assertEqual(audio.read_bytes(), OLD)

    def test_filesystem_failures_and_symlinks_are_visible(self):
        for mode in ("missing-parent", "symlink"):
            result, _, audio = self.invoke("wave-" + mode, mode=mode)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(audio.read_bytes(), OLD)
        self.assertTrue(audio.is_symlink())

    def test_audio_cannot_alias_an_execution_input(self):
        result, trace, audio = self.invoke("wave-alias", mode="alias")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("file collision", result.stderr)
        self.assertEqual(trace, "")
        self.assertEqual(audio.read_bytes(), (audio.parent / "mcu.bfnvm").read_bytes())

    def test_trace_write_failure_does_not_publish_a_successful_recording(self):
        self.assertTrue(Path("/dev/full").exists(), "Linux write-failure fixture must be available")
        result, _, audio = self.invoke("wave-trace-error", mode="trace-error")
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertRegex(result.stderr, r"iostream|trace output")
        self.assertEqual(audio.read_bytes(), OLD)

    def test_incomplete_audio_options_reject_before_trace_creation(self):
        cases = [["--audio-out", "{audio}"], ["--audio-rate", "48000"],
                 ["--audio-out", "{audio}", "--audio-rate", "0", "--audio-max-frames", "4"],
                 ["--audio-out", "{audio}", "--audio-rate", "48000", "--audio-max-frames", "0"],
                 ["--audio-out", "{audio}", "--audio-rate", "48000", "--audio-max-frames", "1048577"]]
        for index, args in enumerate(cases):
            result, trace, audio = self.invoke("wave-invalid-" + str(index), extra=args)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(trace, "")
            self.assertEqual(audio.read_bytes(), OLD)

    def test_missing_pcm_receiver_is_not_an_implicit_audio_source(self):
        text = "\n".join(line for line in config().splitlines() if not line.startswith("pcm-receive")) + "\n"
        result, trace, audio = self.invoke("wave-no-receiver", text)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn("requires pcm-receive", result.stderr)
        self.assertEqual(trace, "")
        self.assertEqual(audio.read_bytes(), OLD)


if __name__ == "__main__":
    if not esai.FIRMWARE.is_file():
        print("SKIP WAV CLI regression: private firmware absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
