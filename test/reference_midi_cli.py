# SPDX-License-Identifier: GPL-3.0-only
"""Run byte-file input through the actual headless MCU/SCI/IRQ path."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import unittest

from test_reference_cli import CONFIG, ROM, image

BINARY, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:])
MIDI = bytes((0xf8, 0x90, 60, 100, 0xf8, 0x80, 60, 0))
OPERATIONS = 24000
START = 54000
# Own MCU program: native SCI setup, CLI, WAI/BRA. ISR reads status/data,
# reports the exact received byte through the simulated PTJ latch, then RTI.
CODE = bytes.fromhex("cf2000 cc000c 7c00c8 8624 7a00cb 10ef 3e 20fd")
ISR = bytes.fromhex("b600cc b600cf 7a0260 0b")


def configuration(pin="PR2", strict=False):
    port, mask = ("R", "04") if pin == "PR2" else ("S", "10")
    text = CONFIG.replace("operations 23", f"operations {OPERATIONS}")
    text = text.replace("dsp-release 450 low", "dsp-release 18446744073709551615 low")
    text += f"initial drive {port} {mask} {mask} 00\nmidi-sci {pin} 6000000 after {START}\n"
    if not strict:
        text += "midi-transfer-reference 12 0\n"
    return text


def nvm(pin):
    code = CODE if pin == "PR2" else bytes.fromhex("8601 7a0257") + CODE
    # Rewrite the payload and recalculate the existing independently specified CRC container.
    import binascii
    import struct
    data = bytearray(image(code))
    data[16 + 0x2100:16 + 0x2100 + len(ISR)] = ISR
    data[16 + 0x7fd6:16 + 0x7fd8] = bytes.fromhex("a100")
    data[-4:] = struct.pack("<I", binascii.crc32(data[:-4]))
    return data


class MidiCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        manifest = json.loads((root / "doc/downloads.json").read_text())
        expected = next(row["sha256"] for row in manifest["files"]
                        if row["file"] == FIRMWARE.name)
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Original container identity mismatch")
        ARTIFACTS.mkdir(parents=True, exist_ok=True)

    def invoke(self, name, text=None, pin="PR2", source_option="--midi-file", source=None):
        root = ARTIFACTS / name
        root.mkdir(exist_ok=True)
        board, rom, flash, midi, trace = [root / p for p in
                                         ("board.bfboard", "rom.bin", "state.bfnvm",
                                          "bytes.bin", "run.trace")]
        board.write_text(text if text is not None else configuration(pin))
        rom.write_bytes(ROM)
        flash.write_bytes(nvm(pin))
        midi.write_bytes(MIDI)
        trace.unlink(missing_ok=True)
        before = {p: p.read_bytes() for p in (board, rom, flash, midi)}
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", rom, "--flash-in", flash, "--trace", trace,
                                 "--instructions", str(OPERATIONS), source_option,
                                 source or midi], capture_output=True, text=True,
                                timeout=40, check=False)
        (root / "run.log").write_text(result.stdout + result.stderr)
        for path, expected in before.items():
            self.assertEqual(path.read_bytes(), expected, f"Input changed: {path}")
        return result, trace.read_text() if trace.is_file() else ""

    def test_native_sci_receives_the_host_file_on_both_routes(self):
        for pin in ("PR2", "PS4"):
            with self.subTest(pin=pin):
                result, trace = self.invoke(pin, pin=pin)
                self.assertEqual(result.returncode, 0, result.stderr)
                values = re.findall(r"^WRITE cycle=\d+ address=0x0260 value=0x([0-9a-f]{2}) ",
                                    trace, re.M)
                self.assertEqual(bytes(int(value, 16) for value in values), MIDI)
                self.assertEqual(trace.count("IRQ vector=0xffd6 stack-reused=1"), len(MIDI))
                self.assertIn("MIDI accepted=8 queued=0 started=1", result.stdout)
                self.assertIn("held-reset=1", result.stdout)
                self.assertRegex(trace, r"MIDI-START requested=54000 actual=54000 bus-hz=6000000 "
                                       r"reference-sbr=12 reference-delay=0")
                rows = re.findall(r"^MIDI-INPUT actual=(\d+) operation=(\d+) bytes=([0-9a-f]+)$",
                                  trace, re.M)
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0][2], MIDI.hex())

    def test_default_strict_transfer_frontier_is_not_hidden(self):
        result, trace = self.invoke("strict", text=configuration(strict=True))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unverified SCI stop-to-data flag transfer", result.stderr)
        self.assertNotIn("address=0x0260", trace)
        self.assertIn("MIDI accepted=8", result.stderr)

    def test_device_mode_does_not_accept_a_file(self):
        result, _ = self.invoke("not-a-device", source_option="--midi-device")
        self.assertEqual(result.returncode, 2)
        self.assertIn("character device", result.stderr)

    def test_live_or_file_reread_cannot_fake_automatic_replay(self):
        result, trace = self.invoke("replay", text=configuration() + "checkpoint 100 2\n")
        self.assertEqual(result.returncode, 1)
        self.assertIn("rejects automatic checkpoint replay", result.stderr)
        self.assertEqual(trace, "")

    def test_invalid_or_conflicting_routes_fail_before_trace_creation(self):
        cases = [configuration().replace("midi-sci PR2", "midi-sci UNKNOWN"),
                 configuration().replace("6000000 after", "1 after"),
                 configuration() + "at 60000 drive R 04 04 00\n",
                 configuration() + "loopback U 01 R 04\n",
                 configuration() + "shi-hreq R 04 direct\n",
                 configuration().replace("initial drive R 04 04", "initial drive R 04 00"),
                 configuration().replace("reference 12 0", "reference 12 73"),
                 configuration().replace(f"midi-sci PR2 6000000 after {START}\n", "")]
        for index, text in enumerate(cases):
            with self.subTest(index=index):
                result, trace = self.invoke(f"invalid-{index}", text=text)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(trace, "")

    def test_native_clock_mismatch_fails_with_context(self):
        text = configuration().replace("6000000 after", "30000000 after")
        result, _ = self.invoke("rate", text=text)
        self.assertEqual(result.returncode, 2)
        self.assertIn("fixed bus rate", result.stderr)
        self.assertIn("address=0x0268", result.stderr)
        self.assertIn("pc=", result.stderr)

    def test_source_output_aliases_fail_before_loading_or_truncating_inputs(self):
        root = ARTIFACTS / "collisions"
        root.mkdir(exist_ok=True)
        source = root / "bytes.bin"
        source.write_bytes(MIDI)
        common = [BINARY, "--firmware", "absent.zip", "--reference-board", "absent.bfboard",
                  "--dsp-rom", "absent.rom", "--midi-file", source]
        outputs = [("--trace", "out.trace", []), ("--flash-out", "out.bfnvm", []),
                   ("--audio-out", "out.wav", ["--audio-rate", "48000",
                                               "--audio-max-frames", "1"])]
        for flag, name, extra in outputs:
            for symbolic in (False, True):
                alias = root / name
                alias.unlink(missing_ok=True)
                if symbolic:
                    alias.symlink_to(source.resolve())
                else:
                    alias.hardlink_to(source)
                result = subprocess.run(common + [flag, alias] + extra, capture_output=True,
                                        text=True, timeout=10, check=False)
                self.assertEqual(result.returncode, 1)
                self.assertIn("collision", result.stderr)
                self.assertEqual(source.read_bytes(), MIDI)
                alias.unlink()


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP host MIDI CLI: private original firmware container absent")
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
