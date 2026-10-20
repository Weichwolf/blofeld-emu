# SPDX-License-Identifier: GPL-3.0-only
"""Native SHI output drivers through the actual MCU/DSP CLI and GPIO reads."""

import binascii
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import unittest

from test_reference_cli import CONFIG, ROM, WAIT, image, replay_sections

BINARY, FIXTURE, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:5])
GROUP = sys.argv[5] if len(sys.argv) == 6 else "all"
ROOT = Path(__file__).resolve().parents[1]
OUTBOUND = 0xa15c37
INBOUND = 0x123456


def store(address, value):
    # Own HC12 LDAA immediate / STAA extended.
    return bytes((0x86, value, 0x7a, address >> 8, address & 255))


def check_pin(address, expected):
    # Own native GPIO read, CMPA and conditional branch; mismatches access unmapped memory.
    return bytes((0xb6, address >> 8, address & 255, 0x81, expected, 0x27, 3, 0xb6, 0x30, 0))


def transfer(complete_read=False):
    # Native GPIO establishes SS/SCK/MOSI before releasing the explicit external drives.
    code = store(1, 2) + store(3, 0x16)
    code += check_pin(0x278, 1)  # Actual inverted HREQ must say ready before selection.
    code += store(0x25c, 1) + store(0x27c, 1)  # Enable real MCU receiver pulls.
    code += store(1, 0)
    for bit in range(23, -1, -1):
        mosi = 16 if INBOUND & (1 << bit) else 0
        code += store(1, mosi | 4)
        code += check_pin(0x258, (OUTBOUND >> bit) & 1)
        code += store(1, mosi)
    if complete_read:
        # Native mode-zero HREQ is driven but unqualified here, despite the receiver pull.
        code += bytes.fromhex("b60278")
    code += store(1, 2)
    code += check_pin(0x278, 0)  # Single receive slot is full; native HREQ is known inactive.
    return code


def irq_image(code):
    result = bytearray(image(code))
    result[16 + 0x7ff2:16 + 0x7ff4] = bytes.fromhex("a000")
    result[-4:] = struct.pack("<I", binascii.crc32(result[:-4]))
    return result


def echo_exchange(words, wire_words=None):
    """Own three-wire CPU program; expected MISO comes from the preceding input word."""
    code = store(1, 2) + store(3, 0x16) + store(0x25c, 1) + store(0x27c, 1)
    failures = []
    response = OUTBOUND
    for expected, outgoing in zip(words, words if wire_words is None else wire_words, strict=True):
        code += check_pin(0x278, 1) + store(1, 0)
        frame = []
        for bit in range(23, -1, -1):
            mosi = 16 if outgoing & (1 << bit) else 0
            code += store(1, mosi | 4)
            frame.append(0x9000 + len(code) + 7)
            code += check_pin(0x258, (response >> bit) & 1)
            code += store(1, mosi)
        code += store(1, 2) + check_pin(0x278, 1)
        failures.append(frame)
        response = expected
    code += check_pin(0x260, 1)
    return code, failures


def spi_words(trace):
    """Independent rising-edge GPIO decoder, including exact word lengths."""
    previous, bits, words = 2, [], []
    for raw in re.findall(r"^SPI .*?value=(0x[0-9a-f]+) ", trace, re.MULTILINE):
        value = int(raw, 16)
        if not value & 2 and not previous & 4 and value & 4:
            bits.append(1 if value & 16 else 0)
        if not previous & 2 and value & 2:
            assert len(bits) == 24
            word = 0
            for bit in bits:
                word = word * 2 + bit
            words.append(word)
            bits = []
        previous = value
    assert not bits
    return words


def echo_counts():
    """Manual-cycle ledger for the declared reference clocks and DSP-before-MCU ties."""
    # MCU: reset5, four LDAA1/STAA3 pairs, then three frames, final GPIO check.
    # Each frame: ready LDAA3/CMPA1/BEQ3, SS4, 24*(SCK4+check7+SCK4), SS4, ready7.
    frame_cycles = 7 + 4 + 24 * 15 + 4 + 7
    mcu_cycles = 5 + 16 + 3 * frame_cycles + 7
    # DSP56300FM native estimates: immediate MOVEP/MOVE2, JCLR4, register MOVEP1,
    # memory MOVE1, absolute JMP4. Data arrives on each last leading clock edge.
    cursor, polls = 450 + 8 * 45, 0
    for frame in range(3):
        capture = (5 + 16 + frame * frame_cycles + 7 + 4 + 23 * 15 + 4) * 90
        count = (capture - cursor) // (4 * 45) + 1
        polls += count  # A completion tied to capture still sees the preceding empty FIFO.
        cursor += (4 * count + 7) * 45
    polls += (mcu_cycles * 90 - cursor) // (4 * 45)
    return mcu_cycles, 4 + polls + 3 * 4, 8 + polls * 4 + 3 * 7


class ShiReturnCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        ARTIFACTS.mkdir(parents=True, exist_ok=True)
        cls.roms = {"disabled": ROM}
        for mode in ("transmit", "unknown-transmit", "delayed-request", "short-request", "echo"):
            path = ARTIFACTS / f"own-{mode}.bin"
            subprocess.run([FIXTURE, mode, path], check=True, capture_output=True, timeout=30)
            cls.roms[mode] = path.read_bytes()

    def run_cli(self, name, config, code, mode="transmit", instructions=1000, irq=False):
        board, rom = ARTIFACTS / f"{name}.bfboard", ARTIFACTS / f"{name}.bin"
        nvm, trace = ARTIFACTS / f"{name}.bfnvm", ARTIFACTS / f"{name}.trace"
        saved = ARTIFACTS / f"{name}-saved.bfnvm"
        trace.unlink(missing_ok=True)
        saved.unlink(missing_ok=True)
        board.write_text(config)
        rom.write_bytes(self.roms[mode])
        nvm.write_bytes(irq_image(code) if irq else image(code))
        result = subprocess.run([BINARY, "--firmware", FIRMWARE, "--reference-board", board,
                                 "--dsp-rom", rom, "--flash-in", nvm, "--flash-out", saved,
                                 "--trace", trace, "--instructions", str(instructions)],
                                capture_output=True, text=True, timeout=30, check=False)
        (ARTIFACTS / f"{name}.log").write_text(result.stdout + result.stderr)
        if result.returncode in (0, 2):
            self.assertEqual(saved.read_bytes(), nvm.read_bytes())
        return result, trace.read_text() if trace.exists() else ""

    @staticmethod
    def serial_config():
        return (CONFIG.replace("operations 23", "operations 1000") +
                "shi-hreq U 01 inverted\nshi-miso P 01 direct\n"
                "initial drive U 3e 00 00\ninitial drive P fe 00 00\n"
                "at 1170 drive B 00 00 00\n")

    def test_native_transmit_word_and_ready_transitions_reach_actual_gpio_reads(self):
        code = transfer()
        result, trace = self.run_cli("transmit", self.serial_config(), code,
                                     instructions=4 + 3 + 4 + 2 + 24 * 7 + 2 + 3)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("spi-writes=51 frames=1", result.stdout)
        self.assertIn(f"pc=0x{0x9000 + len(code):04x}", result.stdout)
        self.assertIn("signal=hreq driven=1 level=0", trace)
        self.assertIn("signal=hreq driven=1 level=1", trace)
        self.assertIn("signal=hreq driven=1 level=unknown", trace)
        self.assertIn("signal=miso driven=1 level=1", trace)
        self.assertIn("signal=miso driven=1 level=0", trace)
        self.assertIn("signal=miso driven=0 level=unknown", trace)

    def test_wrong_pin_expectation_takes_the_real_unmapped_failure_branch(self):
        config = CONFIG + "initial drive Q ff 00 00\n"
        result, _ = self.run_cli("wrong-pin-expectation", config, check_pin(0x260, 1),
                                 mode="disabled", instructions=4)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("0x3000", result.stderr)
        self.assertIn("pc=0x9007", result.stderr)

    def test_echo_replays_both_shift_directions_between_real_cpu_reads(self):
        base = self.serial_config() + "initial drive Q ff 00 00\nat 20000 drive Q ff 01 00\n"
        mcu_cycles, dsp_steps, dsp_units = echo_counts()
        self.assertEqual((mcu_cycles, dsp_steps, dsp_units), (1174, 593, 2337))
        for first in (0x123456, 0xfedcba):
            words = (first, 0x654321, 0x0f0f0f)
            code, _ = echo_exchange(words)
            instructions = 8 + 178 * len(words) + 3
            result, baseline = self.run_cli(f"echo-{first:x}", base, code, mode="echo",
                                             instructions=instructions)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(spi_words(baseline), list(words))
            self.assertIn("spi-writes=151 frames=3", result.stdout)
            self.assertIn("r4=259 lc=0", result.stdout)
            self.assertIn(f"instructions=545 operations=545 interrupts=0 cycles={mcu_cycles} "
                          f"idle=0 ticks={mcu_cycles * 10}", result.stdout)
            self.assertIn(f"completed-steps={dsp_steps} units={dsp_units} ticks={dsp_units * 45}",
                          result.stdout)
            self.assertIn(f"pc=0x{0x9000 + len(code):04x}", result.stdout)
            for point, frame, high in ((71, 0, True), (72, 0, True), (76, 0, False),
                                       (249, 1, True), (250, 1, True), (427, 2, True)):
                with self.subTest(first=first, checkpoint=point):
                    result, trace = self.run_cli(f"echo-{first:x}-cp{point}",
                                                 base + f"checkpoint {point} 2\n", code,
                                                 mode="echo", instructions=instructions)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertIn("r4=259 lc=0", result.stdout)
                    self.assertIn(f"cycles={mcu_cycles} idle=0 ticks={mcu_cycles * 10}", result.stdout)
                    self.assertIn(f"completed-steps={dsp_steps} units={dsp_units} "
                                  f"ticks={dsp_units * 45}", result.stdout)
                    prefix, branches = replay_sections(trace)
                    self.assertEqual(len(branches), 3)
                    self.assertEqual(branches, [branches[0]] * 3)
                    self.assertEqual(prefix + branches[0], baseline)
                    last = [row for row in prefix.splitlines() if row.startswith("SPI ")][-1]
                    value = int(re.search(r"value=(0x[0-9a-f]+)", last)[1], 16)
                    self.assertFalse(value & 2, last)
                    self.assertEqual(bool(value & 4), high, last)
                    self.assertIn(f"frames={frame}", last)
                    self.assertIn(f"operations={point} idle=0", prefix.splitlines()[-1])
                    self.assertEqual(prefix.count("INPUT requested=20000"), int(frame != 0))
                    self.assertEqual(branches[0].count("INPUT requested=20000"), int(frame == 0))

    def test_echo_changed_mosi_bit_changes_native_dsp_reply_and_replays_the_actual_fault(self):
        words = (0x123456, 0x654321, 0x0f0f0f)
        code, failures = echo_exchange(words, (words[0] ^ 0x800000, *words[1:]))
        config = (self.serial_config() + "initial drive Q ff 00 00\n"
                  "at 20000 drive Q ff 01 00\ncheckpoint 71 2\n")
        result, trace = self.run_cli("echo-changed-bit", config, code, mode="echo",
                                     instructions=8 + 178 * len(words) + 3)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("0x3000", result.stderr)
        self.assertIn(f"pc=0x{failures[1][0]:04x}", result.stderr)
        prefix, branches = replay_sections(trace)
        self.assertEqual(len(branches), 3)
        self.assertEqual(branches, [branches[0]] * 3)
        # The first native received word must alter the next frame's first MISO bit.
        self.assertIn("r4=257 lc=0", result.stderr)
        first_frame = prefix + branches[0]
        rows = first_frame.splitlines()
        last_release = next(i for i, row in enumerate(rows) if row.startswith("SPI ") and
                            "frames=1" in row)
        self.assertEqual(spi_words("\n".join(rows[:last_release + 1]) + "\n"),
                         [words[0] ^ 0x800000])

    def test_htx_completion_tied_to_select_respects_both_shared_write_orders(self):
        base = CONFIG.replace("operations 23", "operations 40")
        base = base.replace("dsp-release 450", "dsp-release 1170")
        base += "shi-miso P 01 direct\ninitial drive P fe 00 00\nat 1170 drive B 00 00 00\n"
        # Native HTX completes at1440, exactly when the MCU's select store completes.
        # Mode zero loads the prepared word at selection; a later HTX stays pending.
        code = store(1, 2) + store(3, 0x16) + store(1, 0) + check_pin(0x258, 1)
        for order in ("dsp-before-mcu", "mcu-before-dsp"):
            config = base.replace("ordering dsp-before-mcu", f"ordering {order}")
            result, trace = self.run_cli(f"select-tie-{order}", config, code, instructions=9)
            if order == "dsp-before-mcu":
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("signal=miso driven=1 level=1", trace)
            else:
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("Unknown MCU GPIO input at 0x0258 mask=0x01", result.stderr)
                self.assertIn("signal=miso driven=1 level=unknown", trace)
            self.assertIn("ties=1", result.stdout + result.stderr)
            self.assertIn("DSP end=1440", trace)
            self.assertIn("SPI end=1440", trace)
            self.assertEqual(trace.index("DSP end=1440") < trace.index("SPI end=1440"),
                             order == "dsp-before-mcu")

    def test_driven_unknown_transmit_cannot_be_replaced_by_receiver_pull(self):
        result, trace = self.run_cli("unknown-transmit", self.serial_config(), transfer(),
                                     mode="unknown-transmit")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown MCU GPIO input at 0x0258 mask=0x01", result.stderr)
        self.assertIn("signal=miso driven=1 level=unknown", trace)

    def test_unqualified_hreq_after_complete_word_stays_unknown_with_pull(self):
        result, trace = self.run_cli("complete-hreq", self.serial_config(), transfer(True))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("Unknown MCU GPIO input at 0x0278 mask=0x01", result.stderr)
        self.assertIn("signal=hreq driven=1 level=unknown", trace)

    def test_disabled_driver_is_high_impedance_and_native_pulls_resolve_it(self):
        for polarity in ("direct", "inverted"):
            config = CONFIG + f"shi-miso P 01 {polarity}\ninitial drive P fe 00 00\n"
            result, _ = self.run_cli(f"floating-{polarity}", config, bytes.fromhex("b60258"),
                                     mode="disabled", instructions=1)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertIn("Unknown MCU GPIO input at 0x0258 mask=0x01", result.stderr)
            for level in (0, 1):
                # PERP enables the pull; PPSP selects low (1) or high (0).
                code = store(0x25d, 0 if level else 1) + store(0x25c, 1) + check_pin(0x258, level)
                result, _ = self.run_cli(f"pull-{polarity}-{level}", config, code,
                                         mode="disabled", instructions=7)
                self.assertEqual(result.returncode, 0, result.stderr)

    def test_native_request_pulse_wakes_wai_through_real_external_irq(self):
        config = CONFIG.replace("operations 23", "operations 200")
        config = config.replace("initial drive E 03 03", "initial drive E 01 01")
        config += "shi-hreq E 02 direct\n"
        result, trace = self.run_cli("wai-irq", config, WAIT, mode="delayed-request",
                                     instructions=10, irq=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IRQ vector=0xfff2 stack-reused=1", trace)
        self.assertIn("pc=0x9010 d=0x1234 x=0x4567 y=0x6789 sp=0x2000", result.stdout)
        self.assertIn("interrupts=1", result.stdout)
        idle = re.search(r"cycles=\d+ idle=(\d+)", result.stdout)
        self.assertIsNotNone(idle)
        self.assertGreater(int(idle[1]), 0)
        self.assertLess(trace.index("signal=hreq driven=1 level=0"),
                        trace.index("IRQ vector=0xfff2"))

    def test_subperiod_request_pulse_latches_real_edge_irq_during_wai(self):
        config = CONFIG.replace("operations 23", "operations 200")
        config = config.replace("12000000 540000000", "60000000 540000000")
        config = config.replace("initial drive E 03 03", "initial drive E 01 01")
        config += "shi-hreq E 02 direct\n"
        # Own INTCR edge selection precedes the independently seeded foreground.
        code = WAIT[:3] + store(0x1e, 0xc0) + WAIT[3:]
        result, trace = self.run_cli("short-wai-irq", config, code, mode="short-request",
                                     instructions=12, irq=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IRQ vector=0xfff2 stack-reused=1", trace)
        self.assertIn("interrupts=1", result.stdout)
        self.assertIn("sp=0x2000", result.stdout)
        # Independent MCU:25 setup/reset +7 idle +6 wake +12 handler +1 NOP.
        self.assertIn("instructions=12 operations=20 interrupts=1 cycles=51 idle=7 ticks=510", result.stdout)
        # Independent DSP:257 NOP +3 MOVEP(2 each) +39*(NOP1+JMP4) +one NOP.
        self.assertIn("completed-steps=339 units=459 ticks=4131", result.stdout)
        events = [(int(ticks), int(driven), level) for ticks, driven, level in
                  re.findall(r"SHI-RETURN end=(\d+) signal=hreq driven=([01]) level=(\w+)", trace)]
        self.assertEqual(events, [(450, 0, "unknown"), (2799, 1, "0"), (2817, 0, "unknown")])
        # Both transitions occur strictly inside the same 90-common-tick MCU idle period.
        self.assertEqual(events[1][0] // 90, events[2][0] // 90)
        self.assertLess(events[1][0] % 90, events[2][0] % 90)
        self.assertLess(trace.index("SHI-RETURN end=2817"), trace.index("IRQ vector=0xfff2"))

    def test_checkpoint_replays_native_edge_irq_and_future_input_cursor(self):
        config = CONFIG.replace("operations 23", "operations 200")
        config = config.replace("12000000 540000000", "60000000 540000000")
        config = config.replace("initial drive E 03 03", "initial drive E 01 01")
        config += ("shi-hreq E 02 direct\ninitial drive Q ff 00 00\n"
                   "at 2799 drive Q ff 01 00\ncheckpoint 10 2\n")
        code = WAIT[:3] + store(0x1e, 0xc0) + WAIT[3:] + check_pin(0x260, 1)
        result, trace = self.run_cli("checkpoint-edge-input", config, code, mode="short-request",
                                     instructions=15, irq=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        _, branches = replay_sections(trace)
        self.assertEqual(len(branches), 3)
        self.assertEqual(branches, [branches[0]] * 3)
        for branch in branches:
            self.assertIn("SHI-RETURN end=2799 signal=hreq driven=1 level=0", branch)
            self.assertIn("SHI-RETURN end=2817 signal=hreq driven=0 level=unknown", branch)
            self.assertIn("INPUT requested=2799 actual=2880", branch)
            self.assertIn("IRQ vector=0xfff2 stack-reused=1", branch)
        self.assertIn("interrupts=1", result.stdout)
        self.assertIn("sp=0x2000", result.stdout)
        self.assertIn(f"pc=0x{0x9000 + len(code):04x}", result.stdout)

    def test_conflicting_drivers_feedback_and_invalid_masks_reject_before_execution(self):
        cases = ["shi-hreq B 02 direct\n", "shi-hreq U 80 direct\n",
                 "shi-hreq U 01 unknown\n", "shi-miso P 03 direct\n",
                 "shi-hreq P 01 direct\nshi-miso P 01 direct\n",
                 "shi-hreq P 01 direct\ninitial drive P 01 00 00\n",
                 "shi-hreq P 01 direct\nat 1000 drive P 00 00 01\n",
                 "shi-miso P 01 direct\nloopback U 01 P 01\n",
                 "shi-hreq U 01 inverted\nloopback E 04 B 02\n"]
        for index, extra in enumerate(cases):
            result, trace = self.run_cli(f"invalid-{index}", CONFIG + extra, WAIT)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertFalse(trace)

    def test_gpio_output_contention_is_reported_with_both_processor_contexts(self):
        config = CONFIG + "shi-hreq P 01 direct\ninitial drive P fe 00 00\n"
        # Establish a high MCU P0 output after native receive-ready drives HREQ low.
        code = bytes.fromhex("a7") * 12 + store(0x258, 1) + store(0x25a, 1)
        result, _ = self.run_cli("contention", config, code, instructions=16)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("SHI hreq return pad observation", result.stderr)
        self.assertIn("MCU GPIO output contention at 0x0258", result.stderr)
        self.assertIn("mcu-pc=", result.stderr)
        self.assertIn("dsp-pc=", result.stderr)


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP native SHI CLI: original firmware loader input is private")
        sys.exit(77)
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"] if row["file"] == FIRMWARE.name)
    if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
        raise ValueError("Original firmware identity mismatch")
    if GROUP not in ("all", "signals", "echo"):
        raise ValueError("Unknown SHI CLI test group")
    names = unittest.defaultTestLoader.getTestCaseNames(ShiReturnCli)
    selected = [f"ShiReturnCli.{name}" for name in names
                if GROUP == "all" or name.startswith("test_echo_") == (GROUP == "echo")]
    if not selected:
        raise ValueError("Empty SHI CLI test group")
    unittest.main(argv=[sys.argv[0]], defaultTest=selected, verbosity=2)
