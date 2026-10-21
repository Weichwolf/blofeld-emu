# SPDX-License-Identifier: GPL-3.0-only
"""Actual headless process, own HC12 USB initialization/IRQs and loopback wire replies."""

import binascii
import hashlib
import json
from pathlib import Path
import re
import selectors
import socket
import struct
import subprocess
import sys
import time
import unittest

from test_reference_cli import CONFIG, ROM, image

BINARY, FIRMWARE, ARTIFACTS = map(Path, sys.argv[1:])
# Own structural descriptors. These do not implement or qualify a USB-MIDI class.
DEVICE = bytes.fromhex("120100020000004034127856020100000001")
CONFIGURATION = bytes.fromhex(
    "09023700020100801909040000000101000005241937a5"
    "090401000101030000070581020800000904010101ff12340007058202080000")
OPTIONS = ("--usbip-port", "0", "--usbip-after", "54000", "--usbip-period", "4500",
           "--usbip-probe-timeout", "5400000")
OPERATIONS = 2000000


def operand(opcode, address):
    """Own HC12 extended-address instruction encoding, independently specified."""
    return struct.pack(">BH", opcode, address)


def store_byte(address, value):
    # LDAA #value; STAA extended.
    return bytes((0x86, value)) + operand(0x7a, address)


def store_word(address, value):
    # LDD #value; STD extended.
    return operand(0xcc, value) + operand(0x7c, address)


def copy_handler(source):
    code = b"".join(operand(0xfc, source + offset) + operand(0x7c, 0x380 + offset)
                    for offset in range(0, 64, 2))
    # LDAA #DVALID|TCIE; LDAB saved wLength; STD UEPCSR0; RTI.
    return code + bytes.fromhex("8650") + operand(0xf6, 0x1202) + operand(0x7c, 0x314) + b"\x0b"


def own_image():
    data = bytearray(image(b""))

    def put(address, value):
        offset = 16 + address - 0x8000
        data[offset:offset + len(value)] = value

    # Own LDS; enable MCE, clear DISC while MEN=0, allow the declared PHY input.
    code = operand(0xcf, 0x2000) + store_byte(0x300, 0x40) + store_byte(0x301, 8)
    code += b"\xa7" * 100  # Own NOP delay, no host-supplied USB readiness.
    for bank, value in ((1, 0x02000000), (2, 0x020000d1), (3, 0x020000c2), (8, 0x21)):
        code += store_byte(0x30e, bank)
        code += store_word(0x310, value >> 16) + store_word(0x312, value & 0xffff)
    for address in (0x314, 0x316, 0x318, 0x31a):
        code += store_byte(address, 0x10)  # TCIE, no fabricated payload/DVALID.
    code += store_byte(0x305, 1) + store_byte(0x300, 0xc0)
    code += bytes.fromhex("10ef 3e 20fd")  # ANDCC #~I; WAI; BRA back to WAI.
    put(0x8000, code)
    # Own ISR reads SETUP while valid, saves wLength/type, then acknowledges SETUP.
    setup = (operand(0xfc, 0x338) + operand(0x7c, 0x1200)
             + operand(0xb6, 0x33e) + operand(0x7a, 0x1202)
             + operand(0xb6, 0x33b) + operand(0x7a, 0x1203)
             + store_byte(0x303, 1) + operand(0xb6, 0x1203)
             + bytes.fromhex("8101 2603") + operand(0x06, 0x9400) + operand(0x06, 0x9500))
    for address, value in ((0x9000, setup), (0x9100, DEVICE), (0x9180, CONFIGURATION),
                           (0x9200, store_byte(0x314, 0x30) + b"\x0b"),
                           (0x9240, store_byte(0x316, 0x30) + b"\x0b"),
                           (0x9400, copy_handler(0x9100)), (0x9500, copy_handler(0x9180)),
                           (0xffd2, bytes.fromhex("9000")), (0xffce, bytes.fromhex("9200")),
                           (0xffcc, bytes.fromhex("9240")), (0xfffe, bytes.fromhex("8000"))):
        put(address, value)
    data[-4:] = struct.pack("<I", binascii.crc32(data[:-4]))
    return data


def receive_exact(client, length):
    result = b""
    while len(result) < length:
        chunk = client.recv(length - len(result))
        if not chunk:
            raise ValueError(f"Unexpected TCP EOF after {len(result)}/{length} bytes")
        result += chunk
    return result


def listening(process):
    # Read bytes directly so buffered text reads cannot hide an already-ready banner.
    with selectors.DefaultSelector() as ready:
        ready.register(process.stdout, selectors.EVENT_READ)
        prefix = bytearray()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if not ready.select(timeout=1):
                continue
            byte = process.stdout.read(1)
            if not byte:
                raise ValueError(f"Headless exited before listen banner: {bytes(prefix)!r}")
            prefix.extend(byte)
            if byte == b"\n":
                match = re.search(rb"USBIP listening=127\.0\.0\.1:(\d+) ", prefix)
                if match:
                    return int(match[1]), bytes(prefix)
    raise ValueError("Headless listen banner exceeded its bounded wait")


class UsbCli(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        manifest = json.loads((Path(__file__).resolve().parents[1] / "doc/downloads.json").read_text())
        expected = next(row["sha256"] for row in manifest["files"] if row["file"] == FIRMWARE.name)
        if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
            raise ValueError("Original container identity mismatch")
        ARTIFACTS.mkdir(parents=True, exist_ok=True)

    def prepared(self, name, extra=""):
        root = ARTIFACTS / name
        root.mkdir(exist_ok=True)
        board, rom, flash, trace = [root / p for p in
                                    ("board.bfboard", "rom.bin", "state.bfnvm", "run.trace")]
        config = CONFIG.replace("operations 23", f"operations {OPERATIONS}")
        config = config.replace("dsp-release 450 low", "dsp-release 18446744073709551615 low")
        board.write_text(config + "initial usb-decoder 0c\nat 4500 phy-window 15\n" + extra)
        rom.write_bytes(ROM)
        flash.write_bytes(own_image())
        command = [str(BINARY), "--firmware", str(FIRMWARE), "--reference-board", str(board),
                   "--dsp-rom", str(rom), "--flash-in", str(flash), "--trace", str(trace),
                   "--instructions", str(OPERATIONS)]
        before = {p: p.read_bytes() for p in (board, rom, flash)}
        return root, trace, command, before

    def test_actual_cli_guest_descriptors_and_transfer(self):
        root, trace, command, before = self.prepared("wire")
        prefix = b""
        process = subprocess.Popen([*command, *OPTIONS], stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, bufsize=0)
        try:
            port, prefix = listening(process)
            deadline = time.monotonic() + 20
            while True:
                with socket.create_connection(("127.0.0.1", port), timeout=5) as listing:
                    listing.sendall(bytes.fromhex("0111800500000000"))
                    header = receive_exact(listing, 12)
                    self.assertEqual(header[:8], bytes.fromhex("0111000500000000"))
                    count, = struct.unpack(">I", header[8:])
                    self.assertIn(count, (0, 1))
                    if count:
                        record = receive_exact(listing, 312)
                        self.assertEqual(record[256:260], b"1-1\0")
                        self.assertEqual(record[300:306], bytes.fromhex("123456780102"))
                        self.assertEqual(record[309:], b"\0\x01\0")
                    self.assertEqual(listing.recv(1), b"")
                if count:
                    break
                self.assertLess(time.monotonic(), deadline)
            with socket.create_connection(("127.0.0.1", port), timeout=5) as client:
                client.sendall(bytes.fromhex("0111800300000000") + b"1-1" + bytes(29))
                imported = receive_exact(client, 320)
                self.assertEqual(imported[:8], bytes.fromhex("0111000300000000"))
                # Independent Linux USB/IP SUBMIT: IN ep0, GET_DESCRIPTOR device, 18 bytes.
                submit = struct.pack(">10I", 1, 17, 0x10001, 1, 0, 0, 18, 0, 0, 0)
                client.sendall(submit + bytes.fromhex("8006000100001200"))
                response = receive_exact(client, 66)
                expected = struct.pack(">12I", 3, 17, 0, 0, 0, 0, 18, 0, 0, 0, 0, 0) + DEVICE
                self.assertEqual(response, expected)
                client.shutdown(socket.SHUT_WR)
                self.assertEqual(client.recv(1), b"")
            stdout, stderr = process.communicate(timeout=20)
            self.assertEqual(process.returncode, 2, stderr.decode())
            self.assertIn(b"board-owned recovery required", stderr)
            self.assertIn(b"USBIP descriptors=ready", prefix + stdout)
            text = trace.read_text()
            self.assertIn("USBIP-START actual=54000", text)
            self.assertIn("USBIP-PUBLISH actual=", text)
            for vector in ("ffd2", "ffce", "ffcc"):
                self.assertGreaterEqual(text.count(f"IRQ vector=0x{vector}"), 4)
            self.assertRegex(text, r"WRITE cycle=\d+ address=0x030e .*mcu-pc=0x80")
            for path, original in before.items():
                self.assertEqual(path.read_bytes(), original)
            (root / "run.log").write_bytes(prefix + stdout + stderr)
        finally:
            if process.poll() is None:
                process.kill()
            stdout, stderr = process.communicate(timeout=5)
            (root / "cleanup.log").write_bytes(prefix + stdout + stderr)

    def test_options_reject_incomplete_or_invalid_contracts(self):
        for options in (("--usbip-port", "0"), ("--usbip-port", "65536"),
                        ("--usbip-period", "0"), ("--usbip-after", "-1"),
                        (*OPTIONS, "--inspect")):
            result = subprocess.run([BINARY, "--firmware", "absent.zip", *options],
                                    capture_output=True, text=True, timeout=5, check=False)
            self.assertEqual(result.returncode, 1)
            self.assertIn("USB/IP", result.stderr)

    def test_live_transport_rejects_automatic_replay_before_outputs(self):
        _, trace, command, before = self.prepared("replay", "checkpoint 100 2\n")
        trace.unlink(missing_ok=True)
        result = subprocess.run([*command, *OPTIONS], capture_output=True, text=True,
                                timeout=10, check=False)
        self.assertEqual(result.returncode, 1)
        self.assertIn("rejects automatic checkpoint replay", result.stderr)
        self.assertNotIn("USBIP listening=", result.stdout)
        self.assertFalse(trace.exists())
        for path, original in before.items():
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("Local original firmware container is required", file=sys.stderr)
        sys.exit(77)
    unittest.main(argv=[sys.argv[0]])
