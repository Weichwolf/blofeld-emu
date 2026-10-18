# SPDX-License-Identifier: GPL-3.0-only
"""Offline regressions for firmware framing, corruption and recovered objects."""

import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from inspect_firmware import decode_block, load_input, read_smf, split_blocks, unpack_payload
from extract_runtime import dsp_records, extract, hc12_object


class FirmwareInspectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = ROOT / "doc/Blofeld_Firmware_V1.25.zip"
        if not cls.path.exists():
            raise unittest.SkipTest("Official 1.25 archive is a local reference")
        cls.midi, _ = load_input(cls.path)
        cls.messages, cls.smf = read_smf(cls.midi)
        cls.blocks = split_blocks(cls.messages)

    def test_reference_transport_and_helper_identity(self):
        self.assertEqual(len(self.messages), 4847)
        self.assertEqual(self.smf["events"]["f0"], 4847)
        self.assertEqual(len(self.blocks), 6)
        _, dsp_helper, fields = decode_block(self.blocks[1])
        self.assertEqual(fields["address_words"], 0x2e00)
        # Pin identity without embedding any manufacturer program bytes.
        self.assertEqual(hashlib.sha256(dsp_helper).hexdigest(),
                         "a06e07c7f0b2b5c098e187a799d52c4ebb8fc951c508284470e8c974c8d6099f")

    def test_truncated_smf_is_rejected(self):
        for cut in [0, 10, 21, len(self.midi) - 1]:
            with self.subTest(cut=cut), self.assertRaises(ValueError):
                read_smf(self.midi[:cut])

    def test_changed_payload_is_rejected(self):
        changed = self.messages.copy()
        packet = bytearray(changed[100])
        packet[9] ^= 1
        changed[100] = bytes(packet)
        with self.assertRaisesRegex(ValueError, "XOR"):
            split_blocks(changed)

    def test_missing_packet_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "sequence"):
            split_blocks(self.messages[:100] + self.messages[101:])

    def test_larger_declared_block_is_rejected(self):
        block = dict(self.blocks[1])
        header = bytearray(block["header"])
        header[6] += 1
        block["header"] = bytes(header)
        with self.assertRaisesRegex(ValueError, "length"):
            decode_block(block)

    def test_packed_group_boundaries_and_msb_order(self):
        self.assertEqual(unpack_payload(bytes.fromhex("40 01 02 03 04 05 06 07")),
                         bytes.fromhex("81 02 03 04 05 06 07"))
        self.assertEqual(unpack_payload(bytes.fromhex("01 01 02 03 04 05 06 07")),
                         bytes.fromhex("01 02 03 04 05 06 87"))
        with self.assertRaises(ValueError):
            unpack_payload(bytes(7))

    def test_object_checksum_and_dsp_end_marker(self):
        _, data, _ = decode_block(self.blocks[4])
        helper, end = hc12_object(data)
        self.assertEqual((len(helper), end), (632, 639))
        damaged = bytearray(data)
        damaged[50] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            hc12_object(damaged)
        with self.assertRaisesRegex(ValueError, "end marker"):
            dsp_records(bytes.fromhex("000000 000001 000100 000000"))

    def test_runtime_extraction_and_original_boot_checksum(self):
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary)
            report = extract(self.path, destination)
            host = (destination / "host-mcu-8000.bin").read_bytes()
            self.assertEqual(hashlib.sha256(host).hexdigest(),
                             "a530c788593bc9ab2427f9d8104aefe2ede8eb91a1be34d779731491f96ed4f8")
            self.assertEqual(report["host_reset_vector"], 0x8000)
            self.assertTrue(report["host_boot_checksum_verified"])
            self.assertEqual(report["host_boot_checksum"], 0x2801)
            self.assertEqual(len(report["dsp_records"]), 127)
            reset_bytes = b"".join(word.to_bytes(3, "big") for word in report["dsp_reset_words"])
            self.assertEqual(hashlib.sha256(reset_bytes).hexdigest(),
                             "c2c38fd3763ee3cbef8971f325e63afc8ddeec30764fda72222600ed8bdf03bc")
            self.assertEqual(len(report["dsp_overwrites_in_record_order"]), 2)
            self.assertFalse(report["hardware_execution_verified"])


if __name__ == "__main__":
    unittest.main()
