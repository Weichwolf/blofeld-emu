#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Exercise acquisition against independent synthetic sysfs files, never a USB device."""

import errno
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("usb_observation", ROOT / "tools/capture_usb_sysfs.py")
OBSERVER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OBSERVER)
OWN_DESCRIPTOR = bytes([18, 1, 0, 2, 0, 0, 0, 64, 0x34, 0x12, 0x78, 0x56, 0, 1, 0, 0, 0, 1])


class UsbObservationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="blofeld-usb-cache-")
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.sysfs = self.base / "devices"
        self.device = self.sysfs / "1-2"
        self.device.mkdir(parents=True)
        for name, value in {"busnum": "1", "devnum": "9", "idVendor": "1234", "idProduct": "5678",
                            "product": "Blofeld fixture", "manufacturer": "Own fixture",
                            "bConfigurationValue": "1"}.items():
            (self.device / name).write_text(value + "\n")
        (self.device / "descriptors").write_bytes(OWN_DESCRIPTOR)
        self.interface = self.sysfs / "1-2:1.0"
        self.interface.mkdir()
        (self.interface / "bInterfaceNumber").write_text("00\n")
        (self.interface / "driver").symlink_to("/unopened/kernel/driver/own-usb-driver")
        endpoint = self.interface / "ep_81"
        endpoint.mkdir()
        (endpoint / "bEndpointAddress").write_text("81\n")
        (endpoint / "bmAttributes").write_text("02\n")
        self.repository = self.base / "repository"
        self.repository.mkdir()
        subprocess.run(["git", "init", "-q", str(self.repository)], check=True, capture_output=True)
        (self.repository / ".gitignore").write_text("doc/local/\n")

    def capture(self):
        return OBSERVER.acquire(self.sysfs, "1-2", "keyboard", "1.25")

    def test_stable_cached_data_is_preserved_without_claiming_hardware_validation(self):
        before = {f: f.read_bytes() for f in self.device.iterdir() if f.is_file()}
        metadata, raw = self.capture()
        self.assertEqual(raw, OWN_DESCRIPTOR)
        self.assertEqual(metadata["descriptor_bytes"], 18)
        self.assertFalse(metadata["hardware_equivalence_verified"])
        self.assertFalse(metadata["usb_or_midi_requests_sent"])
        self.assertFalse(metadata["device_programming"])
        self.assertEqual(metadata["operator_declarations"], {"profile": "keyboard", "firmware_version": "1.25"})
        interface = metadata["observations"]["interfaces"][0]
        self.assertEqual(interface["driver"]["name"], "own-usb-driver")
        self.assertEqual(interface["endpoints"][0]["attributes"]["bEndpointAddress"]["value"], "81")
        self.assertEqual(metadata["observations"]["attributes"]["serial"]["status"], "unavailable")
        self.assertEqual(before, {f: f.read_bytes() for f in self.device.iterdir() if f.is_file()})

    def test_short_reads_are_not_treated_as_eof_or_a_stat_size_contract(self):
        read = os.read
        with mock.patch.object(OBSERVER.os, "read", side_effect=lambda fd, size: read(fd, min(size, 3))):
            metadata, raw = self.capture()
        self.assertEqual(raw, OWN_DESCRIPTOR)
        self.assertEqual(metadata["observations"]["attributes"]["idVendor"]["value"], "1234")

    def test_attribute_padding_is_preserved_and_changes_are_detected(self):
        (self.device / "product").write_text(" Blofeld fixture \n")
        metadata, _ = self.capture()
        self.assertEqual(metadata["observations"]["attributes"]["product"]["value"], " Blofeld fixture ")
        read = OBSERVER.read_bytes
        changed = False

        def alter_padding(path, limit):
            nonlocal changed
            data = read(path, limit)
            if path.name == "descriptors" and not changed:
                changed = True
                (self.device / "product").write_text("Blofeld fixture\n")
            return data

        with mock.patch.object(OBSERVER, "read_bytes", side_effect=alter_padding):
            with self.assertRaisesRegex(OBSERVER.CaptureError, "changed during observation"):
                self.capture()

    def test_reenumeration_between_observations_fails(self):
        read = OBSERVER.read_bytes
        changed = False

        def reenumerate(path, limit):
            nonlocal changed
            data = read(path, limit)
            if path.name == "descriptors" and not changed:
                changed = True
                (self.device / "devnum").write_text("10\n")
            return data

        with mock.patch.object(OBSERVER, "read_bytes", side_effect=reenumerate):
            with self.assertRaisesRegex(OBSERVER.CaptureError, "changed during observation"):
                self.capture()

    def test_disconnection_does_not_become_a_successful_capture(self):
        read = OBSERVER.read_bytes

        def disconnect(path, limit):
            data = read(path, limit)
            if path.name == "descriptors":
                (self.device / "idProduct").unlink()
            return data

        with mock.patch.object(OBSERVER, "read_bytes", side_effect=disconnect):
            with self.assertRaisesRegex(OBSERVER.CaptureError, "Required USB cache attribute"):
                self.capture()

    def test_optional_capability_denial_is_explicit_and_required_identity_denial_fails(self):
        read = OBSERVER.read_bytes

        def denied(path, limit):
            if path.name == "manufacturer":
                raise PermissionError(errno.EACCES, "Synthetic permission denial")
            return read(path, limit)

        with mock.patch.object(OBSERVER, "read_bytes", side_effect=denied):
            metadata, _ = self.capture()
        self.assertEqual(metadata["observations"]["attributes"]["manufacturer"],
                         {"status": "unreadable", "errno": errno.EACCES})
        (self.device / "devnum").unlink()
        with self.assertRaisesRegex(OBSERVER.CaptureError, "Required USB cache attribute"):
            self.capture()

    def test_only_ignored_private_outputs_are_written_and_existing_capture_is_not_overwritten(self):
        metadata, raw = self.capture()
        with self.assertRaisesRegex(OBSERVER.CaptureError, "doc/local"):
            OBSERVER.save_capture(metadata, raw, self.repository / "public", self.repository)
        output = self.repository / "doc/local/captured"
        OBSERVER.save_capture(metadata, raw, output, self.repository)
        self.assertEqual((output / "descriptors.bin").read_bytes(), OWN_DESCRIPTOR)
        self.assertEqual(json.loads((output / "capture.json").read_text()), metadata)
        with self.assertRaises(FileExistsError):
            OBSERVER.save_capture(metadata, raw, output, self.repository)
        untracked = subprocess.check_output(["git", "-C", str(self.repository), "ls-files",
                                             "--others", "--exclude-standard"], text=True)
        self.assertNotIn("capture", untracked)

    def test_unignored_private_path_and_inconsistent_bytes_are_rejected(self):
        metadata, raw = self.capture()
        output = self.repository / "doc/local/captured"
        (self.repository / ".gitignore").write_text("")
        with self.assertRaisesRegex(OBSERVER.CaptureError, "Git-ignored"):
            OBSERVER.save_capture(metadata, raw, output, self.repository)
        (self.repository / ".gitignore").write_text("doc/local/\n")
        with self.assertRaisesRegex(OBSERVER.CaptureError, "do not match"):
            OBSERVER.save_capture(metadata, b"different", output, self.repository)
        self.assertFalse(output.exists())

    def test_limits_and_nonregular_attributes_fail_without_blocking(self):
        with mock.patch.object(OBSERVER, "DESCRIPTOR_LIMIT", 8):
            with self.assertRaisesRegex(OBSERVER.CaptureError, "tool's 8-byte limit"):
                self.capture()
        (self.device / "descriptors").unlink()
        os.mkfifo(self.device / "descriptors")
        with self.assertRaisesRegex(OBSERVER.CaptureError, "not a regular attribute"):
            self.capture()

    def test_explicit_device_selection_and_cli_discovery(self):
        for invalid in ("../1-2", "1-2:1.0", "/1-2"):
            with self.assertRaisesRegex(OBSERVER.CaptureError, "Select a USB device port"):
                OBSERVER.acquire(self.sysfs, invalid)
        result = subprocess.run([sys.executable, "-B", str(ROOT / "tools/capture_usb_sysfs.py"),
                                 "--sysfs", str(self.sysfs), "--list"], check=True,
                                capture_output=True, text=True)
        observed = json.loads(result.stdout)
        self.assertEqual([item["port"] for item in observed["matches"]], ["1-2"])
        self.assertFalse(observed["hardware_equivalence_verified"])


if __name__ == "__main__":
    unittest.main()
