#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Observe Linux's cached USB data without opening a USB device or sending traffic."""

import argparse
import datetime
import errno
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import stat
import subprocess
import sys


REPOSITORY = Path(__file__).resolve().parents[1]
DEVICE_NAME = re.compile(r"[0-9]+-[0-9]+(?:\.[0-9]+)*")
DESCRIPTOR_LIMIT = 1024 * 1024
ATTRIBUTE_LIMIT = 4096
ENTRY_LIMIT = 4096
IDENTITY = ("busnum", "devnum", "idVendor", "idProduct")
DEVICE_ATTRIBUTES = (*IDENTITY, "bcdDevice", "bDeviceClass", "bNumConfigurations",
                     "bConfigurationValue", "speed", "version", "manufacturer", "product", "serial")
INTERFACE_ATTRIBUTES = ("bInterfaceNumber", "bAlternateSetting", "bNumEndpoints",
                        "bInterfaceClass", "bInterfaceSubClass", "bInterfaceProtocol")
ENDPOINT_ATTRIBUTES = ("bEndpointAddress", "bmAttributes", "wMaxPacketSize", "bInterval",
                       "direction", "type", "interval")


class CaptureError(ValueError):
    """The requested observation cannot be acquired with its declared provenance."""


def read_bytes(path, limit):
    result = bytearray()
    descriptor = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK | os.O_NOFOLLOW)
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise CaptureError(f"USB cache input is not a regular attribute: {path}")
        while True:
            chunk = os.read(descriptor, min(4096, limit + 1 - len(result)))
            if not chunk:
                return bytes(result)
            result.extend(chunk)
            if len(result) > limit:
                raise CaptureError(f"Observation exceeds the tool's {limit}-byte limit: {path}")
    finally:
        os.close(descriptor)


def attributes(path, names, required=()):
    result = {}
    for name in names:
        try:
            value = read_bytes(path / name, ATTRIBUTE_LIMIT).decode("utf-8").removesuffix("\n")
            result[name] = {"status": "observed", "value": value}
        except OSError as error:
            if name in required:
                raise CaptureError(f"Required USB cache attribute unavailable: {path / name}") from error
            status = "unavailable" if error.errno == errno.ENOENT else "unreadable"
            result[name] = {"status": status, "errno": error.errno}
    return result


def children(path):
    result = []
    for entry in path.iterdir():
        result.append(entry)
        if len(result) > ENTRY_LIMIT:
            raise CaptureError(f"Directory exceeds the tool's entry limit: {path}")
    return sorted(result)


def endpoints(path):
    return [{"name": entry.name, "attributes": attributes(entry, ENDPOINT_ATTRIBUTES)}
            for entry in children(path)
            if re.fullmatch(r"ep_[0-9a-fA-F]{2}", entry.name) and entry.is_dir()]


def interface_snapshot(root, device):
    result = []
    for entry in children(root):
        if not entry.name.startswith(device.name + ":") or not entry.is_dir():
            continue
        try:
            driver = {"status": "observed", "name": Path(os.readlink(entry / "driver")).name}
        except OSError as error:
            driver = {"status": "unavailable" if error.errno == errno.ENOENT else "unreadable",
                      "errno": error.errno}
        result.append({"name": entry.name, "attributes": attributes(entry, INTERFACE_ATTRIBUTES),
                       "driver": driver, "endpoints": endpoints(entry)})
    return result


def snapshot(root, device):
    return {"attributes": attributes(device, DEVICE_ATTRIBUTES, IDENTITY),
            "interfaces": interface_snapshot(root, device), "endpoints": endpoints(device)}


def acquire(root, name, profile=None, firmware_version=None):
    if DEVICE_NAME.fullmatch(name) is None:
        raise CaptureError("Select a USB device port such as 1-3, not an interface or path")
    root = root.resolve(strict=True)
    device = root / name
    started = datetime.datetime.now(datetime.timezone.utc).isoformat()
    initial = device.stat()
    first = snapshot(root, device)
    raw = read_bytes(device / "descriptors", DESCRIPTOR_LIMIT)
    second = snapshot(root, device)
    repeated = read_bytes(device / "descriptors", DESCRIPTOR_LIMIT)
    final = device.stat()
    if ((initial.st_dev, initial.st_ino) != (final.st_dev, final.st_ino) or
            first != second or raw != repeated):
        raise CaptureError("USB cache changed during observation; no stable snapshot acquired")
    metadata = {"schema": 1, "status": "kernel-cache-observation", "port": name,
                "source_root": str(root), "resolved_device": str(device.resolve(strict=True)),
                "started_utc": started,
                "finished_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "kernel_release": platform.release(), "uid": os.geteuid(),
                "consistency": "two equal cached observations and unchanged device inode",
                "descriptor_bytes": len(raw), "descriptor_sha256": hashlib.sha256(raw).hexdigest(),
                "operator_declarations": {"profile": profile, "firmware_version": firmware_version},
                "observations": first, "hardware_equivalence_verified": False,
                "usb_or_midi_requests_sent": False, "device_programming": False}
    return metadata, raw


def save_capture(metadata, raw, output, repository=REPOSITORY):
    repository = repository.resolve(strict=True)
    output = output.absolute()
    private = repository / "doc/local"
    if not output.resolve().is_relative_to(private.resolve()):
        raise CaptureError("Hardware observations must stay under the repository's doc/local/")
    result = subprocess.run(["git", "-C", str(repository), "check-ignore", "-q", "--", str(output)],
                            check=False, capture_output=True)
    if result.returncode != 0:
        raise CaptureError("Hardware observation output must be Git-ignored")
    if len(raw) != metadata["descriptor_bytes"] or hashlib.sha256(raw).hexdigest() != metadata["descriptor_sha256"]:
        raise CaptureError("Descriptor bytes do not match the acquired observation")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()
    with (output / "descriptors.bin").open("xb") as stream:
        stream.write(raw)
    with (output / "capture.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=True)
        stream.write("\n")


def candidates(root):
    result = []
    for entry in children(root.resolve(strict=True)):
        if DEVICE_NAME.fullmatch(entry.name) is None or not entry.is_dir():
            continue
        observed = attributes(entry, ("manufacturer", "product"))
        if "blofeld" in observed["product"].get("value", "").lower():
            result.append({"port": entry.name, "cached_strings": observed})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--list", action="store_true", help="List matching cached product names")
    selection.add_argument("--device", help="Explicit USB device port, for example 1-3")
    parser.add_argument("--sysfs", type=Path, default=Path("/sys/bus/usb/devices"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--profile", choices=("desktop", "keyboard"))
    parser.add_argument("--firmware-version", help="Operator declaration, not device verification")
    args = parser.parse_args()
    try:
        if args.list:
            print(json.dumps({"source": "kernel-cache", "matches": candidates(args.sysfs),
                              "hardware_equivalence_verified": False}, indent=2))
        else:
            if args.output is None:
                parser.error("--device requires --output under doc/local/")
            metadata, raw = acquire(args.sysfs, args.device, args.profile, args.firmware_version)
            save_capture(metadata, raw, args.output)
            print(json.dumps({"status": metadata["status"], "output": str(args.output),
                              "descriptor_bytes": metadata["descriptor_bytes"],
                              "descriptor_sha256": metadata["descriptor_sha256"]}))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        parser.exit(1, f"USB observation failed: {error}\n")


if __name__ == "__main__":
    main()
