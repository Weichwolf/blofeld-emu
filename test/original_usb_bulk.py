# SPDX-License-Identifier: GPL-3.0-only
"""Require original ROM-derived TCP discovery/import and native MIDI peer delivery."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(trace):
    bulk = re.findall(r"^USBIP_BULK (\d+) (\d+) (\d+) (\d+) (\d+)$", trace, re.M)
    discovery = re.findall(r"^RUNTIME_DISCOVERY (\d+) (\d+) (\d+) (\d+)$", trace, re.M)
    if len(bulk) != 1 or len(discovery) != 1:
        raise ValueError("Missing complete original bulk/discovery evidence")
    native, interrupts, payload, output, input_packets = map(int, bulk[0])
    setup_native, setup_irqs, polls, reply_bytes = map(int, discovery[0])
    if (native <= 0 or (interrupts, payload, output, input_packets) != (3, 6, 2, 1) or
            setup_native <= 0 or setup_irqs != 10 or polls <= 0 or reply_bytes != 332):
        raise ValueError("Wrong original native USB IRQ, payload, packet or discovery outcome")
    return {"bulk_instructions": native, "bulk_interrupts": interrupts,
            "native_peer_bytes": payload, "bulk_OUT_packets": output,
            "bulk_IN_ZLP_packets": input_packets, "discovery_instructions": setup_native,
            "discovery_interrupts": setup_irqs, "host_operation_polls": polls,
            "DEVLIST_bytes": reply_bytes}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original USB bulk: private firmware absent")
        return 77
    reference = json.loads((ROOT / "test/reference/usb-reset-reference.json").read_text())
    original_mcu(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=160)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "ThreadSanitizer", "runtime error:")):
        raise ValueError(f"Original USB bulk execution failed: {diagnostics}")
    totals = validate(trace.read_text())
    reply = Path(str(trace) + ".devlist")
    if reply.stat().st_size != totals["DEVLIST_bytes"]:
        raise ValueError("Missing exact original-device TCP DEVLIST artifact")
    report.write_text(json.dumps({"passed": True, **totals,
                                  "DEVLIST_sha256": hashlib.sha256(reply.read_bytes()).hexdigest(),
                                  "scope": "Conditional untimed MCU original startup/USB/TCP/peer; no coupled DSP synthesis, kernel attachment, physical timing or complete instrument qualification",
                                  "physical_traffic": False}, indent=2) + "\n")
    print("Original firmware TCP discovery/import, two bulk OUT notes, six native peer bytes, "
          "initial IN ZLP/ACK and three actual completion IRQs verified.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
