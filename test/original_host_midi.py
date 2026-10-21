# SPDX-License-Identifier: GPL-3.0-only
"""Bind the original cold-start/SCI image and the exact own host MIDI input."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original host MIDI: private firmware absent")
        return 77
    root = Path(__file__).resolve().parents[1]
    reference = json.loads((root / "doc/host-midi-reference.json").read_text())
    original_mcu(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    source = trace.with_suffix(".bytes")
    message = bytes((0xf8, 0x90, 60, 100, 0xf8, 0x80, 60, 0))
    source.write_bytes(message)
    identities = {path: hashlib.sha256(path.read_bytes()).hexdigest()
                  for path in (binary, firmware, source)}
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([binary, firmware, source, trace], capture_output=True,
                            text=True, timeout=280, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "runtime error:")):
        raise ValueError(f"Original host MIDI execution failed: {diagnostics}")
    values = re.findall(r"^HOST-MIDI byte=0x([0-9a-f]{2}) isr-instructions=(12|16) pc=0x8315$",
                        trace.read_text(), re.M)
    expected = [(f"{byte:02x}", "12" if byte >= 0xf8 else "16") for byte in message]
    if values != expected:
        raise ValueError("Missing complete ordered original SCI/RTI returns for the host source")
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest
           for path, digest in identities.items()):
        raise ValueError("Original program/input executable changed during execution")
    report.write_text(json.dumps({"passed": True, "bytes": len(message),
                                  "source_sha256": hashlib.sha256(message).hexdigest(),
                                  "scope": "Conditional untimed original MCU cold-start and SCI byte "
                                           "handlers, not complete MIDI/board/DSP/audio or hardware",
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print(diagnostics, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
