# SPDX-License-Identifier: GPL-3.0-only
"""Pin original GPIO routines for a declared passive reset-net experiment."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]
SLA_SHA256 = "b42ab86cb4c230e49c21e73c4295fc009caa2b2f31662e8f29d8dd65f699bce8"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def reference():
    data = json.loads((ROOT / "doc/mcu-power-on-reference.json").read_text())
    return {
        "mcu_sha256": data["mcu_sha256"],
        "routines": [
            {"entry": 0xb938, "bytes": 29,
             "sha256": "e85b3cab221d177e8961c14e8b886a481e4b82b46337058c4a688a7f53a14a43"},
            {"entry": 0xb98a, "bytes": 8,
             "sha256": "d9404ddbe85a533c7a6a79cf2a5dfbbc1e31b3eba291a8f1681ebec6318f6e78"},
        ],
    }


def validate(text):
    lines = text.splitlines()
    expected = [
        "Original prepare/U0-release/poll: ordered seven writes, passive high exit; "
        "other U inputs remain unknown",
        "Original poll remains low while external reset is asserted; exits after release",
    ]
    if len(lines) != 5 or lines[:2] != expected or lines[-1] != (
            "Conditional original GPIO routines pass; no physical wiring, DSP execution, "
            "synchronizer timing or complete boot claim"):
        raise ValueError("Missing original GPIO cases or changed scope")
    for line in lines[2:4]:
        if not line.startswith("Original unknown U0 refused: ") or not all(
                marker in line for marker in ("0x0278", "0xb98d", "mask=0x01")):
            raise ValueError("Missing actual original masked-input refusal/context")


def main():
    binary, firmware, specification, report = map(Path, sys.argv[1:])
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print("SKIP original GPIO reset: private firmware absent")
        return 77
    pinned = reference()
    original_mcu(firmware, pinned)
    if digest(specification) != SLA_SHA256:
        raise ValueError("Changed native HC12 specification")
    binary_hash = digest(binary)
    result = subprocess.run(
        [str(binary.resolve()), str(firmware.resolve()), str(specification.resolve())],
        capture_output=True, text=True, check=False, timeout=45)
    report.parent.mkdir(parents=True, exist_ok=True)
    report.with_suffix(".log").write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr:
        raise ValueError(f"Original GPIO execution failed: {result.stdout}{result.stderr}")
    validate(result.stdout)
    original_mcu(firmware, pinned)
    if digest(binary) != binary_hash or digest(specification) != SLA_SHA256:
        raise ValueError("Original GPIO executable/specification changed during execution")
    report.write_text(json.dumps({
        "passed": True, "cases": 4, "binary_sha256": binary_hash,
        "firmware_sha256": digest(firmware), "HC12_SLA_sha256": SLA_SHA256,
        "reference": pinned,
        "scope": "Declared passive PU0/reset net; isolated unchanged original routines. "
                 "No physical continuity, DSP execution, synchronizer timing or complete boot.",
    }, indent=2) + "\n")
    print("Verified four original GPIO reset-net cases with literal register/cycle expectations.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
