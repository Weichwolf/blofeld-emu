# SPDX-License-Identifier: GPL-3.0-only
"""Retain contextual fail-stop behavior for pinned native shift regressions."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def identity(path):
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def main():
    binary, reference, manifest, output = map(Path, sys.argv[1:])
    if not reference.is_dir():
        print("SKIP: pinned external operand-regression inputs are absent")
        return 77
    record = json.loads(manifest.read_text())
    output.mkdir(parents=True, exist_ok=True)
    frozen_binary = identity(binary)
    reports = []
    for item in record["regressions"]:
        directory = reference / item["bank"]
        for name, expected in item["files"].items():
            if identity(directory / name) != expected:
                raise ValueError(f"Operand regression identity mismatch: {directory / name}")
        target = output / f"{item['bank']}.native.txt"
        result = subprocess.run([str(binary.resolve()), str(directory.resolve()), str(target)],
                                capture_output=True, text=True, check=False, timeout=120)
        text = result.stdout + result.stderr
        (output / f"{item['bank']}.log").write_text(text)
        if result.returncode != item["expected_exit"]:
            raise AssertionError(f"Wrong native exit for {item['bank']}: {result.returncode}")
        for token in ("Unqualified DSP reference " + item["operation"],
                      "pc=" + item["pc"], "omr=" + item["omr"], item["control"]):
            if token not in result.stderr:
                raise AssertionError(f"Missing contextual native diagnostic {token}: {text}")
        for token in ("runtime error:", "AddressSanitizer", "UndefinedBehaviorSanitizer"):
            if token in text:
                raise AssertionError(f"Sanitizer failure in {item['bank']}: {text}")
        if identity(binary) != frozen_binary:
            raise AssertionError("Native binary changed during operand regression execution")
        reports.append({"bank": item["bank"], "expected_rejection": True,
                        "exit": result.returncode, "diagnostic": result.stderr,
                        "inputs": item["files"], "output": identity(target)})
    (output / "verification.json").write_text(json.dumps({
        "scope": record["scope"], "binary": frozen_binary, "regressions": reports,
        "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Both fixed native operand regressions reject before host UB with exact PC/OMR/control.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
