# SPDX-License-Identifier: GPL-3.0-only
"""Pin the unchanged status reader before executing independent cell-result cases."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original NAND status: private firmware absent")
        return 77
    reference = json.loads((ROOT / "test/reference/nand-functional-status-reference.json").read_text())
    records = json.loads((ROOT / "test/reference/dsp-voice-reference.json").read_text())
    program = {}
    original_records(firmware, records, program)
    span = reference["original_status_slice"]
    data = b"".join(program[address].to_bytes(3, "big")
                    for address in range(span["begin"], span["end"]))
    if len(data) != span["bytes"] or hashlib.sha256(data).hexdigest() != span["sha256"]:
        raise ValueError("Changed original NAND status reader")
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())],
                            check=False, capture_output=True, text=True, timeout=110)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.write_text(result.stdout + result.stderr)
    if result.returncode:
        print(result.stderr, end="", file=sys.stderr)
        return result.returncode
    names = [line.split()[1] for line in result.stdout.splitlines() if line.startswith("PASS ")]
    if names != reference["cases"]:
        raise ValueError("Incomplete original NAND status cases")
    print("Verified eight original status paths; full instrument and physical wiring remain open.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
