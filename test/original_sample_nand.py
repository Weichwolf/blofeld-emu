# SPDX-License-Identifier: GPL-3.0-only
"""Pin official assets before original sample-helper, queue and NAND execution."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from original_dsp_voice import original_records
from original_sample_preparation import extract_inputs, load_reference, sample_blocks
from sample_nand_validation import validate

ROOT = Path(__file__).resolve().parents[1]


def execute(binary, firmware, archive, output):
    report = output / "report.json"
    report.unlink(missing_ok=True)
    if not firmware.is_file() or not archive.is_file():
        print("SKIP original sample NAND: private original firmware or sample archive absent")
        return 77
    original_records(firmware, json.loads((ROOT / "doc/dsp-voice-reference.json").read_text()))
    preparation = json.loads((ROOT / "doc/sample-preparation-reference.json").read_text())
    messages, observations = load_reference(archive)
    helper, frames = extract_inputs(sample_blocks(messages), preparation, observations)
    output.mkdir(parents=True, exist_ok=True)
    paths = [output / "normal.trace", output / "inhibited.trace"]
    for path in paths:
        path.unlink(missing_ok=True)
    with tempfile.TemporaryDirectory(prefix="original-sample-nand-", dir=output) as directory:
        temporary = Path(directory)
        helper_path, frames_path = temporary / "helper.be24.bin", temporary / "frames.be24.bin"
        helper_path.write_bytes(helper)
        frames_path.write_bytes(frames)
        result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                                 str(helper_path), str(frames_path), str(output.resolve())],
                                check=False, capture_output=True, text=True, timeout=240)
    diagnostics = result.stdout + result.stderr
    (output / "native.log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer",
            "ThreadSanitizer", "runtime error:")):
        raise ValueError("Original sample NAND execution failed: " + diagnostics)
    reference = json.loads((ROOT / "doc/sample-nand-reference.json").read_text())
    totals = validate(diagnostics, [path.read_text() for path in paths], frames, reference)
    report.write_text(json.dumps({"passed": True, "scope": reference["scope"], **totals,
                                  "traces_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                                                    for path in paths}}, indent=2) + "\n")
    print(f"Original sample NAND: {totals['runs']} runs, 128 pages and 132 Timer1 entries each; "
          "actual failed status preserves original counter behavior; no SL/runtime acceptance.")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 5:
        sys.exit("Usage: original_sample_nand.py binary firmware sample-archive output")
    try:
        sys.exit(execute(*map(Path, sys.argv[1:])))
    except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
