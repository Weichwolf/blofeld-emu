# SPDX-License-Identifier: GPL-3.0-only
"""Require native foreground and repeating callers to reject changed program words."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, directory = (Path(name).resolve() for name in sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original program guard: private firmware absent")
        return 77
    original_records(firmware, json.loads((ROOT / "doc/dsp-voice-reference.json").read_text()))
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest() for path in (binary, firmware)}
    directory.mkdir(parents=True, exist_ok=True)
    for profile in ("foreground", "stream"):
        trace = directory / (profile + ".trace")
        trace.unlink(missing_ok=True)
        result = subprocess.run([str(binary), str(firmware), str(trace), profile],
                                capture_output=True, text=True, timeout=50, check=False)
        diagnostics = result.stdout + result.stderr
        (directory / (profile + ".log")).write_text(diagnostics)
        if result.returncode or result.stderr or any(
                marker in diagnostics for marker in
                ("AddressSanitizer", "LeakSanitizer", "ThreadSanitizer", "runtime error:")):
            raise ValueError("Original program guard failed: " + diagnostics)
        if re.findall(r"^program-guard-rejected profile=(\w+) blocks=1$",
                      result.stdout, re.M) != [profile]:
            raise ValueError("Missing exact first-block guard rejection for " + profile)
    if not all(hashlib.sha256(path.read_bytes()).hexdigest() == digest
               for path, digest in frozen.items()):
        raise ValueError("Program-guard reference inputs changed")
    (directory / "result.json").write_text(json.dumps({
        "passed": True, "profiles": 2, "mutated_words": 2, "completed_blocks_per_profile": 1,
        "rejected_before_next_block": True, "fuzz_execution": False, "physical_traffic": False,
        "scope": "Offline directed program-corruption controls; ordinary original-firmware "
        "qualification remains unmodified. No boot or hardware equivalence claim."
    }, indent=2) + "\n")
    print("Both native foreground program guards reject before the second block.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
