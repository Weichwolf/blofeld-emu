# SPDX-License-Identifier: GPL-3.0-only
"""Pin original SHI source records before the native tagged-reception regression."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original SHI receive: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    selected = [entry for entry in reference["records"]
                if entry["space"] == 0 and entry["address"] in (0x44, 0x4a, 0x23df)]
    if [(entry["record"], entry["address"], entry["words"]) for entry in selected] != [
            (96, 0x44, 2), (97, 0x4a, 2), (100, 0x23df, 189)]:
        raise ValueError("Changed complete original SHI record selection")
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    result = subprocess.run([binary.resolve(), firmware.resolve(), trace.resolve()],
                            check=False, timeout=170)
    if result.returncode:
        return result.returncode
    text = trace.read_text()
    expected = ("cases=8 runs=16 wire=196 received=194 queued=182 idle=12 lost=2 "
                "overwrites=138 steps=3480 units=7348")
    if re.findall(r"^RESULT (.+)$", text, re.M) != [expected]:
        raise ValueError("Incomplete original tagged SHI scope")
    names = ("one-tagged", "one-idle", "three-tagged", "three-idle", "mixed",
             "full-fifo", "overrun", "ring-alias")
    if re.findall(r"^CASE (\S+) replay=([01])$", text, re.M) != [
            (name, str(replay)) for name in names for replay in range(2)]:
        raise ValueError("Changed original SHI case/restore matrix")
    trace.with_suffix(".json").write_text(json.dumps({
        "passed": True, "cases": 8, "runs": 16, "received": 194,
        "pending_overwrites": 138, "handler_steps": 3480, "nominal_units": 7348,
        "records": selected, "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
        "qualification": "Unmodified original initializer/vectors/tagged receiver; explicit own "
                         "native caller, nominal CPU clock/IRQ estimates and untimed SPI inputs. "
                         "Independent wire/FIFO/ring/flags/register/order/return expectations. "
                         "No original caller/boot provenance, untagged reset qualification, "
                         "physical timing, complete MIDI/audio, update/recovery or hardware proof."
    }, indent=2) + "\n")
    print("Verified eight original tagged SHI cases and sixteen native runs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
