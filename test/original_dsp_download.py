# SPDX-License-Identifier: GPL-3.0-only
"""Check unchanged download commands against independent own instruction/data fixtures."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE_SHA256 = "ed7e558417d370174eaa925eee860615a56fc4b644eda45d26acee19589a18d8"
HEADER_MODES = (0, 1, 2, 3, 4, 5, 6, 127)
HEADERS = ((0, 0), (0, 1), (0, 2), (0, 3), (0x600, 9), (0x600, 0x7fffff),
           (0x600, 0x800000), (0x600, 0xfffffd), (0x600, 0xffffff),
           (0x1000600, 0x1000009), (0xfffffff, 0xfffffff), (0x8000600, 0x8000009))


def fixture(assembler, trace):
    version = subprocess.check_output([assembler, "--version"], text=True, timeout=10).strip()
    if version != "dsp56300-asm 1.0.0":
        raise ValueError("Expected the independent offline assembler 1.0.0")
    source = ROOT / "test/dsp_download_templates.asm"
    binary = trace.with_suffix(".own-templates.bin")
    subprocess.run([assembler, source, "-f", "u24be", "-o", binary], check=True, timeout=10)
    data = binary.read_bytes()
    if len(data) != 18 or hashlib.sha256(data).hexdigest() != TEMPLATE_SHA256:
        raise ValueError("Independent own instruction expectations changed")
    words = trace.with_suffix(".own-templates.txt")
    words.write_text("".join(f"{int.from_bytes(data[i:i + 3], 'big'):06x}\n"
                             for i in range(0, len(data), 3)))
    return words, {"version": version, "sha256": hashlib.sha256(assembler.read_bytes()).hexdigest(),
                   "own_source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                   "own_words_sha256": TEMPLATE_SHA256}


def expected_events():
    events = []
    for mode in range(4):
        for command in (0x74, 0x75):
            for checksum in (0, 127):
                events.append(("CASE", f"{mode} {command} {checksum}"))
    for mode in range(4):
        for command in (0x74, 0x75):
            events.append(("CASE", f"{mode} {command} 0"))  # Real reconfiguration recovery.
            events.extend((name, f"{mode} {command}")
                          for name in ("SEQUENCE", "WRAP", "ZERO_ADDRESS", "EMPTY"))
    events.extend(("HEADER", f"{mode} {address} {length}")
                  for mode in HEADER_MODES for address, length in HEADERS)
    return events


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    trace.unlink(missing_ok=True)
    assembler = Path(os.environ.get("BLOFELD_DSP_ASSEMBLER", str(
        ROOT / "doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm"))).resolve()
    if not firmware.is_file() or not assembler.is_file():
        print("SKIP original DSP download: local firmware or independent offline assembler absent")
        return 77
    reference = json.loads((ROOT / "doc/dsp-voice-reference.json").read_text())
    original_records(firmware, reference)
    selected = [reference["records"][i] for i in (93, 94)]
    if [(row["space"], row["address"], row["words"]) for row in selected] != [
            (1, 0x2c25, 12), (0, 0x2361, 126)]:
        raise ValueError("Changed original complete download record selection")
    trace.parent.mkdir(parents=True, exist_ok=True)
    words, provenance = fixture(assembler, trace)
    with trace.open("w") as output:
        result = subprocess.run([binary.resolve(), firmware.resolve(), words.resolve()],
                                stdout=output, stderr=subprocess.STDOUT, check=False, timeout=60)
    if result.returncode:
        print(trace.read_text())
        return result.returncode
    text = trace.read_text()
    observed = re.findall(r"^(CASE|SEQUENCE|WRAP|ZERO_ADDRESS|EMPTY|HEADER) (.+) passed$", text, re.M)
    if observed != expected_events():
        raise ValueError("Changed fixed original download/recovery/header case matrix")
    totals = re.findall(r"^TOTAL 144 cases 1240 calls (\d+) steps prefetches 1088 stores 96 launches 6$", text, re.M)
    if len(totals) != 1 or not 0 < int(totals[0]) < 1240 * 2048:
        raise ValueError("Incomplete original command/caller/I/O scope")
    if any(marker in text for marker in ("AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "runtime error:")):
        raise ValueError("Sanitizer finding in original download regression")
    report.write_text(json.dumps({
        "passed": True, "cases": 144, "calls": 1240, "observed_steps": int(totals[0]),
        "prefetches": 1088, "target_stores": 96, "launches": 6, "records": selected,
        "independent_assembler": provenance, "trace_sha256": hashlib.sha256(trace.read_bytes()).hexdigest(),
        "fuzz_execution": False, "physical_traffic": False,
        "qualification": "Unchanged original command record with declared linear/unscaled own caller; "
                         "independent full accumulators/header/end arithmetic, exact payload/bank guards, "
                         "self-modification, sequence failure/disabled/wrap/reconfiguration and SC=0 zero groups. "
                         "No original dispatcher/transport/checksum layer, complete boot, physical flash "
                         "programming/interruption/recovery, pipeline/timing or hardware equivalence proof."
    }, indent=2) + "\n")
    print("Verified 144 original DSP download cases and 1240 actual calls")
    return 0


if __name__ == "__main__":
    sys.exit(main())
