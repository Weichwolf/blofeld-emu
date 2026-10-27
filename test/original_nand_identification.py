# SPDX-License-Identifier: GPL-3.0-only
"""Pin both original updater objects and qualify their conditional ID paths."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from inspect_firmware import decode_block, load_input, read_smf, split_blocks


def verify_firmware(path, reference):
    if hashlib.sha256(path.read_bytes()).hexdigest() != reference["firmware_sha256"]:
        raise ValueError("Changed original firmware archive")
    data, _ = load_input(path)
    blocks = split_blocks(read_smf(data)[0])
    for helper in reference["helpers"]:
        body = decode_block(blocks[helper["block"]])[1]
        if len(body) != helper["bytes"] or hashlib.sha256(body).hexdigest() != helper["sha256"]:
            raise ValueError("Changed original NAND updater object")
        suffix = body[(helper["identify"] - 0x2e00) * 3:]
        if hashlib.sha256(suffix).hexdigest() != helper["identify_span_sha256"]:
            raise ValueError("Changed original identification routine")


def events(body, helper, replay):
    read_rows = [tuple(map(int, row)) for row in re.findall(r"^READ (\d+) (\d+)$", body, re.M)]
    expected_bytes = (0xec, 0x76, 0x5a, 0x3f)
    first = 2 if replay else 0
    if read_rows != list(enumerate(expected_bytes))[first:]:
        raise ValueError("Missing independent ID byte observations or replay cursor")
    latches = [tuple(map(int, row)) for row in re.findall(r"^LATCH (\d+) (\d+)$", body, re.M)]
    if latches != ([] if replay else [(0x400, 0x90), (0x800, 0)]):
        raise ValueError("Wrong actual command/address latches or repeated setup after restore")
    steps = [tuple(map(int, row)) for row in re.findall(
        r"^STEP (\d+) (\d+) (\d+) (\d+) (\d+)$", body, re.M)]
    # Own setup=29 instructions; original tail=4*(BSR+7 reader steps)+6=38.
    # The cut after the second RE store leaves 27 original instructions.
    expected_steps = 27 if replay else 67
    original = [row for row in steps if 0x2e00 <= row[0] < 0x2e00 + helper["bytes"] // 3]
    if len(steps) != expected_steps or len(original) != (27 if replay else 38):
        raise ValueError("Wrong native/original instruction count or source extent")
    if sum(row[0] == helper["reader"] for row in steps) != (2 if replay else 4):
        raise ValueError("Missing actual original byte-reader entries")
    if not steps or steps[-1][2:] != (1, 1, 4) or steps[-1][1] < 0xff0000:
        raise ValueError("Original path must finish through its own return into the declared caller")
    if not replay and (steps[0][0] != 0xff0000 or
                       sum(row[0] == helper["packing"] for row in steps) != 1):
        raise ValueError("Missing actual reset/caller and original packing entry")
    return len(steps), len(original)


def validate(text, reference):
    def rows(kind, width):
        pattern = r"^" + kind + (r" (\d+)" * width) + "$"
        return [tuple(map(int, row)) for row in re.findall(pattern, text, re.M)]
    helpers = {helper["block"]: helper for helper in reference["helpers"]}
    expected = list(helpers)
    if rows("PACK", 3) != [(block, 67, 0xec763f) for block in expected]:
        raise ValueError("Missing both original packed-ID outcomes")
    if rows("REPLAY", 3) != [(block, replay, 67) for block in expected for replay in range(2)]:
        raise ValueError("Missing four joint native/ID/cursor restores")
    packs = re.findall(r"^BEGIN_PACK (\d+)\n(.*?)^END_PACK \1\n", text, re.M | re.S)
    replays = re.findall(r"^BEGIN_REPLAY (\d+) (\d+)\n(.*?)^END_REPLAY \1 \2\n", text, re.M | re.S)
    if [int(block) for block, _ in packs] != expected or [
            (int(block), int(replay)) for block, replay, _ in replays] != [
            (block, replay) for block in expected for replay in range(2)]:
        raise ValueError("Missing actual event traces for original paths and restores")
    totals = [events(body, helpers[int(block)], False) for block, body in packs]
    totals += [events(body, helpers[int(block)], True) for block, _, body in replays]
    turns = re.findall(r"^TURNAROUND (\d+) (\d+) (\d+) (.+)$", text, re.M)
    if [(int(block), int(cut), int(reads)) for block, cut, reads, _ in turns] != [
            (block, 0, 0) for block in expected]:
        raise ValueError("Missing both unchanged original output-to-input frontiers")
    for block, _, _, reason in turns:
        pc = helpers[int(block)]["packing"] - 2
        if reason != (f"NAND ID at X:0xffffbd dsp-pc=0x{pc:06x} omr=0x600389: "
                      "Unqualified NAND read turnaround: DSP data pins still driven"):
            raise ValueError("Different failure replaced the original turnaround")
    missing = re.findall(r"^MISSING (\d+) (\d+) (\d+) (.+)$", text, re.M)
    if [(int(block), int(cut), int(reads)) for block, cut, reads, _ in missing] != [
            (block, cut, cut) for block in expected for cut in range(4)]:
        raise ValueError("Missing all eight native unknown-ID boundaries")
    for block, cut, _, reason in missing:
        pc = helpers[int(block)]["reader"]
        expected_reason = (f"NAND ID at X:0xffffbd dsp-pc=0x{pc:06x} omr=0x600389: "
                           "NAND identification read page=0x000000 column=0x000 command=0x90: "
                           f"unknown bits byte={cut} mask=0xff")
        if reason != expected_reason:
            raise ValueError("Missing precise unknown-byte/PC/operation context")
    return {"packing_paths": 2, "joint_restores": 4, "unknown_byte_failures": 8,
            "turnaround_frontiers": 2, "positive_native_retirements": sum(row[0] for row in totals),
            "positive_original_retirements": sum(row[1] for row in totals)}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original NAND ID: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/nand-identification-reference.json").read_text())
    verify_firmware(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=120)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or result.stderr or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "ThreadSanitizer", "runtime error:")):
        raise ValueError(f"Original NAND ID execution failed: {diagnostics}")
    totals = validate(trace.read_text(), reference)
    report.write_text(json.dumps({"passed": True, **totals, "scope": reference["qualification"],
                                 "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified both original ID packing tails and joint restores under declared inputs; "
          "full original ID setup retains its turnaround failure. No boot or hardware qualification.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
