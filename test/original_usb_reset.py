# SPDX-License-Identifier: GPL-3.0-only
"""Pin original USB routines before cold-start/IRQ/reset and replay checks."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate_suspend(text):
    def rows(kind):
        return [tuple(map(int, row)) for row in re.findall(
            rf"^{kind} (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    cases = rows("SUSPEND_CASE")
    expected = [(profile, int(profile % 2 == 0), 7, 1, 1, 3) for profile in range(4)]
    if cases != expected or rows("SUSPEND_REPLAY") != [row for row in expected for _ in range(2)]:
        raise ValueError("Missing original suspend/resume IRQ, actual RTI or eight mid-handler restores")
    foreground = [tuple(map(int, row)) for row in re.findall(
        r"^SUSPEND_FOREGROUND (\d+) (\d+)$", text, re.M)]
    if len(foreground) != 4 or [row[0] for row in foreground] != list(range(4)):
        raise ValueError("Missing all four original foreground continuations to actual WAI")
    if any(native <= 7 for _, native in foreground):
        raise ValueError("Original foreground must execute its own instructions after RTI")
    # Each initial handler retires seven steps; each restore after step five
    # retires only the final two again. Replayed prefixes are not new work.
    return {"suspend_resume_changes": 4, "suspend_resume_irqs": 4, "suspend_resume_returns": 12,
            "suspend_resume_restores": 8, "suspend_handler_retirements": 4 * (7 + 2 * 2),
            "suspend_foreground_retirements": sum(native - 7 for _, native in foreground)}


def validate(text):
    def rows(kind):
        return [tuple(map(int, match)) for match in re.findall(
            rf"^{kind} (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    resets, replays, configurations = rows("RESET"), rows("REPLAY"), rows("CONFIGURATION")
    if len(resets) != 3 or [row[:5] for row in resets] != [
            (0, 1, 0, 1, 1), (1, 3, 2, 1, 1), (2, 256, 255, 1, 1)]:
        raise ValueError("Incomplete original USB reset/counter/IRQ/RTI matrix")
    if replays != [resets[1], resets[1]]:
        raise ValueError("Missing complete mid-ISR replay comparisons")
    if (len(configurations) != 3 or configurations != [configurations[0]] * 3 or
            configurations[0][:5] != (1, 0, 0, 1, 1)):
        raise ValueError("Missing original configuration IRQ acknowledgement and two replays")
    if re.findall(r"^CONFIGURATION_SETUP (.+)$", text, re.M) != ["ACK NAK"]:
        raise ValueError("Missing actual configuration SETUP and pending status NAK")
    status = rows("CONFIGURATION_STATUS")
    if (len(status) != 3 or status != [status[0]] * 3 or
            status[0][:5] != (1, 0, 0, 1, 1) or status[0][-1] <= 0):
        raise ValueError("Missing original control-IN TFRC IRQ/RTI/WAI and two replays")
    host = [tuple(map(int, row)) for row in re.findall(
        r"^CONFIGURATION_HOST_STATUS (\d+) (\d+) (\d+) (\d+)$", text, re.M)]
    if host != [(7, 0, 0, 0x3000)] * 3:
        raise ValueError("Missing actual zero-length status/ACK host completion or literal TFRC/TCIE")
    if re.search(r"^CONFIGURATION_.*FRONTIER", text, re.M):
        raise ValueError("The old unsupported configuration-status frontier must not remain")
    frontiers = re.findall(r"^MIDI_FRONTIER (IN|OUT) (.+)$", text, re.M)
    if len(frontiers) != 2 or {kind for kind, _ in frontiers} != {"IN", "OUT"}:
        raise ValueError("Missing original MIDI descriptor failure evidence")
    for kind, error in frontiers:
        expected = "physical-endpoint=2 description=0x820008d1" if kind == "IN" else (
            "physical-endpoint=3 description=0x820008c1")
        if "Reserved USB local endpoint NISOF field " + expected != error:
            raise ValueError("A different failure replaced the original NISOF boundary")
    if any(row[-1] <= 0 for row in resets + replays + configurations):
        raise ValueError("A genuine original instruction path is required")
    setup_cases = rows("SETUP_CASE")
    setup_replays = rows("SETUP_REPLAY")
    profiles = [(0, 18), *((1, length) for length in (0, 1, 8, 9, 17, 18, 65535))]
    if len(setup_cases) != len(profiles) or [row[:2] for row in setup_cases] != profiles:
        raise ValueError("Incomplete original ordinary/overrun SETUP length matrix")
    for row in setup_cases:
        overrun, length, native, entries, returns, size = row
        if entries != 1 + overrun or returns != overrun or size != (min(length, 18) if overrun else 0):
            raise ValueError("Wrong original SETUP/RTI/descriptor branch outcome")
        if native <= 0 or (not overrun and native != 9):
            raise ValueError("Missing real original SETUP instructions before the retained fault")
    if setup_replays != [row for row in setup_cases for _ in range(2)]:
        raise ValueError("Missing sixteen joint pre-read SETUP restores")
    probe = [tuple(map(int, row)) for row in re.findall(
        r"^DESCRIPTOR_PROBE_UNAVAILABLE (\d+) (\d+) (\d+)$", text, re.M)]
    if probe != [(9, 0xbc3d, 0x90)] * 3:
        raise ValueError("Missing original descriptor-probe refusal and two joint restores")
    runtime = rows("USBIP_RUNTIME_REFUSAL")
    if runtime != [(index, 9, 0xbc3d, 0x90, 12, 1) for index in range(3)]:
        raise ValueError("Missing production runtime original refusal, zero-device TCP reply or restore retirement")
    setup_instructions = sum(row[2] for row in setup_cases + setup_replays)
    reset_instructions = sum(row[-1] for row in resets + replays + configurations)
    return {**validate_suspend(text), "setup_profiles": len(setup_cases), "setup_replays": len(setup_replays),
            "descriptor_deliveries": sum(row[0] for row in setup_cases + setup_replays),
            "descriptor_bytes": sum(row[5] for row in setup_cases + setup_replays),
            "setup_native_instructions": setup_instructions,
            "reset_configuration_instructions": reset_instructions,
            "reset_profiles": len(resets), "replays": len(replays),
            "configuration_irqs": len(configurations), "configuration_replays": 2,
            "configuration_host_completions": len(host),
            "configuration_status_irqs": len(status),
            "configuration_status_instructions": sum(row[-1] for row in status),
            "original_descriptor_probe_refusals": len(probe),
            "original_runtime_refusals": len(runtime),
            "original_runtime_restore_retirements": sum(row[5] for row in runtime),
            "runtime_native_instructions": sum(row[1] for row in runtime),
            "reserved_midi_failures": len(frontiers),
            "original_delay_calls": sum(row[2] for row in resets),
            "native_instructions": reset_instructions + setup_instructions +
            sum(row[-1] for row in status) + sum(row[1] for row in runtime)}


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print("SKIP original USB reset: private firmware absent")
        return 77
    reference = json.loads((ROOT / "doc/usb-reset-reference.json").read_text())
    original_mcu(firmware, reference)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix(".json")
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=280)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix(".log").write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            "AddressSanitizer", "LeakSanitizer", "UndefinedBehaviorSanitizer", "ThreadSanitizer", "runtime error:")):
        raise ValueError(f"Original USB reset execution failed: {diagnostics}")
    totals = validate(trace.read_text())
    report.write_text(json.dumps({"passed": True, **totals, "scope": reference["qualification"],
                                  "fuzz_execution": False, "physical_traffic": False}, indent=2) + "\n")
    print("Verified original cold-start USB reset/configuration and status-IN IRQs, actual "
          "DATA1/ZLP/ACK host completion and two joint restores, "
          "eight original SETUP branches, seven conditional descriptor sizes and sixteen "
          "pre-read restores. Ordinary/chained SETUP retains its invalid-USTB boundary; "
          "original MIDI endpoints retain their reserved NISOF failure. Three fresh production runtime "
          "attempts return exact zero-device TCP lists and retire on trusted restore. "
          "no successful MIDI, physical USB timing, full board boot or fuzzing claim.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
