# SPDX-License-Identifier: GPL-3.0-only
"""Original cold reset through the actual CLI, explicit inputs and strict DSP fault."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from extract_runtime import extract

# A nominal reference experiment, not hardware-verified wiring or ready timing.
# Inputs are supplied by boundary timestamps; the CLI never branches on firmware PCs.
CONFIG = """blofeld-reference-board 2
dsp-execution reference-effects
clock 12000000 0 12000000 540000000 reference-boundaries
ordering dsp-before-mcu dsp-before-mcu 16
spi-b 02 04 10 clock-before-select
dsp-release 87317172 low
operations 2000000
initial drive E fb 03 00
initial drive Q ff 00 00
initial drive R 04 04 00
initial drive U 3f 00 00
loopback E 04 E 08
at 6120 phy-window 15
at 4212846 drive Q 00 00 ff
at 4212846 usb-decoder 08
at 4212846 drive E eb 03 10
at 4212846 drive B e8 00 00
at 4213890 drive T 01 01 00
at 4213890 timer-inputs 01
at 87322572 drive U 3f 01 00
"""


def identity(path):
    data = path.read_bytes()
    return {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def audit(trace, firmware):
    frames = []
    bits = []
    selected, clock = False, True
    spi_writes = dsp_periods = dsp_retirements = 0
    previous_dsp_end = 87317172 + 2 * 45
    last_mcu = ""
    with trace.open() as stream:
        for row in stream:
            if row.startswith("MCU end="):
                last_mcu = row
            elif row.startswith("DSP-RELEASE "):
                assert row.strip() == "DSP-RELEASE requested=87317172 actual=87317172 pc=0xff0000"
            elif row.startswith("DSP-PIPELINE "):
                fields = dict(field.split("=") for field in row.split()[1:])
                end, start, cycle = (int(fields[key]) for key in ("end", "start", "cycle"))
                # The configured12MHz EXTAL gives45 common ticks. The original
                # PCTL setup changes the native reference clock to10 ticks.
                # This checks continuous real periods, not old nominal costs.
                assert start == previous_dsp_end and end - start in (45, 10)
                assert cycle == dsp_periods
                if dsp_periods == 0:
                    assert fields["f1"] == str(0xff0000) and fields["f1-known"] == "1"
                previous_dsp_end = end
                dsp_periods += 1
                dsp_retirements += int(fields["retirement"])
            elif row.startswith("SPI end="):
                match = re.search(r"value=(0x[0-9a-f]+)", row)
                assert match is not None
                value = int(match[1], 16)
                next_selected, next_clock = not bool(value & 2), bool(value & 4)
                # Explicit clock-before-select: the falling edge sees the old SS.
                if selected and clock and not next_clock:
                    bits.append(int(bool(value & 16)))
                if selected and not next_selected:
                    assert len(bits) == 24, len(bits)
                    frames.append(sum(bit << (23 - index) for index, bit in enumerate(bits)))
                    bits = []
                selected, clock = next_selected, next_clock
                spi_writes += 1
    assert not bits and len(frames) == 97 and spi_writes == 7275
    assert dsp_periods > 0 and dsp_retirements > 0
    assert "pc=0xb9ad " in last_mcu
    last_mcu_end = int(last_mcu.split()[1].split("=")[1])
    assert last_mcu_end <= previous_dsp_end < last_mcu_end + 90
    with tempfile.TemporaryDirectory() as temporary:
        private = Path(temporary)
        extract(firmware, private)
        mcu = (private / "host-mcu-8000.bin").read_bytes()
    payload = b"".join(word.to_bytes(3, "big") for word in frames)
    assert frames[:2] == [94, 0x6a]
    assert payload == mcu[0x7875:0x7875 + 97 * 3]
    return {"spi_writes": spi_writes, "frames": len(frames), "dsp_periods": dsp_periods,
            "dsp_retirements": dsp_retirements,
            "last_completed_common_ticks": previous_dsp_end,
            "frame_sha256": hashlib.sha256(payload).hexdigest(), "trace": identity(trace)}


def main():
    binary, firmware, rom, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not rom.is_file():
        print("SKIP original CLI reference: private firmware/ROM absent")
        return 77
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"] if row["file"] == firmware.name)
    assert identity(firmware) == {"bytes": 102986, "sha256": expected}
    assert identity(rom) == {"bytes": 165,
                           "sha256": "26531751e85d9d796795e443869330c84689b6bdb1eccbafc2cbd1554e377f16"}
    trace.parent.mkdir(parents=True, exist_ok=True)
    config = trace.with_suffix(".bfboard")
    config.write_text(CONFIG)
    result = subprocess.run([str(binary.resolve()), "--firmware", str(firmware.resolve()),
                             "--reference-board", str(config.resolve()), "--dsp-rom", str(rom.resolve()),
                             "--instructions", "2000000", "--trace", str(trace.resolve())],
                            capture_output=True, text=True, timeout=180, check=False)
    trace.with_suffix(".log").write_text(result.stdout + result.stderr)
    assert result.returncode == 2, result.stderr
    # Native write context names the executing instruction, not a legacy
    # pre-incremented register PC or an unperformed estimated completion.
    for expected in ("Unmapped DSP memory write at Y:0x002000", "HC12 cycle execution pc=0xb9ad",
                     "pc=0x000072 omr=0x600389", "r4=8193 lc=122880",
                     "spi-writes=7275 frames=97"):
        assert expected in result.stderr, (expected, result.stderr)
    observed = audit(trace, firmware)
    assert f"completed-steps={observed['dsp_retirements']} " in result.stderr
    assert f"pipeline-cycles={observed['dsp_periods']} " in result.stderr
    assert f"ticks={observed['last_completed_common_ticks']} timebase=540000000" in result.stderr
    report = {"firmware": identity(firmware), "rom": identity(rom), "config": identity(config),
              "binary": identity(binary), "returncode": result.returncode, **observed}
    trace.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print("Actual original CLI: native reset, 97 SPI frames and strict reserved Y write verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
