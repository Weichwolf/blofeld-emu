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
CONFIG = """blofeld-reference-board 1
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
    spi_writes = dsp_attempts = 0
    previous_dsp_end = 0
    last_mcu = ""
    with trace.open() as stream:
        for row in stream:
            if row.startswith("MCU end="):
                last_mcu = row
            elif row.startswith("DSP end="):
                fields = dict(field.split("=") for field in row.split()[1:])
                end, start, ticks = (int(fields[key]) for key in ("end", "start", "ticks"))
                assert end == 87317172 + start + ticks
                assert end > previous_dsp_end
                previous_dsp_end = end
                dsp_attempts += 1
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
    assert dsp_attempts == 59287 and previous_dsp_end == 96589457
    assert last_mcu.startswith("MCU end=96589440 pc=0xb9ad instructions=1857015 operations=1857015")
    with tempfile.TemporaryDirectory() as temporary:
        private = Path(temporary)
        extract(firmware, private)
        mcu = (private / "host-mcu-8000.bin").read_bytes()
    payload = b"".join(word.to_bytes(3, "big") for word in frames)
    assert frames[:2] == [94, 0x6a]
    assert payload == mcu[0x7875:0x7875 + 97 * 3]
    return {"spi_writes": spi_writes, "frames": len(frames), "dsp_attempts": dsp_attempts,
            "last_attempt_common_ticks": previous_dsp_end,
            "frame_sha256": hashlib.sha256(payload).hexdigest(), "trace": identity(trace)}


def main():
    binary, firmware, rom, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not rom.is_file():
        print("SKIP original CLI reference: private firmware/ROM absent")
        return 77
    manifest = json.loads((ROOT / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"] if row["file"] == firmware.name)
    assert identity(firmware) == {"bytes": 102986, "sha256": expected}
    assert identity(rom) == {"bytes": 174,
                           "sha256": "8d5193c52b22cfdd633bcdd2acf8651a498e361ff06e4716454fddf92be11901"}
    trace.parent.mkdir(parents=True, exist_ok=True)
    config = trace.with_suffix(".bfboard")
    config.write_text(CONFIG)
    result = subprocess.run([str(binary.resolve()), "--firmware", str(firmware.resolve()),
                             "--reference-board", str(config.resolve()), "--dsp-rom", str(rom.resolve()),
                             "--instructions", "2000000", "--trace", str(trace.resolve())],
                            capture_output=True, text=True, timeout=180, check=False)
    trace.with_suffix(".log").write_text(result.stdout + result.stderr)
    assert result.returncode == 2, result.stderr
    for expected in ("Unmapped DSP memory write at Y:0x002000", "mcu-pc=0xb9ad dsp-pc=0x000073",
                     "attempted-completion=96589457", "cycles=5365748 idle=0 ticks=10732160",
                     "completed-steps=59286 units=212426 ticks=9272275",
                     "spi-writes=7275 frames=97 ordered-pairs=1 ties=368", "r4=8193 lc=122880"):
        assert expected in result.stderr, (expected, result.stderr)
    report = {"firmware": identity(firmware), "rom": identity(rom), "config": identity(config),
              "binary": identity(binary), "returncode": result.returncode, **audit(trace, firmware)}
    trace.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    print("Actual original CLI: native reset, 97 SPI frames and strict reserved Y write verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
