# SPDX-License-Identifier: GPL-3.0-only
"""Run the original Desktop SL instrument through a captured JACK session."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time

A013 = bytes.fromhex("7184608a4c908ac410fb44608a44108a4490898460")

def isolate_profile(profile, directory):
    fields = {}
    for line in profile.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        key, value = shlex.split(line)
        fields[key] = (profile.parent / value).resolve()
    for key in ("device-state", "nand"):
        fields[key] = Path(shutil.copyfile(fields[key], directory / key))
    isolated = directory / "live.profile"
    isolated.write_text("".join(f"{key} {json.dumps(str(value), ensure_ascii=False)}\n"
                                for key, value in fields.items()))
    return isolated

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary_dir", type=Path)
    parser.add_argument("profile", type=Path)
    args = parser.parse_args()
    ports = subprocess.check_output(["jack_lsp"], text=True, timeout=4)
    if "blofeld-emu:midi_in" in ports.splitlines():
        parser.error("Close the existing Blofeld emulator before starting this test")
    inputs = Path(__file__).with_name("nachtmusik-jack.events")
    scratch = Path(__file__).resolve().parents[3] / "build/local"
    scratch.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix="nachtmusik-jack-", dir=scratch))
    profile = isolate_profile(args.profile, directory)
    runtime = directory / "runtime"
    runtime.mkdir(mode=0o700)
    log = directory / "emulator.log"
    with log.open("w") as output:
        emulator = subprocess.Popen(
            [args.binary_dir / "blofeld-headless", "--profile", profile],
            env=os.environ | {"XDG_RUNTIME_DIR": str(runtime)}, stdout=output, stderr=output)
    try:
        until = time.monotonic() + 15
        while "Live MIDI input active:" not in log.read_text():
            if emulator.poll() is not None or time.monotonic() >= until:
                raise RuntimeError(f"Original firmware did not become playable. See {log}")
            time.sleep(0.04)
        replay = subprocess.run([args.binary_dir / "jack-nachtmusik", inputs], timeout=43)
        display = (runtime / "blofeld-emu" / "display.pbm").read_bytes()
        glyph = b"".join(display[10+16*row:13+16*row] for row in range(11, 18))
    finally:
        if emulator.poll() is None:
            emulator.send_signal(signal.SIGINT)
            try:
                emulator.wait(timeout=5)
            except subprocess.TimeoutExpired:
                emulator.kill()
                emulator.wait(timeout=3)
    counters = re.findall(r"underruns=(\d+)/(\d+) playing=", log.read_text())
    print(f"A013 selected={glyph == A013}, host/guest underruns="
          f"{'/'.join(counters[-1]) if counters else 'unavailable'}. Log: {log}")
    return int(replay.returncode != 0 or glyph != A013 or emulator.returncode != 0 or
               not counters or any(pair != ("0", "0") for pair in counters))

if __name__ == "__main__":
    raise SystemExit(main())
