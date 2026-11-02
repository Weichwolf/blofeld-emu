# SPDX-License-Identifier: GPL-3.0-only
"""Ordinary CLI, owned ALSA tools and authored native SCI receive/transmit."""

import hashlib
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import time

from test_reference_cli import CONFIG, ROM, image

EXPECTED_ZIP = "095c760a576dcb04fe3e379bed1b95a0360002295ab42b29e3335a39514f0c72"
OPERATIONS = 250000


def require(condition, message):
    if not condition:
        raise ValueError(message)


def native_code(small):
    # Authored LDS, MODRR, SCI divisor12/TE+RE and RDRF polling. Only an actual
    # received F8 can reach the native transmit instructions. No injected ACK.
    code = bytearray.fromhex("cf2000 86" + ("01" if small else "00") +
                             " 7a0257 cc000c 7c00c8 860c 7a00cb")
    poll = len(code)
    code += bytes.fromhex("b600cc 8520 27f9 b600cf 7a0260 81f8 2600")
    code[-1] = (poll - len(code)) & 255
    for byte in bytes.fromhex("f8 f0 7d 01 02 03 f7"):
        code += bytes.fromhex("b600cc 8580 27f9 86") + bytes([byte]) + bytes.fromhex("7a00cf")
    code += bytes.fromhex("3e20fd")
    return code


def configuration(small):
    receive, transmit, port, mask = ("PS4", "PS5", "S", "10") if small else (
        "PR2", "PR3", "R", "04")
    return (CONFIG.replace("operations 23", f"operations {OPERATIONS}")
            .replace("dsp-release 450 low", "dsp-release 18446744073709551615 low") +
            f"mcu-power-on 3 high\ndsp-input-order input-before-dsp 4\n"
            f"initial drive {port} {mask} {mask} 00\n"
            f"midi-sci-timed {receive} after 0\nmidi-transfer-reference 12 0\n"
            f"midi-output {transmit}\n")


class Lines:
    def __init__(self, stream):
        self.stream = stream
        self.pending = bytearray()
        self.seen = bytearray()

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while True:
            while b"\n" in self.pending:
                line, _, self.pending = self.pending.partition(b"\n")
                decoded = line.decode() + "\n"
                if predicate(decoded):
                    return decoded
            remaining = deadline - time.monotonic()
            require(remaining > 0 and select.select([self.stream], [], [], remaining)[0],
                    "Timed out observing the owned process")
            data = os.read(self.stream.fileno(), 8192)
            require(data, "Owned process exited before its readiness observation")
            self.pending.extend(data)
            self.seen.extend(data)


def exchange(binary, firmware, folder, small):
    folder.mkdir(parents=True, exist_ok=True)
    board, rom, flash, trace = [folder / p for p in
                               ("board.bfboard", "rom.bin", "mcu.bfnvm", "run.trace")]
    board.write_text(configuration(small))
    rom.write_bytes(ROM)
    flash.write_bytes(image(native_code(small)))
    name = "Blofeld CLI MIDI " + ("PS5" if small else "PR3")
    inputs = {p: p.read_bytes() for p in (binary, firmware, board, rom, flash)}
    environment = os.environ.copy()
    environment["LC_ALL"] = "C"
    command = [str(binary), "--firmware", str(firmware), "--reference-board", str(board),
               "--dsp-rom", str(rom), "--flash-in", str(flash), "--trace", str(trace),
               "--instructions", str(OPERATIONS), "--midi-port", name]
    child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             env=environment)
    stdout = Lines(child.stdout)
    observer = None
    observed, observer_error = b"", b""
    try:
        ready = stdout.until(lambda line: line.startswith("MIDI port="))
        match = re.fullmatch(r'MIDI port=(\d+):(\d+) name="' + re.escape(name) +
                             r'" protocol=(\d+) input-events=(\d+) output-events=(\d+) '
                             r'period-ticks=540000\n', ready)
        require(match and all(int(match[i]) > 0 for i in (3, 4, 5)),
                "CLI must publish its actual address and positive negotiated grants")
        target = f"{match[1]}:{match[2]}"
        discovered = subprocess.check_output(["aconnect", "-l"], text=True, env=environment)
        require(any(line.startswith(f"client {match[1]}:") and f"'{name}'" in line
                    for line in discovered.splitlines()), "Published endpoint is not the owned client")
        observer = subprocess.Popen(["stdbuf", "-oL", "aseqdump", "-u", "0", "-p", target],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=environment)
        records = Lines(observer.stdout)
        records.until(lambda line: "Source" in line and "Event" in line)
        sent = subprocess.run(["aseqsend", "-u", "0", "-p", target, "f8"],
                              capture_output=True, text=True, timeout=10, check=False,
                              env=environment)
        require(sent.returncode == 0, sent.stdout + sent.stderr)
        tail, error = child.communicate(timeout=120)
        full = bytes(stdout.seen) + tail
        (folder / "stdout.txt").write_bytes(full)
        (folder / "stderr.txt").write_bytes(error)
        require(child.returncode == 0 and not error, full.decode() + error.decode())
        require(b"MIDI accepted=1 queued=0 started=1" in full and
                b"MIDI session closed=1" in full, "One native input and explicit retirement required")
        listing = trace.read_text()
        require(re.findall(r"^WRITE cycle=\d+ address=0x0260 value=0x([0-9a-f]{2}) ",
                           listing, re.M) == ["f8"], "Native SCI must actually read the tool's F8")
        outputs = re.findall(r"^MIDI-OUTPUT start=\d+ stop-sample=\d+ value=0x([0-9a-f]{2})$",
                             listing, re.M)
        require(bytes.fromhex("".join(outputs)) == bytes.fromhex("f8 f0 7d 01 02 03 f7"),
                "Only the authored native SCI reply may leave the modeled output")
        observer.send_signal(signal.SIGINT)
        observed, observer_error = observer.communicate(timeout=10)
        observed = bytes(records.seen) + observed
        (folder / "aseqdump.txt").write_bytes(observed)
        (folder / "aseqdump.stderr.txt").write_bytes(observer_error)
        require(not observer_error, observer_error.decode())
        text = observed.decode()
        fragments = re.findall(r"System exclusive[^\S\n]+([0-9a-f ]+)", text, re.I)
        require(len(re.findall(r"\bClock\b", text)) == 1 and
                bytes.fromhex(" ".join(fragments)) == bytes.fromhex("f0 7d 01 02 03 f7"),
                "Independent aseqdump must see one clock and the complete literal SysEx: " + text)
        after = subprocess.check_output(["aconnect", "-l"], text=True, env=environment)
        require(not any(line.startswith(f"client {match[1]}:") and f"'{name}'" in line
                        for line in after.splitlines()), "Owned endpoint survived natural CLI completion")
        require(all(p.read_bytes() == data for p, data in inputs.items()), "An input was modified")
        print("Ordinary CLI bidirectional native SCI / standard ALSA tools passed:", name)
    finally:
        if child.poll() is None:
            # Preserve the bounded guest's natural outcome even if observation fails.
            try:
                tail, error = child.communicate(timeout=120)
                (folder / "failed-stdout.txt").write_bytes(bytes(stdout.seen) + tail)
                (folder / "failed-stderr.txt").write_bytes(error)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.communicate(timeout=10)
                raise
        if observer is not None and observer.poll() is None:
            observer.send_signal(signal.SIGINT)
            observed, observer_error = observer.communicate(timeout=10)
            (folder / "failed-aseqdump.txt").write_bytes(observed)
            (folder / "failed-aseqdump.stderr.txt").write_bytes(observer_error)


def refusals(binary, firmware, folder):
    folder.mkdir(parents=True, exist_ok=True)
    base = [str(binary), "--firmware", "absent.zip"]
    shared = ["--reference-board", "absent.bfboard", "--dsp-rom", "absent.bin"]
    cases = [(["--midi-port", "name"], "MIDI requires"),
             (shared + ["--midi-port", "name", "--inspect"], "conflicts"),
             (shared + ["--midi-port", "name", "--midi-file", "unused"], "MIDI requires"),
             (shared + ["--midi-port", "name", "--midi-device", "unused"], "MIDI requires"),
             (shared + ["--midi-port", ""], "MIDI port name"),
             (shared + ["--midi-port", "x" * 64], "MIDI port name")]
    for index, (arguments, expected) in enumerate(cases):
        result = subprocess.run(base + arguments, capture_output=True, text=True, timeout=15)
        (folder / f"options-{index}.txt").write_text(result.stdout + result.stderr)
        require(result.returncode == 1 and expected in result.stderr, result.stdout + result.stderr)
    board, rom, trace = folder / "board.bfboard", folder / "rom.bin", folder / "never.trace"
    rom.write_bytes(ROM)
    texts = [(configuration(False).replace("midi-output PR3\n", ""), "midi-output"),
             (configuration(False).replace("midi-sci-timed PR2 after 0\n", "")
              .replace("midi-transfer-reference 12 0\n", ""), "Host MIDI requires"),
             (configuration(False) + "checkpoint 100 2\n", "rejects automatic checkpoint"),
             (configuration(False).replace("midi-sci-timed PR2", "midi-sci-timed UNKNOWN"),
              "PR2 or PS4"),
             (configuration(False).replace("mcu-power-on 3 high\n", ""), "Timed SCI MIDI requires"),
             (configuration(False).replace("dsp-input-order input-before-dsp 4\n", ""),
              "Timed SCI MIDI requires"),
             (configuration(False) + "midi-sci PR2 6000000 after 0\n", "Timed SCI MIDI requires"),
             (configuration(False) + "midi-sci-timed PR2 after 0\n", "Duplicate")]
    for index, (text, expected) in enumerate(texts):
        board.write_text(text)
        result = subprocess.run([str(binary), "--firmware", str(firmware), "--reference-board",
                                 str(board), "--dsp-rom", str(rom), "--midi-port", "name",
                                 "--trace", str(trace)], capture_output=True, text=True, timeout=15)
        (folder / f"board-{index}.txt").write_text(result.stdout + result.stderr)
        require(result.returncode == 1 and expected in result.stderr and not trace.exists(),
                result.stdout + result.stderr)


def main():
    binary, firmware, artifacts = (p.resolve() for p in map(Path, sys.argv[1:]))
    for tool in ("aseqsend", "aseqdump", "aconnect", "stdbuf"):
        if shutil.which(tool) is None:
            print("SKIP MIDI session CLI: standard tool absent:", tool)
            return 77
    if not firmware.is_file() or not os.access("/dev/snd/seq", os.R_OK | os.W_OK):
        print("SKIP MIDI session CLI: private firmware or ALSA sequencer unavailable")
        return 77
    require(hashlib.sha256(firmware.read_bytes()).hexdigest() == EXPECTED_ZIP,
            "Private original firmware identity differs")
    refusals(binary, firmware, artifacts / "refusals")
    for small in (False, True):
        exchange(binary, firmware, artifacts / ("ps5" if small else "pr3"), small)
    return 0


if __name__ == "__main__":
    sys.exit(main())
