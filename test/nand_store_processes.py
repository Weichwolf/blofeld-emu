# SPDX-License-Identifier: GPL-3.0-only
"""Kill native original-helper workers at real NAND state publication phases."""

import binascii
import hashlib
import json
from pathlib import Path
import re
import select
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import time
import unittest

WORKER = Path(sys.argv[1]).resolve()
FIRMWARE = Path(sys.argv[2]).resolve()
REPORT = Path(sys.argv[3]).resolve()
PAGE_BYTES = 528
ROW = 37
BLOCK = 1
NEIGHBOR = 64
CASES = []
COMPLETED = []


def encoded(program, stage, fail=False):
    """Independent sparse encoding; no production encoder or extracted bytes."""
    uncertain = stage == "uncertain" or (stage == "completed" and fail)
    known_flags, flags = (0, 0) if uncertain else (7, 1)
    blocks = struct.pack("<HBB", BLOCK, known_flags, flags)
    payload = b"".join((((index * 29) + 0x123) & 0x7ff).to_bytes(3, "big")
                       for index in range(176))
    pages = bytearray()
    for page in range(32, 65):
        history = None
        if page == NEIGHBOR:
            data = bytes([0x42]) + bytes(PAGE_BYTES - 1)
            known = bytes([255]) + bytes(PAGE_BYTES - 1)
        elif uncertain and (not program or page == ROW):
            data = known = bytes(PAGE_BYTES)
        elif stage == "completed" and not fail:
            data = payload if program and page == ROW else bytes([255]) * PAGE_BYTES
            known = bytes([255]) * PAGE_BYTES
            history = (1, 1) if program and page == ROW else (0, 0)
        else:
            data = bytes([255 if program else 0x91]) * PAGE_BYTES
            known = bytes([255]) * PAGE_BYTES
            history = (0, 0) if program else None
        main, spare = history if history is not None else (255, 255)
        pages.extend(struct.pack("<IBBH", page, main, spare, 0))
        pages.extend(value for pair in zip(data, known) for value in pair)
    header = b"BLFNAND\n" + struct.pack("<IIIIII", 1, 1, 33, 1, PAGE_BYTES, 0)
    body = header + blocks + pages
    return body + struct.pack("<I", binascii.crc32(body))


def command(path, program, fail, transaction, phase):
    return [WORKER, FIRMWARE, path, str(int(program)), str(int(fail)),
            str(transaction), str(phase)]


def wait_for_phase(child, transaction, phase):
    deadline = time.monotonic() + 20
    output = bytearray()
    while time.monotonic() < deadline:
        ready, _, _ = select.select([child.stdout], [], [], max(0, deadline - time.monotonic()))
        if not ready:
            break
        line = child.stdout.readline()
        if not line:
            break
        output.extend(line)
        if line.startswith(f"NAND_FILE {transaction} {phase} ".encode()):
            return line.decode().strip()
    raise AssertionError(f"Worker did not reach its requested native publication: {output!r}")


class NandStoreProcesses(unittest.TestCase):
    def test_native_process_death_at_every_publication_phase(self):
        for program in (False, True):
            for fail in (False, True):
                for transaction in range(2):
                    for phase in range(5):
                        with self.subTest(program=program, fail=fail, transaction=transaction,
                                          phase=phase), tempfile.TemporaryDirectory() as directory:
                            path = Path(directory) / "state.bfnand"
                            initial = encoded(program, "initial")
                            uncertain = encoded(program, "uncertain")
                            previous = initial if transaction == 0 else uncertain
                            following = uncertain if transaction == 0 else encoded(program, "completed", fail)
                            path.write_bytes(initial)
                            path.chmod(0o600)
                            child = subprocess.Popen(command(path, program, fail, transaction, phase),
                                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                                     stderr=subprocess.PIPE, bufsize=0)
                            try:
                                acknowledgement = wait_for_phase(child, transaction, phase)
                                child.kill()
                                _, errors = child.communicate(timeout=10)
                                self.assertEqual(child.returncode, -signal.SIGKILL, errors.decode())
                            finally:
                                if child.poll() is None:
                                    child.kill()
                                    child.communicate(timeout=10)
                            match = re.fullmatch(r"NAND_FILE (\d+) (\d+) pc=(0x[0-9a-f]+) "
                                                 r"source=(0x[0-9a-f]+) pending=(\d+) ready=([01])",
                                                 acknowledgement)
                            self.assertIsNotNone(match)
                            pc = int(match[3], 16)
                            self.assertGreaterEqual(pc, 0x2e00)
                            self.assertLess(pc, 0x2f90)
                            self.assertEqual(int(match[4], 16), 0xc0b0 if program else 0xc000)
                            self.assertEqual(int(match[5]), (3 if program else 4) if transaction else 0)
                            self.assertEqual(int(match[6]), 0 if transaction else 1)
                            observed = path.read_bytes()
                            wanted = previous if phase < 3 else following
                            self.assertEqual(observed, wanted)
                            self.assertEqual(len(observed), 35152)
                            self.assertEqual(binascii.crc32(observed[:-4]),
                                             struct.unpack("<I", observed[-4:])[0])
                            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
                            self.assertEqual(stat.S_IMODE(Path(str(path) + ".lock").stat().st_mode), 0o600)
                            temporaries = list(Path(directory).glob("*.tmp.*"))
                            self.assertEqual(len(temporaries), 1 if phase < 3 else 0)
                            for temporary in temporaries:
                                self.assertEqual(stat.S_IMODE(temporary.stat().st_mode), 0o600)
                                self.assertEqual(temporary.read_bytes(), b"" if phase == 0 else following)
                            probe = subprocess.run(command(path, True, False, 2, 4), input=b"resume",
                                                   capture_output=True, timeout=20, check=False)
                            permits_program = (transaction == 0 and phase < 3 and program) or (
                                transaction == 1 and phase >= 3 and not program and not fail)
                            self.assertEqual(probe.returncode, 0 if permits_program else 1,
                                             probe.stderr.decode())
                            if permits_program:
                                self.assertEqual(path.read_bytes(), encoded(True, "completed"))
                            else:
                                self.assertEqual(path.read_bytes(), wanted)
                                reason = "main partial-program limit" if (
                                    transaction == 1 and phase >= 3 and program and not fail
                                ) else "unknown/invalid block condition"
                                # Before an erase starts, valid old data has unknown program history.
                                if transaction == 0 and phase < 3 and not program:
                                    reason = "unknown program history"
                                self.assertIn(reason.encode(), probe.stderr)
                            CASES.append({"program": program, "fail": fail, "transaction": transaction,
                                          "phase": phase, "ack": acknowledgement,
                                          "sha256": hashlib.sha256(observed).hexdigest(),
                                          "probe_program_permitted": permits_program})

    def test_completed_native_outcomes_are_retained(self):
        for program in (False, True):
            for fail in (False, True):
                with self.subTest(program=program, fail=fail), tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "state.bfnand"
                    path.write_bytes(encoded(program, "initial"))
                    path.chmod(0o600)
                    result = subprocess.run(command(path, program, fail, 2, 4), input=b"resume",
                                            capture_output=True, timeout=20, check=False)
                    self.assertEqual(result.returncode, 0, result.stderr.decode())
                    self.assertEqual(path.read_bytes(), encoded(program, "completed", fail))
                    self.assertEqual(list(Path(directory).glob("*.tmp.*")), [])
                    COMPLETED.append({"program": program, "fail": fail})


if __name__ == "__main__":
    if not FIRMWARE.is_file():
        print("SKIP original persistent NAND reference: private firmware absent")
        sys.exit(77)
    manifest = json.loads((Path(__file__).resolve().parents[1] / "doc/downloads.json").read_text())
    expected = next(row["sha256"] for row in manifest["files"]
                    if row["file"] == "Blofeld_Firmware_V1.25.zip")
    if hashlib.sha256(FIRMWARE.read_bytes()).hexdigest() != expected:
        raise ValueError("Original persistent NAND firmware identity mismatch")
    result = unittest.main(argv=[sys.argv[0]], exit=False).result
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps({"passed": result.wasSuccessful(), "cases": CASES,
                                  "completed": COMPLETED}, indent=2) + "\n")
    sys.exit(0 if result.wasSuccessful() else 1)
