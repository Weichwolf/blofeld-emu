# SPDX-License-Identifier: GPL-3.0-only
"""Check host file publication across process death with an independent encoding."""

import binascii
from pathlib import Path
import select
import signal
import stat
import struct
import subprocess
import sys
import tempfile
import unittest

WORKER = Path(sys.argv[1]).resolve()


def encoded(pattern):
    payload = bytes([pattern]) * 32768 + bytes([255]) * 4096 + bytes(2048)
    data = b"BLFNVM\r\n" + struct.pack("<II", 1, 38912) + payload
    return data + struct.pack("<I", binascii.crc32(data))


class FlashFileProcesses(unittest.TestCase):
    def test_process_death_at_each_publication_phase(self):
        for phase in range(5):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "state.bfnvm"
                previous, following = encoded(165), encoded(90)
                path.write_bytes(previous)
                path.chmod(0o600)
                child = subprocess.Popen([WORKER, path, "90", str(phase)],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.PIPE)
                try:
                    ready, _, _ = select.select([child.stdout], [], [], 10)
                    self.assertTrue(ready, "Worker did not reach the requested real file phase")
                    self.assertEqual(child.stdout.readline(), f"{phase}\n".encode())
                    child.kill()
                    _, errors = child.communicate(timeout=10)
                    self.assertEqual(child.returncode, -signal.SIGKILL, errors.decode())
                finally:
                    if child.poll() is None:
                        child.kill()
                        child.communicate(timeout=10)
                observed = path.read_bytes()
                self.assertEqual(observed, previous if phase < 3 else following)
                self.assertEqual(len(observed), 38932)
                self.assertEqual(binascii.crc32(observed[:-4]), struct.unpack("<I", observed[-4:])[0])
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
                temporaries = list(Path(directory).glob("*.tmp.*"))
                self.assertEqual(len(temporaries), 1 if phase < 3 else 0)
                for temporary in temporaries:
                    self.assertEqual(stat.S_IMODE(temporary.stat().st_mode), 0o600)
                    self.assertEqual(temporary.read_bytes(), b"" if phase == 0 else following)

    def test_complete_replacement_and_failed_publish_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "state.bfnvm"
            path.write_bytes(encoded(165))
            result = subprocess.run([WORKER, path, "90", "4"], input=b"resume",
                                    capture_output=True, timeout=15, check=False)
            self.assertEqual(result.returncode, 0, result.stderr.decode())
            self.assertEqual(path.read_bytes(), encoded(90))
            self.assertEqual(list(root.glob("*.tmp.*")), [])
            destination = root / "directory.bfnvm"
            destination.mkdir()
            result = subprocess.run([WORKER, destination, "90", "4"], input=b"resume",
                                    capture_output=True, timeout=15, check=False)
            self.assertEqual(result.returncode, 1)
            self.assertTrue(destination.is_dir())
            self.assertEqual(list(root.glob("*.tmp.*")), [])
            self.assertEqual(path.read_bytes(), encoded(90))

    def test_symlink_and_invalid_extension_preserve_target(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "target.bfnvm"
            original = encoded(165)
            target.write_bytes(original)
            link = root / "link.bfnvm"
            link.symlink_to(target)
            for path in (link, root / "wrong-extension.bin"):
                result = subprocess.run([WORKER, path, "90", "4"], input=b"resume",
                                        capture_output=True, timeout=15, check=False)
                self.assertEqual(result.returncode, 1)
                self.assertEqual(target.read_bytes(), original)
            self.assertTrue(link.is_symlink())


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
