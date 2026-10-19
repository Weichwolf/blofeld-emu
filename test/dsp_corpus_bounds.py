# SPDX-License-Identifier: GPL-3.0-only
"""Check distinct program-size and dynamic-step bounds with an own one-word NOP."""
from pathlib import Path
import subprocess
import sys
import tempfile

from dsp_hardware_corpus import parse_dump


def check_case(binary, directory, start, end, limit, accepted):
    (directory / "corpus.meta").write_text(f"case own {start:04x} {end:04x} {limit}\n")
    (directory / "corpus.lod").write_text("P 0100 000000\n")
    target = directory / "native.txt"
    result = subprocess.run([str(binary), str(directory), str(target)],
                            capture_output=True, text=True, check=False)
    if accepted:
        if result.returncode:
            raise AssertionError(f"Valid DSP corpus step bound rejected: {limit}: {result.stderr}")
        dump = parse_dump(target)
        if list(dump) != ["own"] or dump["own"]["pc"] != 0x101:
            raise AssertionError("Own NOP did not retire at its exact independent end PC")
    elif result.returncode == 0 or "Unsupported DSP corpus case" not in result.stderr:
        raise AssertionError(f"Invalid DSP corpus range/bound accepted: {start}/{end}/{limit}")


def main():
    binary = Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="blofeld-corpus-bounds-") as name:
        directory = Path(name)
        for limit in (1, 4096, 70000, 100000):
            check_case(binary, directory, 0x100, 0x101, limit, True)
        for start, end, limit in ((0x100, 0x101, 0), (0x100, 0x101, 100001),
                                  (0x100, 0x100, 1), (0x1000, 0x1001, 1)):
            check_case(binary, directory, start, end, limit, False)
    print("DSP corpus program/dynamic-step bounds: four positive and four negative cases passed")


if __name__ == "__main__":
    main()
