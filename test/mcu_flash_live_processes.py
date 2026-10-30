# SPDX-License-Identifier: GPL-3.0-only
"""Directed own-ROM native durability, actual process death and independent wire checks."""
from pathlib import Path
import select
import signal
import os
import subprocess
import sys
import tempfile
import mcu_flash_live_wire as wire

NORMAL, QUEUE, WORKER = (Path(value).resolve() for value in sys.argv[1:])


def run(binary, arguments):
    child = subprocess.run([binary, *map(str, arguments)], capture_output=True, text=True, timeout=30)
    if child.returncode:
        raise RuntimeError(f"{binary.name} exit={child.returncode}: {child.stdout}{child.stderr}")
    return child.stdout


def kill_at(path, publication, phase):
    child = subprocess.Popen([WORKER, path, "kill", str(publication), str(phase)],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        assert select.select([child.stdout], [], [], 30)[0], "Worker did not reach file boundary"
        expected = f"READY pid={child.pid} publication={publication} phase={phase}\n".encode()
        assert child.stdout.readline() == expected, "Wrong native worker/file boundary"
        os.kill(child.pid, signal.SIGSTOP)
        waited, status = os.waitpid(child.pid, os.WUNTRACED)
        assert waited == child.pid and os.WIFSTOPPED(status)
        assert os.WSTOPSIG(status) == signal.SIGSTOP
        child.kill()
        output, errors = child.communicate(timeout=30)
        assert child.returncode == -signal.SIGKILL and not output and not errors
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=30)


def main():
    with tempfile.TemporaryDirectory(prefix="blofeld-live-flash-") as name:
        directory = Path(name)
        print(run(NORMAL, [directory]), end="")
        wire.normal_outputs(directory)
        print(run(QUEUE, [directory]), end="")
        wire.durable_outputs(directory)
        for mode in ("kill", "fault"):
            for publication in (1, 2):
                for phase in range(5):
                    path = directory / f"{mode}-{publication}-{phase}.bfnvm"
                    if mode == "kill":
                        kill_at(path, publication, phase)
                    else:
                        assert "fault blocked" in run(WORKER, [path, mode, publication, phase])
                    initial = wire.worker_initial()
                    wire.verify(str(path) + ".initial", initial)
                    previous = "initial" if publication == 1 else "unknown"
                    following = "unknown" if publication == 1 else "program"
                    wire.verify(path, wire.transition(initial, previous if phase < 3 else following))
                    before = path.read_bytes()
                    assert "cold native POR" in run(WORKER, [path, "read", 0, 0])
                    assert path.read_bytes() == before, "Cold native execution changed the NVM file"
    print("Native live flash:10actual SIGKILLs,10I/O faults,20cold guest loads,57full wire checks passed.")


if __name__ == "__main__":
    main()
