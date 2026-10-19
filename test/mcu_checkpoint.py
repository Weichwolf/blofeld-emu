# SPDX-License-Identifier: GPL-3.0-only
"""Give backend mutation regressions an isolated disposable directory."""
import subprocess
import sys
import tempfile

with tempfile.TemporaryDirectory(prefix="blofeld-checkpoint-") as temporary:
    subprocess.run([sys.argv[1], temporary], check=True)
