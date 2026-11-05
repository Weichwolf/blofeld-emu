# SPDX-License-Identifier: GPL-3.0-only
"""Pin the original master stage before independent-core silent-history checks."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = ("MASTER_HISTORY calls=336 output-words=10752 fixed-words=31 "
            "endpoint-updates=389 gains=6 original-instructions=48720\n")


def validate(result, output):
    diagnostics = result.stdout + result.stderr
    (output / 'native.log').write_text(diagnostics)
    if result.returncode or result.stdout != EXPECTED or result.stderr:
        raise ValueError('Original master-history execution failed: ' + diagnostics)
    return {'passed': True, 'calls': 336, 'output_words': 10752, 'fixed_words': 31,
            'sharp_settling_updates': 389, 'master_volumes': [0, 1, 32, 64, 96, 127],
            'original_instructions': 48720, 'log_sha256': hashlib.sha256(diagnostics.encode()).hexdigest(),
            'scope': 'Independent DSP core, original master stage, authored caller, unscaled transfers, '
                     'declared zero input/RAM and initial histories; no boot, CC7 ingress, DMA, host audio '
                     'or physical timing claim', 'physical_traffic': False}


def execute(binary, firmware, output):
    output.mkdir(parents=True, exist_ok=True)
    report = output / 'report.json'
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print('SKIP original master histories: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'test/reference/dsp-voice-reference.json').read_text())
    original_records(firmware, reference)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve())], capture_output=True,
                            text=True, check=False, timeout=120)
    report.write_text(json.dumps(validate(result, output), indent=2) + '\n')
    print('Original master histories: 336 calls, 10752 words; fixed residuals preserved.')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 4:
        sys.exit('Usage: original_master_history.py binary firmware.zip output')
    try:
        sys.exit(execute(*map(Path, sys.argv[1:])))
    except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
        sys.exit(str(error))
