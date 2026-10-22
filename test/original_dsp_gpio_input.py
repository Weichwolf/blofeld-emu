# SPDX-License-Identifier: GPL-3.0-only
"""Pin unchanged firmware and validate fixed original GPIO-producer evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    if not firmware.is_file():
        print('SKIP original GPIO producer: private firmware absent')
        return 77
    all_records = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    reference = json.loads((ROOT / 'doc/dsp-gpio-input-reference.json').read_text())
    original_records(firmware, all_records)
    if reference['midi_sha256'] != all_records['midi_sha256']:
        raise ValueError('Changed original GPIO input MID reference')
    for entry in reference['records']:
        if entry != all_records['records'][entry['record']]:
            raise ValueError('Changed final original GPIO producer/vector selection')
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, timeout=60)
    if result.returncode:
        return result.returncode
    rows = trace.read_text().splitlines()
    inputs = [byte for byte in range(256) if byte != reference['excluded_request_byte']]
    expected = [f'BYTE value={byte} slot={index % 128}' for index, byte in enumerate(inputs)]
    if rows[:-1] != expected or not rows[-1].startswith('RESULT '):
        raise ValueError('Incomplete fixed original GPIO byte matrix')
    observed = {key: int(value) for key, value in
                (token.split('=', maxsplit=1) for token in rows[-1][len('RESULT '):].split())}
    if observed != reference['expected']:
        raise ValueError('Changed native original GPIO producer/frame/return totals')
    report.write_text(json.dumps({
        'passed': True, 'records': reference['records'], **observed,
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'qualification': reference['qualification'], 'fuzz_execution': False,
        'physical_traffic': False}, indent=2) + '\n')
    print('Verified 255 original GPIO input bytes through native IRQA and RTI')
    return 0


if __name__ == '__main__':
    sys.exit(main())
