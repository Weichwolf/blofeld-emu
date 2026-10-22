# SPDX-License-Identifier: GPL-3.0-only
"""Pin original records and fixed untimed F4 request execution evidence."""
import hashlib
import json
from pathlib import Path
import re
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
        print('SKIP original F4 requests: private firmware absent')
        return 77
    all_records = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    reference = json.loads((ROOT / 'doc/dsp-gpio-request-reference.json').read_text())
    original_records(firmware, all_records)
    if reference['midi_sha256'] != all_records['midi_sha256']:
        raise ValueError('Changed original GPIO request MID reference')
    for entry in reference['records']:
        if entry != all_records['records'][entry['record']]:
            raise ValueError('Changed complete original GPIO request/vector selection')
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, timeout=240)
    if result.returncode:
        return result.returncode
    rows = trace.read_text().splitlines()
    cases = len(reference['case_names'])
    if len(rows) != cases * 2 + 1:
        raise ValueError('Incomplete original F4 request matrix')
    instructions = 0
    for number in range(cases):
        if not re.fullmatch(rf'BEGIN {number} \d+ \d+ \d+', rows[number * 2]):
            raise ValueError('Changed fixed F4 request input ordering')
        observed = re.fullmatch(
            rf'CASE {number} steps=(\d+) low=(\d+) high=(\d+) reads=(\d+) reply=(\d+|none)',
            rows[number * 2 + 1])
        if not observed:
            raise ValueError('Missing checked native request outcome')
        instructions += int(observed[1])
    expected = reference['expected']
    totals = f'RESULT cases={cases} restores={expected["restores"]} '
    totals += f'instructions_excluding_replays={instructions}'
    digest = hashlib.sha256(trace.read_bytes()).hexdigest()
    if (rows[-1] != totals or cases != expected['cases']
            or instructions != expected['instructions_excluding_replays']
            or digest != reference['trace_sha256']):
        raise ValueError('Changed native original F4 outcomes or restore totals')
    report.write_text(json.dumps({
        'passed': True, 'records': reference['records'], **expected,
        'trace_sha256': digest, 'qualification': reference['qualification'],
        'fuzz_execution': False, 'physical_traffic': False}, indent=2) + '\n')
    print(f'Verified {cases} original F4 cases with {expected["restores"]} joint restores; '
          'board boot, physical timing and later bulk response remain open')
    return 0


if __name__ == '__main__':
    sys.exit(main())
