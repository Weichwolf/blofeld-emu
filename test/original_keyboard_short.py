# SPDX-License-Identifier: GPL-3.0-only
"""Execute the pinned Keyboard short message through both unchanged raw parsers."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from keyboard_update_reference import verified_blocks
from original_dsp_voice import ROOT, original_records
from inspect_firmware import load_input, read_smf

VALUES = (0, 1, 0x7fffff, 0x800000, 0xfffffe, 0xffffff)


def validate(lines):
    expected = []
    for route in range(2):
        for u in VALUES:
            for v in VALUES:
                for copy in range(2):
                    result = 0x700000 + (u >> 8) + (v >> 16)
                    assert 0x700000 <= result <= 0x7100fe and result != 0x7e0601
                    expected.append(f'CASE {route} {copy} {u} {v} {result}')
    for route in range(2):
        for copy in range(2):
            expected.append(f'BULK {route} {copy} blocked={route} replays={2 if route else 0}')
        for missing in range(3):
            expected.append(f'CUT {route} {missing} replays=2')
    expected.append('RESULT 144 4 6 12')
    if lines != expected:
        raise ValueError('Missing or changed original short-message arithmetic/state/boundary evidence')


def controls(lines):
    changes = []
    wrong_math = list(lines)
    wrong_math[0] += ' changed'
    changes.append(wrong_math)
    changes.append(lines[1:])
    for prefix in ('BULK 1 0', 'CUT 0 0', 'CUT 1 2', 'RESULT'):
        index = next(i for i, row in enumerate(lines) if row.startswith(prefix))
        changes.append(lines[:index] + lines[index + 1:])
    for changed in changes:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Fixed corrupted short-message evidence was accepted')
    return len(changes)


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    if not all(path.is_file() for path in (firmware, keyboard)):
        print('SKIP original Keyboard short message: pinned local archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    shared = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    xdata, _ = original_records(firmware, shared)
    assert all(address not in xdata for address in range(0x779, 0x77c))
    _, reference, _ = verified_blocks(keyboard)
    messages, _ = read_smf(load_input(keyboard)[0])
    short = [messages[position] for position in reference['unclassified_short_message_positions']]
    assert len(short) == 2 and short[0] == short[1]
    inputs = trace.with_suffix('.message')
    inputs.write_bytes(b'\xf0' + short[0])
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, inputs)}
    run = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                          str(inputs.resolve()), str(trace.resolve())],
                         capture_output=True, text=True, check=False, timeout=110)
    output = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Original Keyboard short-message rejection, stale reads and deferred F7 verified.'
    if run.returncode or any(row and row != allowed and
                            not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                            for row in output.splitlines()):
        raise ValueError('Original Keyboard short-message execution failed: ' + output)
    lines = trace.read_text().splitlines()
    validate(lines)
    corruptions = controls(lines)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != value for path, value in frozen.items()):
        raise ValueError('Short-message native input changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(
        passed=True, raw_parser_messages=144, declared_bulk_cases=4,
        deferred_secondary_cases=2, deferred_poll_returns=6, deferred_trusted_replays=4,
        stale_read_fault_cases=6, stale_read_trusted_replays=12,
        fixed_collector_corruptions=corruptions,
        original_short_positions=reference['unclassified_short_message_positions'],
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        scope='Isolated original raw ring/parser rejection of both identical Keyboard short '
              'message copies, under declared linear/unscaled callers and stale RAM. No wire '
              'timing, interleaved sender/helper transaction, complete update, controller '
              'execution, full boot or hardware-equivalence qualification.',
        fuzz_execution=False, physical_traffic=False), indent=2) + '\n')
    print('Verified144 original raw messages, deferred secondary F7 and strict stale overreads.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
