# SPDX-License-Identifier: GPL-3.0-only
"""Run original complete voices with nonzero Drive and independent equations."""

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

from dsp_voice_model import waveform_model
from dsp_voice_validation import new_totals, validate_stream
from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]


def validate(text, xdata, ydata, reference):
    tables = {mode: waveform_model(ydata, mode) for mode in (False, True)}
    cases = re.findall(r'CASE (normal|sync) ([01]) (\d+) ([01]) ([01])\n([\s\S]*?)(?=CASE |Entry-streams visited PCs:)', text)
    if len(cases) != reference['streams']:
        raise ValueError('Incomplete original Drive voice matrix')
    totals = new_totals()
    levels = reference['drive_levels']
    entries = set()
    for index, (*identity, stream) in enumerate(cases):
        curve, remainder = divmod(index, 4)
        mode, routing = divmod(remainder, 2)
        if identity != ['sync', str(mode), '127', '1', str(routing)]:
            raise ValueError('Original Drive stream order/profile differs')
        second = (curve + reference['second_selector_offset']) % reference['selectors']
        drives = [(curve, levels[curve % len(levels)]), (second, levels[second % len(levels)])]
        settings = re.findall(r'^DRIVE_SETTING ([^\n]+)$', stream, re.M)
        expected = [*drives[0], *drives[1], reference['blocks_per_stream']]
        if len(settings) != 1 or list(map(int, settings[0].split())) != expected:
            raise ValueError('Declared raw Drive fields differ')
        validate_stream(stream, xdata, ydata, tables, identity, totals,
                        reference['blocks_per_stream'], drives)
        entries.update(xdata[0x2f36 + selector] for selector, _ in drives)
    blocks = reference['streams'] * reference['blocks_per_stream']
    if (totals['blocks'] != blocks or totals['dependent_blocks'] != blocks - reference['streams']
            or totals.get('drive_calls') != 2 * blocks):
        raise ValueError('Incomplete dependent original Drive calls')
    if any(totals.get(f'curve_{curve}_changed_words', 0) == 0 for curve in range(13)):
        raise ValueError('A manual Drive curve never affected an independently checked word')
    if any(totals.get(f'curve_{curve}_changed_words', 0) != 0 for curve in range(13, 16)):
        raise ValueError('Reserved original RTS changed samples')
    coverage = re.findall(r'^Entry-streams visited PCs:([^\n]+)$', text, re.M)
    if len(coverage) != 1:
        raise ValueError('Missing observed original voice/Drive PCs')
    visited = [int(value, 16) for value in coverage[0].split()]
    if visited != sorted(set(visited)) or not entries <= set(visited):
        raise ValueError('Not every original selected callback executed')
    # Observed counts are reported without inventing a runtime-derived coverage baseline.
    instructions = re.findall(r'^Entry-streams instructions: (\d+)$', text, re.M)
    if len(instructions) != 1 or int(instructions[0]) <= 0:
        raise ValueError('Missing actual native step count')
    totals['native_steps_including_setup_and_caller'] = int(instructions[0])
    totals['observed_instruction_starts'] = len(visited)
    return totals


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original Drive voices: private firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/dsp-drive-voice-reference.json').read_text())
    path = ROOT / reference['runtime_reference']['path']
    if hashlib.sha256(path.read_bytes()).hexdigest() != reference['runtime_reference']['sha256']:
        raise ValueError('Changed ordered original runtime identities')
    runtime = json.loads(path.read_text())
    for key in ('program', 'callbacks', 'caller'):
        expected = reference[key]
        if runtime['records'][expected['record']] != expected:
            raise ValueError('Changed original Drive component identity')
    xdata, ydata = original_records(firmware, runtime)
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, check=False, timeout=280)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in
                               ('AddressSanitizer', 'LeakSanitizer', 'UndefinedBehaviorSanitizer', 'runtime error:')):
        raise ValueError(f'Original Drive voices failed ({result.returncode}): {diagnostics}')
    totals = validate(trace.read_text(), xdata, ydata, reference)
    report.write_text(json.dumps({'passed': True, 'scope': reference['qualification'],
                                  'streams': reference['streams'], **totals,
                                  'fuzz_execution': False, 'physical_traffic': False}, indent=2) + '\n')
    print('Verified complete original nonzero-Drive voices; full boot, live MIDI/PCM and physical timing remain open.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
