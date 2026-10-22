# SPDX-License-Identifier: GPL-3.0-only
"""Pin original programs and check fixed native GPIO/MIDI integration evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import original_records
from original_mcu_scan import original_mcu

ROOT = Path(__file__).resolve().parents[1]


def validate(text, expected):
    matrix = {(transport, inverted, ratio, channel)
              for transport in (0, 1) for inverted in (0, 1)
              for ratio in (1, 2, 4) if transport or ratio != 1 for channel in (0, 5)}
    seen = set()
    totals = {'profiles': 0, 'replays': 0, 'slow_rejections': 0, 'profile_bytes': 0,
              'profile_clocks': 0, 'profile_callbacks': 0, 'profile_producer_steps': 0}
    rows = text.splitlines()
    if rows[-1] != 'RESULT profiles=20 replays=2 slow-rejections=1':
        raise ValueError('Missing complete native GPIO/MIDI result')
    for line in rows[:-1]:
        if line == 'REJECT slow-frame-profile=1 exact-data-mismatch=1':
            totals['slow_rejections'] += 1
            continue
        if not line.startswith('CASE '):
            raise ValueError('Unexpected native GPIO/MIDI trace row')
        values = {key: int(value) for key, value in
                  (token.split('=', maxsplit=1) for token in line.split()[1:])}
        key = tuple(values[name] for name in ('transport', 'inverted', 'ratio', 'channel'))
        if key not in matrix or key in seen:
            raise ValueError('Duplicate or out-of-matrix native GPIO/MIDI profile')
        seen.add(key)
        transport, inverted, ratio, channel = key
        byte_count = 160 if transport else 128
        callbacks = 3 if channel else 48
        replay = int(not inverted and ratio == 4 and not channel)
        wanted = dict(zip(('transport', 'inverted', 'ratio', 'channel'), key))
        wanted.update(bytes=byte_count, consumed=byte_count, irqs=byte_count, rtis=byte_count,
                      clocks=16, callbacks=callbacks, pitches=callbacks, replay=replay,
                      **{'producer-steps': (19 * byte_count) + 32,
                         'mcu-steps': 5747 if transport else 2326, 'transitions': 2 * byte_count})
        if values.get('preemptions', 0) < 1:
            raise ValueError('Original parser must actually be preempted')
        del values['preemptions']
        if values != wanted:
            raise ValueError(f'Independent native GPIO/MIDI mismatch: {key}: {values} != {wanted}')
        totals['profiles'] += 1
        totals['replays'] += replay
        totals['profile_bytes'] += byte_count
        totals['profile_clocks'] += 16
        totals['profile_callbacks'] += callbacks
        totals['profile_producer_steps'] += (19 * byte_count) + 32
    if seen != matrix or totals != expected:
        raise ValueError('Incomplete original GPIO/MIDI matrix or independent totals')
    return totals


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    report = trace.with_suffix('.json')
    for path in (trace, report, trace.with_suffix('.log')):
        path.unlink(missing_ok=True)
    if not firmware.is_file():
        print('SKIP native GPIO/MIDI: private original firmware absent')
        return 77
    reference = json.loads((ROOT / 'doc/gpio-midi-reference.json').read_text())
    complete = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    original_records(firmware, complete)
    original_mcu(firmware, reference)
    if reference['midi_sha256'] != complete['midi_sha256']:
        raise ValueError('Changed original GPIO/MIDI MID identity')
    for record in reference['records']:
        if record != complete['records'][record['record']]:
            raise ValueError('Changed final complete original GPIO/parser record selection')
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            check=False, capture_output=True, text=True, timeout=280)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            'AddressSanitizer', 'LeakSanitizer', 'UndefinedBehaviorSanitizer', 'runtime error:')):
        raise ValueError(f'Original native GPIO/MIDI integration failed: {diagnostics}')
    totals = validate(trace.read_text(), reference['expected'])
    report.write_text(json.dumps({'passed': True, **totals,
                                  'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
                                  'qualification': reference['qualification'],
                                  'fuzz_execution': False, 'physical_traffic': False}, indent=2) + '\n')
    print('Verified 20 original native MCU/GPIO/parser profiles and two joint snapshot replays')
    return 0


if __name__ == '__main__':
    sys.exit(main())
