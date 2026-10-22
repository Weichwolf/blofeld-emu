# SPDX-License-Identifier: GPL-3.0-only
"""Check actual modeled USB OUT/MCU ISR/GPIO/DSP parser integration."""
from pathlib import Path
import sys

from original_gpio_midi import run


def validate(text, expected):
    matrix = {(inverted, ratio, channel) for inverted in (0, 1)
              for ratio in (1, 2, 4) for channel in (0, 5)}
    seen = set()
    totals = dict.fromkeys(expected, 0)
    rows = text.splitlines()
    if not rows or rows[-1] != 'RESULT endpoint-profiles=12 joint-replays=1':
        raise ValueError('Missing complete native USB/GPIO/MIDI result')
    for line in rows[:-1]:
        if not line.startswith('CASE '):
            raise ValueError('Unexpected native USB/GPIO/MIDI trace row')
        values = {key: int(value) for key, value in
                  (token.split('=', maxsplit=1) for token in line.split()[1:])}
        key = tuple(values[name] for name in ('inverted', 'ratio', 'channel'))
        if key not in matrix or key in seen:
            raise ValueError('Duplicate or out-of-matrix endpoint profile')
        seen.add(key)
        inverted, ratio, channel = key
        callbacks = 3 if channel else 48
        replay = int(not inverted and ratio == 4 and not channel)
        instructions = (4 * 14) + (4 * 15) + (48 * 101) + (16 * 51) + (4 * 8) + (2 * 4)
        wanted = {'transport': 2, 'inverted': inverted, 'ratio': ratio, 'channel': channel,
                  'bytes': 160, 'consumed': 160, 'irqs': 160, 'rtis': 160, 'clocks': 16,
                  'callbacks': callbacks, 'pitches': callbacks, 'producer-steps': (19 * 160) + 32,
                  'mcu-steps': instructions, 'transitions': 320, 'replay': replay,
                  'usb-packets': 4, 'usb-irqs': 4, 'mcu-frame-reuses': 3}
        if values.pop('preemptions', 0) < 1 or values.pop('usb-naks', 0) < 1:
            raise ValueError('Actual parser preemption and native OUT backpressure are required')
        if values != wanted:
            raise ValueError(f'Independent native endpoint mismatch: {key}: {values} != {wanted}')
        totals['profiles'] += 1
        totals['replays'] += replay
        totals['profile_bytes'] += 160
        totals['profile_clocks'] += 16
        totals['profile_callbacks'] += callbacks
        totals['profile_producer_steps'] += (19 * 160) + 32
        totals['profile_mcu_instructions'] += instructions
        totals['profile_usb_packets'] += 4
        totals['profile_usb_irqs'] += 4
        totals['profile_mcu_frame_reuses'] += 3
    if seen != matrix or totals != expected:
        raise ValueError('Incomplete actual native USB/GPIO/MIDI matrix or independent totals')
    return totals


def main():
    return run(*map(Path, sys.argv[1:]), 'usb-gpio-midi-reference.json', validate, ('endpoint',))


if __name__ == '__main__':
    sys.exit(main())
