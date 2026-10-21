# SPDX-License-Identifier: GPL-3.0-only
"""Check the actual original Drive calls inside independently modeled voices."""

import re

from dsp_drive_model import drive_live_state


def rows(block, label):
    return [[int(word, 16) for word in line.split()]
            for line in re.findall('^' + label + r' ([^\n]+)$', block, re.M)]


def validate_drive_calls(voice, block, xdata, oscillators):
    entries, returned = rows(block, 'DRIVE_ENTRY'), rows(block, 'DRIVE_RETURN')
    if len(entries) != 2 or len(returned) != 2:
        raise ValueError('Missing original Drive frames')
    inputs, outputs = [], []
    metrics = dict(drive_calls=2, changed_drive_words=0)
    for i, ((curve, _), entry, end) in enumerate(zip(voice.drives, entries, returned)):
        drive = max(0, voice.parameters[5 * i + 3])
        if (len(entry) != 8 or entry[:4] != [i, curve, xdata[0x2f36 + curve], 2]
                or entry[4] != (0x5c9 if i == 0 else 0x5e4 if voice.serial else 0x5ff)
                or entry[5:7] != [0x580 + 16 * i, drive]):
            raise ValueError('Original Drive callback/parameter/buffer/frame differs')
        if (entry[7] & 0x200c00 != (0x200000 if voice.twos else 0)
                or entry[7] & 0x0c != (4 if drive == 0 else 8)):
            raise ValueError('Original Drive CLR/CMP flags or rounding/scaling differs')
        r0, x1, r5 = drive_live_state(curve, drive,
                                     voice.drive_inputs[i] + voice.drive_lookahead[i],
                                     oscillators[0] + [oscillators[1][0]],
                                     voice.drive_lookahead[i][0], xdata[0x2f36 + curve],
                                     buffer=0x580 + 16 * i)
        if end != [i, entry[4], 1, r0, x1, r5]:
            raise ValueError('Original Drive did not return through its genuine frame')
        inputs.append([i, *voice.drive_inputs[i]])
        outputs.append([i, *voice.drive_outputs[i]])
        differences = sum(a != b for a, b in zip(voice.drive_inputs[i], voice.drive_outputs[i]))
        metrics['changed_drive_words'] += differences
        metrics[f'curve_{curve}_changed_words'] = metrics.get(f'curve_{curve}_changed_words', 0) + differences
    if rows(block, 'DRIVE_INPUT') != inputs or rows(block, 'DRIVE_OUTPUT') != outputs:
        raise ValueError('Original in-voice Drive input/output differs from independent equations')
    lookahead = [[i, *voice.drive_lookahead[i], oscillators[1][0], voice.drive_lookahead[i][0]]
                 for i in range(2)]
    if rows(block, 'DRIVE_LOOKAHEAD') != lookahead:
        raise ValueError('Original Drive neighboring words or incoming X1 differ')
    if rows(block, 'DRIVE_OSCILLATOR') != [[0, *oscillators[0]], [1, *oscillators[0]]]:
        raise ValueError('Original Drive oscillator-1 modulation input differs')
    return metrics
