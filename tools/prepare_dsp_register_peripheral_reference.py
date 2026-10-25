# SPDX-License-Identifier: GPL-3.0-only
"""Generate fixed own MOVEP windows; assemble, execute and download nothing."""
import argparse
import json
from pathlib import Path


def reference_program():
    source = ['; SPDX-License-Identifier: GPL-3.0-only', '        org p:$100']
    instructions = []
    cases = []

    def emit(text):
        source.append('        ' + text)
        instructions.append(text)
        return len(instructions) - 1

    def quiet(count=8):
        for _ in range(count):
            emit('nop')

    emit('move #>$305,omr')
    emit('move #>$c00300,sr')
    emit('move #>$ffffff,m0')
    quiet()
    forms = [('x0_quiet', 'x0', 3, False),
             ('r1_quiet', 'r1', 9, False),
             ('r0_quiet', 'r0', 11, False),
             ('immediate_quiet', 'immediate', 13, False),
             ('r0_younger_r0_update', 'r0', 17, True),
             ('r1_younger_r0_update', 'r1', 19, True)]
    for name, register, value, alias in forms:
        for target in ('r0', 'r1', 'x0'):
            emit(f'move #>${value:x},{target}')
        emit('move #>0,y1')
        quiet()
        before = len(instructions) - 1
        operand = f'#>${value:x}' if register == 'immediate' else register
        body = emit(f'movep {operand},x:<<$ffff90')
        if alias:
            emit('move x:(r0)+,x0')
        after = len(instructions) + 5
        quiet()
        state_check = emit('move r0,y1')
        quiet(4)
        cases.append({'name': name, 'source': register, 'value': value, 'alias': alias,
                      'body_index': body, 'before_anchor': before, 'after_anchor': after,
                      'state_check_index': state_check, 'expected_hckr': value,
                      'expected_x0': 0x234567 if alias else value,
                      'expected_y1': value + int(alias),
                      'expected_anchor_clocks': 1 + int(register == 'immediate') + int(alias) + 6})
    source.append('        end')
    return '\n'.join(source) + '\n', {
        'scope': 'Six fixed own initial-map MOVEP write windows. No hardware traffic; register '
                 'source/adjacent AGU semantics and primary instruction counts only; '
                 'vendor peripheral/silicon model unqualified.',
        'instructions': instructions, 'cases': cases,
        'data_addresses': [case['value'] for case in cases]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, nargs='?',
                        default=Path(__file__).resolve().parents[1] /
                                'doc/local/dsp-register-peripheral-reference-generated')
    args = parser.parse_args()
    source, metadata = reference_program()
    args.directory.mkdir(parents=True, exist_ok=True)
    (args.directory / 'stores.asm').write_text(source)
    (args.directory / 'cases.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('Prepared', len(metadata['cases']), 'fixed windows and',
          len(metadata['instructions']), 'own static instructions')


if __name__ == '__main__':
    main()
