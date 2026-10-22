# SPDX-License-Identifier: GPL-3.0-only
"""Check original request framing and release against an independent HCS12 ledger."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_runtime_clocks import one, rows

ROOT = Path(__file__).resolve().parents[1]
INPUTS = ((0, 0, 0), (1, 64, 0), (1, 0, 0), (2, 0, 0),
          (0, 15, 511), (1, 79, 63), (1, 7, 24), (2, 0, 76))
PACKETS = ((244, 0, 128, 0), (244, 64, 192, 64), (244, 64, 128, 0),
           (244, 96, 128, 0), (244, 63, 255, 127), (244, 79, 255, 127),
           (244, 71, 152, 24), (244, 96, 204, 76))

# S12CPU15UG/D v1.2: STAB EXT PwO, JSR EXT SPPP, NOP O,
# BSET EXT rPwP, RTS UfPPP, LDAB IDX rPf, CLR EXT PwO.
TAIL = ((0xb799, 3), (0xb79c, 4), (0xb6d0, 1), (0xb6d1, 1), (0xb6d2, 1),
        (0xb6d3, 4), (0xb6d7, 1), (0xb6d8, 1), (0xb6d9, 1), (0xb6da, 1),
        (0xb6db, 5), (0xb79f, 3), (0xb7a1, 5), (0xb75d, 3), (0xb760, 3))


def validate(text):
    cases = re.findall(r'^CASE ([^\n]+)\n([\s\S]*?)(?=^CASE |\Z)', text, re.M)
    if len(cases) != len(INPUTS):
        raise ValueError('CASE incomplete original MCU request matrix')
    for index, ((header, body), inputs, packet) in enumerate(zip(cases, INPUTS, PACKETS, strict=True)):
        fields = dict((key, int(value)) for key, value in re.findall(r'(\w+)=(\d+)', header))
        if fields != dict(index=index, kind=inputs[0], location=inputs[1], parameter=inputs[2]):
            raise ValueError('CASE input or order changed')
        if one(body, 'PREFIX') != dict(pc=0x8050, instructions=35, cycles=76, ticks=752,
                                       timebase=60000000, divider=2):
            raise ValueError('PREFIX original clock initialization differs')
        writes = rows(body, 'WRITE')
        data = [row for row in writes if row['address'] == 0x260]
        direction = [row for row in writes if row['address'] == 0x262]
        strobe = [row for row in writes if row['address'] == 0x250]
        if [row['value'] for row in data] != list(packet):
            raise ValueError('WRITE original F4/header/field/reloaded-field packet differs')
        if [(row['pc'], row['value']) for row in direction] != [(0xb740, 255), (0xb760, 0)]:
            raise ValueError('WRITE original direction setup/release differs')
        if [row['value'] for row in strobe] != [1, 0, 1, 1, 1]:
            raise ValueError('WRITE original J0 sequence differs')
        if (any((row['mask'], row['known'], row['levels']) != (255, 255, row['value'])
                for row in data) or direction[-1]['mask'] or direction[-1]['known']):
            raise ValueError('WRITE data drivers must retain their latch until native release')
        steps = rows(body, 'STEP')
        if any(row['ticks'] - 752 != 2 * (row['cycles'] - 76) for row in steps):
            raise ValueError('STEP original selected-clock period changed')
        starts = [i for i, row in enumerate(steps) if row['pc'] == 0xb799]
        if len(starts) != 2:
            raise ValueError('STEP original inherited-frame byte helper call count differs')
        tail = steps[starts[-1]:]
        end = one(body, 'RELEASE')
        if [row['pc'] for row in tail] != [pc for pc, _ in TAIL]:
            raise ValueError('STEP original release-tail instruction path differs')
        for row, following, (_, cycles) in zip(tail, [*tail[1:], end], TAIL, strict=True):
            if following['cycles'] - row['cycles'] != cycles or following['ticks'] - row['ticks'] != 2 * cycles:
                raise ValueError('STEP original HCS12 cycle ledger differs')
        # Both writes commit after P,w; the data latch changes three cycles
        # before DDR-Q releases. No input poll is executed or acknowledged here.
        if (data[2]['cycles'] != tail[0]['cycles'] + 2
                or data[3]['cycles'] - data[2]['cycles'] != 31
                or direction[-1]['cycles'] - data[2]['cycles'] != 34
                or direction[-1]['ticks'] - data[2]['ticks'] != 68):
            raise ValueError('WRITE original 34-cycle driven interval differs')
        if (end['pc'], end['data_reads'], end['nvm_unchanged']) != (0xb763, 0, 1):
            raise ValueError('RELEASE must stop before reply polling and preserve NVM')


def negative_controls(text):
    for label, field in (('CASE', 'kind'), ('PREFIX', 'divider'), ('WRITE', 'value'),
                         ('STEP', 'ticks'), ('RELEASE', 'data_reads')):
        changed, count = re.subn('^' + label + r' (.*?\b' + field + r'=)(\d+)',
                                 lambda m: label + ' ' + m[1] + str(int(m[2]) + 1),
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing fixed MCU release oracle control')
        try:
            validate(changed)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong fixed MCU release oracle failure') from error
        else:
            raise ValueError('MCU release oracle accepted fixed corruption of ' + label)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original MCU request release: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    report = trace.with_suffix('.json')
    report.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=110, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or result.stderr or any(marker in diagnostics for marker in
                                                ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Original MCU request release failed: ' + diagnostics)
    text = trace.read_text()
    validate(text)
    negative_controls(text)
    report.write_text(json.dumps({'passed': True, 'cases': len(INPUTS),
                                  'driven_tail_cycles': 34, 'post_latch_release_cycles': 3,
                                  'whole_tail_cycles': sum(cycles for _, cycles in TAIL),
                                  'reply_reads': 0, 'fixed_oracle_corruptions': 5,
                                  'fuzz_execution': False, 'physical_traffic': False,
                                  'scope': 'Unmodified original MCU request/strobe helpers after '
                                  'the original clock prefix, with explicit caller/RAM and '
                                  'GPIO/PHY observations. Independent packet/cycle expectations; '
                                  'no DSP reply, complete transaction, boot or hardware timing proof.'},
                                 indent=2) + '\n')
    print('Verified eight original MCU request paths and the independent34-cycle release interval.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
