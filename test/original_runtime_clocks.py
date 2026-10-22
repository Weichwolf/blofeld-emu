# SPDX-License-Identifier: GPL-3.0-only
"""Verify the original clock prefixes against independent nominal instruction costs."""

import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records

ROOT = Path(__file__).resolve().parents[1]
DELAY = 0xabe0
RESET, SLOW, FAST = 0x04401d, 0x02e112, 0x02a012


def rows(text, label):
    return [{key: int(value) for key, value in re.findall(r'(\w+)=(\d+)', line)}
            for line in re.findall('^' + label + r' (.+)$', text, re.M)]


def one(text, label):
    found = rows(text, label)
    if len(found) != 1:
        raise ValueError('Missing/duplicate ' + label)
    return found[0]


def validate(text):
    # MCU count: original reset5 cycles and the prior independent HCS12 census,
    # including the declared E2/E3 loopback and one15-edge PHY detector window.
    mcu = one(text, 'MCU')
    # S12CPU15UG BSET DIR has rPwO: CLKSEL commits at cycle75, then
    # its final O slot runs at the newly selected source (two timebase ticks).
    if mcu != dict(pc=0x8050, instructions=35, cycles=76, ticks=752,
                   timebase=60000000, divider=2):
        raise ValueError('MCU original clock selection differs from the independent prefix')
    slots = re.findall(r'^MCU_SLOT kind=(\w) prior-cycles=(\d+) prior-ticks=(\d+) source=(\d+)$',
                       text, re.M)
    if slots != [('r', '72', '720', '0'), ('P', '73', '730', '0'),
                 ('w', '74', '740', '0'), ('O', '75', '750', '1')]:
        raise ValueError('MCU clock-write bus slots differ from the independent rPwO sequence')
    # DSP56300FM Rev5 A-1/A-6/A-7/A-8: long MOVEC/MOVE2, NOP1,
    # long-immediate MOVEP2, register REP5. No SDK timing table is imported.
    expected = [(0x100, 2, 90, RESET, RESET, 1), (0x102, 2, 90, RESET, RESET, 1),
                (0x104, 1, 90, RESET, RESET, 1), (0x105, 2, 90, RESET, SLOW, 1),
                (0x107, 5, 20, SLOW, SLOW, 1), (0x108, 1, 20, SLOW, SLOW, DELAY),
                (0x109, 2, 20, SLOW, FAST, 1), (0x10b, 2, 5, FAST, FAST, 1)]
    observed = [tuple(row[key] for key in ('pc', 'units', 'period', 'before', 'after', 'count'))
                for row in rows(text, 'ROW')]
    if observed != expected:
        raise ValueError('ROW original DSP instruction/rate ledger differs')
    begin, partial, end, replay, short = [one(text, label) for label in
                                         ('BEGIN', 'PARTIAL', 'END', 'REPLAY', 'SHORT')]
    # Own entry: long MOVEC2, three NOPs, JMP ea3+lab1 at bypass12MHz.
    if begin['cycles'] != 9 or begin['ticks'] != 810 or begin['pc'] != 0x100:
        raise ValueError('BEGIN own DSP entry does not retain its complete native clock')
    if partial['cycles'] != 22021 or partial['ticks'] != 441540 or partial['pc'] != 0x108:
        raise ValueError('PARTIAL mid-REP count/time differs')
    if end != replay or end != dict(pc=0x10d, cycles=44025, ticks=881590,
                                    pctl=FAST, x0=DELAY, m0=0xffffff, sr=0x300):
        raise ValueError('END original DSP clock/register or REPLAY state differs')
    if short['pc'] != 0x10b or short['cycles'] != 44023 or short['ticks'] != 440790 or short['pctl'] != FAST:
        raise ValueError('SHORT timebase must fail only after the original final PLL command')
    faults = re.findall(r'^FAULT (.+)$', text, re.M)
    if len(faults) != 2 or faults[0] != faults[1] or any(
            'not integral' not in fault or 'pc=0x00010b' not in fault for fault in faults):
        raise ValueError('FAULT must preserve the nonintegral period and original PC through restore')


def negative_controls(text):
    for label, field in (('MCU', 'divider'), ('ROW', 'count'), ('PARTIAL', 'ticks'),
                         ('END', 'pctl'), ('SHORT', 'pc')):
        pattern = '^' + label + r' (.*?\b' + field + r'=)(\d+)'
        changed, count = re.subn(pattern, lambda m: label + ' ' + m[1] + str(int(m[2]) + 1),
                                 text, count=1, flags=re.M)
        if count != 1:
            raise ValueError('Missing fixed clock negative control')
        try:
            validate(changed)
        except ValueError as error:
            if label not in str(error):
                raise ValueError('Wrong fixed clock negative-control failure') from error
        else:
            raise ValueError('Clock oracle accepted fixed corruption of ' + label)


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original runtime clocks: local firmware absent')
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
        raise ValueError('Original clock prefix failed: ' + diagnostics)
    text = trace.read_text()
    validate(text)
    negative_controls(text)
    report.write_text(json.dumps({'passed': True, 'original_dsp_delay_iterations': DELAY,
                                  'mid_repeat_replay': True, 'nonintegral_fault_replays': 2,
                                  'fixed_oracle_corruptions': 5, 'nominal_mcu_bus_hz': 30000000,
                                  'nominal_dsp_hz': [12000000, 54000000, 216000000],
                                  'reference_timebase_hz': 1080000000, 'fuzz_execution': False,
                                  'physical_traffic': False,
                                  'scope': 'Bounded unchanged MCU reset clock prefix with declared '
                                  'GPIO/PHY observations and original DSP clock entry with own '
                                  'native caller. Independent nominal instruction/time ledger; '
                                  'not complete boot, physical PLL/timing or coupled GPIO proof.'},
                                 indent=2) + '\n')
    print('Verified original MCU/DSP clock prefixes and complete44000-iteration delay; '
          'physical clocks and complete boot remain unqualified.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
