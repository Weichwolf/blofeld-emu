# SPDX-License-Identifier: GPL-3.0-only
"""Verify original global settle/refresh with continuously clocked native RTI."""
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_notification import rows
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ((1, 0, 0), (1, 0, 1), (1, 254, 0), (1, 254, 1), (0, 254, 0), (0, 254, 1))


def validate(report, trace):
    sections = re.findall(r'^CASE enabled=(\d+) tick=(\d+) order=(\d+)\n([\s\S]*?)(?=^CASE |\Z)',
                          trace, re.M)
    lines = report.splitlines()
    if len(lines) != 6 or len(sections) != 6:
        raise ValueError('CASE incomplete native global wait matrix')
    queries = unresolved = clocks = polls = 0
    for line, section, context in zip(lines, sections, MATRIX, strict=True):
        enabled, tick, order = context
        if tuple(map(int, section[:3])) != context:
            raise ValueError('CASE changed global wait order')
        result = fields(line)
        variable = ('polls', 'clock-checks', 'elapsed-osc', 'queries', 'unresolved-events')
        fixed = {k: v for k, v in result.items() if k not in variable}
        if (fixed != dict(enabled=enabled, tick=tick, order=order, stores=1, waits=1,
                          **{'irq-entries': 8 if enabled else 0, 'irq-returns': 8 if enabled else 0,
                             'flag-rises': 8 if enabled else 1, 'wait-returns': enabled,
                             'readbacks': enabled, 'sources': 3 if enabled else 0,
                             'final-tick': (tick + 8) & 255 if enabled else tick,
                             'unknown-queries': 0, 'program-unchanged': 1, 'nvm-unchanged': 1,
                             'caller-unchanged': 1, 'board-rejected': 1,
                             'net-rejected': enabled, 'unfinished-rejected': 1 - enabled})
                or any(not result.get(k) for k in variable if k != 'unresolved-events')
                or bool(result.get('unresolved-events')) != bool(enabled)):
            raise ValueError('CASE missing actual wait/IRQ/refresh/preservation evidence')
        # Polls are deliberately variable; they must establish real held reads.
        if result['polls'] <= 32:
            raise ValueError('POLL insufficient actual original waiting')
        periods = 8 if enabled else 9
        if not periods * 8192 <= result['elapsed-osc'] < (periods + 1) * 8192:
            raise ValueError('CLOCK original wait has wrong oscillator duration')
        if not 2 * result['elapsed-osc'] < result['clock-checks'] < 3 * result['elapsed-osc']:
            raise ValueError('CLOCK declared30MHz/12MHz bus/OSC relation differs')
        body = section[3]
        if rows(body, 'WAIT_ENTRY') != [dict(tick=tick, delta=8)]:
            raise ValueError('WAIT actual original delay argument differs')
        if rows(body, 'WAIT_RETURN') != ([dict(tick=(tick + 8) & 255, interrupts=8)] if enabled else []):
            raise ValueError('WAIT early/fabricated original completion')
        expected_rises = [dict(index=n, oscillator=n * 8192) for n in range(1, 9 if enabled else 2)]
        if rows(body, 'RTIF_RISE') != expected_rises:
            raise ValueError('RTIF exact independently derived timeout edges differ')
        returns = [dict(index=n, tick=(tick + n) & 255) for n in range(1, 9)] if enabled else []
        if rows(body, 'IRQ_RETURN') != returns:
            raise ValueError('IRQ original tick increment/return differs')
        if len(re.findall(r'^M \d+ 0x88a5$', body, re.M)) != (8 if enabled else 0):
            raise ValueError('IRQ original handler entry count differs')
        if len(re.findall(r'^M \d+ 0xbbe9$', body, re.M)) != enabled:
            raise ValueError('WAIT original RTS entry count differs')
        acknowledgements = re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0037 (0x[0-9a-f]+)$', body, re.M)
        if len(acknowledgements) != (8 if enabled else 0) or any(not int(v, 16) & 128 for v in acknowledgements):
            raise ValueError('W1C missing actual original RTIF acknowledgement')
        stores = rows(body, 'GLOBAL_STORE')
        if len(stores) != 1 or stores[0]['address'] != 0x742 or stores[0]['value'] != 7:
            raise ValueError('STORE actual original global effect differs')
        sources = [dict(address=0x9b6, x0=3), dict(address=0x9c6, x0=4), dict(address=0x742, x0=7)]
        if rows(body, 'SOURCE_REGISTER') != (sources if enabled else []):
            raise ValueError('SOURCE page queries/readback bypassed the wait')
        if rows(body, 'RESULT') != ([dict(value=7)] if enabled else []):
            raise ValueError('RESULT missing or invented original caller completion')
        wire = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)$', body, re.M)]
        expected_wire = [245, 96, 34, 7, 247]
        if enabled:
            expected_wire += [244, 1, 128, 0, 244, 1, 144, 16, 244, 96, 162, 34]
        if wire != expected_wire:
            raise ValueError('GPIO actual global-write/refresh/readback traffic differs')
        pads = rows(body, 'PAD_QUERY')
        if len(pads) != result['queries'] or any(p['known'] != p['requested'] or p['conflicts'] for p in pads):
            raise ValueError('PAD unresolved original wait/refresh consumer')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET unqualified interval audit differs')
        queries += result['queries']
        unresolved += result['unresolved-events']
        clocks += result['clock-checks']
        polls += result['polls']
    return dict(contexts=6, native_global_stores=6, completed_waits=4, expected_unfinished_waits=2,
                original_timer_interrupts=32, original_full_context_returns=32, page_refresh_reads=8,
                final_readbacks=4, known_pad_queries=queries, unresolved_queries=0,
                unresolved_events=unresolved, board_rejections=6, net_rejections=4,
                incomplete_call_rejections=2, native_cycle_checks=clocks,
                actual_wait_polls=polls)


def negative_controls(report, trace):
    controls = [
        (report.replace('irq-returns=8', 'irq-returns=7', 1), trace),
        (report.replace('wait-returns=1', 'wait-returns=0', 1), trace),
        (report.replace('final-tick=8', 'final-tick=9', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('oscillator=8192', 'oscillator=8193', 1)),
        (report, trace.replace('IRQ_RETURN index=1 tick=1', 'IRQ_RETURN index=1 tick=2', 1)),
        (report, trace.replace('SOURCE_REGISTER address=0x09b6 x0=0x0003',
                               'SOURCE_REGISTER address=0x09b6 x0=0x0004', 1)),
        (report, trace.replace('RESULT value=7', 'RESULT value=8', 1)),
        (report.replace('readbacks=0', 'readbacks=1', 1), trace),
    ]
    for altered_report, altered_trace in controls:
        if (altered_report, altered_trace) == (report, trace):
            raise ValueError('Missing fixed global-wait corruption')
        try:
            validate(altered_report, altered_trace)
        except ValueError:
            continue
        raise ValueError('Global-wait oracle accepted corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original global wait: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(report.resolve()),
                             str(trace.resolve())], capture_output=True, text=True, timeout=570, check=False)
    diagnostic = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostic)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostic.splitlines()):
        raise ValueError('Original clocked global wait failed: ' + diagnostic)
    totals = validate(report.read_text(), trace.read_text())
    controls = negative_controls(report.read_text(), trace.read_text())
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=controls,
        full_boot=False, board_qualified=False, fuzz_execution=False, physical_traffic=False,
        scope='Original reserved Global34 write, nominally scheduled8-tick wait, actual CRG IRQ/'
              'INC/W1C/RTI and original two page-state queries/readback. Missing timer delivery '
              'must leave the native wait unfinished after9 elapsed periods. Full-record/RAM/'
              'page-field/clock and scan-inhibited preconditions, not full initialization, normal '
              'panel scan, other field/program loads, physical timing, MIDI/audio or safety.'), indent=2) + '\n')
    print('Verified4 original clocked waits/refreshes and2 actual held waits with32 timer IRQs.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
