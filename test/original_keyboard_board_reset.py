# SPDX-License-Identifier: GPL-3.0-only
"""Original HC12 RESET/SYNC on the production HCS08 board/wire owner."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_voice import ROOT, original_records
from original_keyboard_receiver import reference_files
from original_keyboard_reset import ResetTrace, changed_rows, fields


class BoardResetTrace(ResetTrace):
    def signal_event(self, tag, row):
        if tag != 'TARGET':
            return super().signal_event(tag, row)
        expected = dict(pc=0, hx=0, sp=255, a=0, ccr=0x68, known=0x68, status=0xc8,
                        **{'pc-known': 0, 'a-known': 0, 'hx-known': 0xff00})
        if self.stage != 'prefix' or len(self.reset_rows) != 4 or row != expected:
            raise ValueError('Changed production reset PC/A/X/CCR knowledge or missing pin phases')
        self.stage = 'running'


def validate_rows(lines):
    state = BoardResetTrace()
    for line in lines:
        state.consume(*fields(line))
    if not state.ended:
        raise ValueError('Missing complete production-board reset/SYNC matrix')
    if state.steps != 1086885:
        raise ValueError('Original reset/SYNC host instruction count changed')
    return dict(cases=state.cases, fresh_joint_suffix_replays=state.replays,
                native_HC12_steps=state.steps, accepted_application_TIM_cases=2,
                rejected_recovery_TIM_cases=1, actual_target_resets=3,
                actual_SYNC_responses=6, target_retired=0)


def validate(path):
    with path.open() as stream:
        return validate_rows(stream)


def negative_controls(path):
    for control in ('reset', 'target', 'returned', 'known', 'counter', 'host', 'replay', 'case'):
        changed = [False]
        with path.open() as stream:
            try:
                validate_rows(changed_rows(stream, control, changed))
            except ValueError:
                if not changed[0]:
                    raise ValueError('Directed collector control not applied') from None
            else:
                raise ValueError('Production-board collector accepted control:' + control)
    for key, value in (('a-known', 255), ('hx-known', 65535), ('pc-known', 1)):
        changed = [False]
        def rows():
            with path.open() as stream:
                for line in stream:
                    if not changed[0] and line.startswith('TARGET '):
                        tag, row = fields(line)
                        row[key] = value
                        changed[0] = True
                        line = tag + ' ' + ' '.join(f'{name}={item}' for name, item in row.items()) + '\n'
                    yield line
        try:
            validate_rows(rows())
        except ValueError:
            if not changed[0]:
                raise ValueError('Target-knownness collector control not applied') from None
        else:
            raise ValueError('Invented target-knownness accepted:' + key)
    return 11


def main():
    if len(sys.argv) != 7:
        raise ValueError('Expected binary,main/Keyboard archives,both native specifications,trace')
    binary, firmware, keyboard, hcs08, hc12, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original production-board reset:pinned archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, _ = reference_files(keyboard, trace)
    pins = {p: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (binary, firmware, keyboard, hcs08, hc12, helper, word)}
    result = subprocess.run([str(p.resolve()) for p in (binary, firmware, helper, trace)],
                            capture_output=True, text=True, timeout=350, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    expected = 'Original HC12 reset/SYNC uses production board memory,CPU,BDC and wire ownership.'
    if result.returncode or result.stderr or result.stdout.strip() != expected:
        raise ValueError('Original production-board reset failed:' + result.stdout + result.stderr)
    totals = validate(trace)
    totals['collector_negative_controls'] = negative_controls(trace)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != pin for p, pin in pins.items()):
        raise ValueError('Production-board reset input changed during native execution')
    report = dict(passed=True, **totals,
                  trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  full_boot=False, complete_update=False, full_caller_history=False,
                  physical_timing_qualified=False, active_background_BUSCLK_qualified=False,
                  hardware_equivalence=False, fuzz_execution=False, physical_traffic=False,
                  input_sha256={str(p): pin for p, pin in pins.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print('Verified original PE7/PB1 reset/SYNC with production target ownership and strict clock limits.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
