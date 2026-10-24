# SPDX-License-Identifier: GPL-3.0-only
"""Original native HC12 reset/SYNC, reset peer and free-running TIM observations."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from original_keyboard_receiver import reference_files
from original_dsp_voice import ROOT, original_records


def fields(line):
    tag, *values = line.split()
    return tag, {k: int(v) for k, v in (f.split('=') for f in values)}


class ResetTrace:
    expected = ((1, 2), (14, 2), (1, 6))

    def __init__(self):
        self.cases = self.replays = self.steps = 0
        self.stage = 'closed'
        self.ended = False

    def first_after(self, ticks):
        phase = self.case['phase']
        return phase + ((ticks - phase) // 15 + 1) * 15

    def begin(self, row):
        if self.stage != 'closed' or self.cases >= 3 or (row['phase'], row['prescale']) != self.expected[self.cases]:
            raise ValueError('Changed case order, clock inputs or missing case completion')
        self.case, self.stage = row, 'prefix'
        self.hosts = self.commands = self.case_replays = 0
        self.assertion = self.release = None
        self.reset_driver = 2
        self.reset_rows, self.sync_rows, self.readings = [], [], []

    def reset_event(self, tag, row):
        if self.stage != 'prefix':
            raise ValueError('Reset data outside the original reset prefix')
        if tag == 'ASSERT':
            if self.assertion is not None or row['pc'] != 0x1397:
                raise ValueError('RESET must come from the actual original PE7 direction write')
            self.assertion = row['ticks']
            return
        if self.assertion is None:
            raise ValueError('Reset phase without a native assertion')
        index = len(self.reset_rows)
        if index >= 4 or row['phase'] != index + 1 or row['mode'] != (2 if index < 2 else 1):
            raise ValueError('Missing, duplicate or reordered reset drive/sample/hold/entry phase')
        first = self.first_after(self.assertion)
        if index < 2:
            edge = (34, 72)[index]
            expected_tick = first + (edge - 1) * 15
        else:
            if self.release is None:
                raise ValueError('No actual external RESET release before mode/architectural entry')
            expected_tick = self.first_after(self.release) + (6 * 15 if index == 3 else 0)
            edge = (expected_tick - first) // 15 + 1
        if row != dict(ticks=expected_tick, edges=edge, phase=index + 1, mode=2 if index < 2 else 1):
            raise ValueError('Reference reset drive/sample/entry edges changed')
        self.reset_rows.append(row)

    def signal_event(self, tag, row):
        if tag == 'PIN':
            if self.stage not in ('prefix', 'running'):
                raise ValueError('GPIO data outside native execution')
            if self.reset_driver == 1 and row['reset'] == 2:
                if self.release is not None or row['pc'] != 0x13a3 or row['bkgd'] != 1:
                    raise ValueError('Changed RESET release or BKGD-low mode stimulus')
                self.release = row['ticks']
            self.reset_driver = row['reset']
            return
        if tag == 'TARGET':
            if self.stage != 'prefix' or len(self.reset_rows) != 4 or row != dict(pc=0, **{'pc-known': 0}, hx=0x22, sp=255, a=0xa5, ccr=0x68, known=0x68, status=0xc8):
                raise ValueError('Missing reset phases or changed native pre-vector PC/register/CCR knowledge')
            self.stage = 'running'
            return
        if self.stage != 'running':
            raise ValueError('Counter/SYNC/command data before target reset or after return')
        if tag == 'SYNC':
            index = len(self.sync_rows)
            if index >= 6 or (row['phase'], row['count']) != ((1, 0), (3, 0), (0, 1), (1, 1), (3, 1), (0, 2))[index]:
                raise ValueError('Missing or reordered long-low/response/completion SYNC phase')
            self.sync_rows.append(row)
        elif tag == 'COUNTER':
            index = len(self.readings)
            if index >= 3 or row['pc'] != (0x13f1, 0x13f9, 0x13fb)[index]:
                raise ValueError('Missing or reordered actual TCNT reads/subtraction')
            self.readings.append(row)
        elif tag == 'RX':
            if row['value'] != 0x90:
                raise ValueError('Unexpected original command')
            self.commands += 1

    def returned(self, row):
        if self.stage != 'running' or len(self.readings) != 3 or len(self.sync_rows) != 6:
            raise ValueError('Incomplete reset/SYNC/TIM prefix before native return')
        accepted = int(self.case['prescale'] == 2)
        width = (self.readings[1]['value'] - self.readings[0]['value']) & 65535
        if self.readings[2]['value'] != width or int(200 <= width <= 300) != accepted:
            raise ValueError('Original subtraction/range branch differs from actual TCNT samples')
        #128 nominal4MHz target periods, CPU30MHz and /4 versus /64 TIM;
        #independent bounds include original polling/read latency and quantization.
        lower, upper = (237, 243) if accepted else (14, 16)
        if not lower <= width <= upper:
            raise ValueError('Polled width outside independent nominal/polling bounds')
        for index in (1, 4):
            low, high = self.sync_rows[index:index + 2]
            if high['cycle'] - low['cycle'] != 129 or high['ticks'] - low['ticks'] != 129 * 15:
                raise ValueError('Target low128/high1 response timing changed')
        if row != dict(accepted=accepted, steps=self.hosts, commands=accepted, syncs=2, **{'target-retired': 0}) or self.commands != accepted:
            raise ValueError('Native host-step/return/command/held-target result differs')
        self.steps += self.hosts
        self.stage = 'returned'

    def consume(self, tag, row):
        if self.ended:
            raise ValueError('Data after completion')
        if tag == 'CASE':
            self.begin(row)
        elif tag in ('ASSERT', 'RESET'):
            self.reset_event(tag, row)
        elif tag in ('PIN', 'TARGET', 'SYNC', 'COUNTER', 'RX'):
            self.signal_event(tag, row)
        elif tag == 'HOST':
            if self.stage not in ('prefix', 'running'):
                raise ValueError('Host instructions outside native case')
            self.hosts += 1
        elif tag == 'RETURN':
            self.returned(row)
        elif tag == 'REPLAY':
            if self.stage != 'returned' or row != dict(index=self.case_replays, bytes=row['bytes'], identical=1) or row['bytes'] <= 0 or self.case_replays >= 2:
                raise ValueError('Missing or duplicate fresh joint suffix replay')
            self.case_replays += 1
            self.replays += 1
        elif tag == 'CASE_END':
            if self.stage != 'returned' or self.case_replays != 2 or row != dict(**self.case, steps=self.hosts):
                raise ValueError('Missing complete case/return/replays')
            self.stage = 'closed'
            self.cases += 1
        elif tag == 'END':
            if self.stage != 'closed' or self.cases != 3 or self.replays != 6 or row != dict(cases=3, replays=6, accepted=2, rejected=1, **{'target-retired': 0}):
                raise ValueError('Incomplete original reset/SYNC matrix')
            self.ended = True
        else:
            raise ValueError('Unknown trace row')


def validate_rows(lines):
    state = ResetTrace()
    for line in lines:
        state.consume(*fields(line))
    if not state.ended:
        raise ValueError('Missing original reset/SYNC completion')
    return dict(cases=state.cases, fresh_joint_suffix_replays=state.replays, native_HC12_steps=state.steps,
                accepted_application_TIM_cases=2, rejected_recovery_TIM_cases=1,
                actual_target_resets=3, actual_SYNC_responses=6, target_retired=0)


def validate(path):
    with path.open() as stream:
        return validate_rows(stream)


def changed_rows(stream, control, changed):
    prefixes = dict(reset='RESET ', target='TARGET ', returned='RETURN ', known='TARGET ',
                    counter='COUNTER pc=5115 ', host='HOST ', replay='REPLAY ', case='CASE_END ')
    for line in stream:
        if not changed[0] and line.startswith(prefixes[control]):
            changed[0] = True
            if control == 'known':
                line = line.replace('known=104', 'known=255')
            elif control == 'counter':
                tag, row = fields(line)
                row['value'] += 1
                line = tag + ' ' + ' '.join(f'{k}={v}' for k, v in row.items()) + '\n'
            else:
                continue
        yield line


def negative_controls(path):
    for control in ('reset', 'target', 'returned', 'known', 'counter', 'host', 'replay', 'case'):
        changed = [False]
        with path.open() as stream:
            try:
                validate_rows(changed_rows(stream, control, changed))
            except ValueError:
                if not changed[0]:
                    raise ValueError('Directed collector mutation was not applied') from None
            else:
                raise ValueError('Directed collector corruption accepted: ' + control)
    return 8


def main():
    binary, firmware, keyboard, hcs08, hc12, trace = map(Path, sys.argv[1:])
    if not firmware.is_file() or not keyboard.is_file():
        print('SKIP original reset/SYNC: pinned original archives absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    helper, word, _ = reference_files(keyboard, trace)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, firmware, keyboard, hcs08, hc12, helper, word)}
    result = subprocess.run([str(p.resolve()) for p in (binary, firmware, helper, trace)], capture_output=True, text=True, timeout=350, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    if result.returncode or result.stderr or result.stdout.strip() != 'Original HC12 reset/SYNC uses native timer and actual reset/BKGD peer state.':
        raise ValueError('Original reset/SYNC failed: ' + result.stdout + result.stderr)
    totals = validate(trace)
    totals['collector_negative_controls'] = negative_controls(trace)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original reset/SYNC inputs changed during execution')
    trace.with_suffix('.json').write_text(json.dumps(dict(passed=True, **totals,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, physical_timing_qualified=False,
        full_caller_history=False, full_DSP_transport=False, hardware_equivalence=False,
        physical_traffic=False, fuzz_execution=False,
        scope='Original HC12 reset/SYNC function beneath an own native JSR caller; actual GPIO reset/BKGD '
              'peer, warm native HCS08 reset effects and continuously clocked native TIM. Explicit nominal '
              '34/38/six reset-edge contract, reset recognition/setup/hold/wiring and4MHz target assumptions; '
              'declared locked PHY/30MHz and /4 versus /64 TIM input states, not executed original startup. '
              'Target stays held; no full upload/helper/SMF stream, NVM clock or physical equivalence.'), indent=2) + '\n')
    print('Verified original reset/SYNC with actual GPIO peer, native reset knowledge and independent TIM acceptance.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
