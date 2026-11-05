# SPDX-License-Identifier: GPL-3.0-only
"""Audit original reset through GPIO setup to an explicitly absent NAND peer."""

import json
from pathlib import Path
import subprocess
import sys

from original_dsp_clock import identity
from original_dsp_clocked_frontier import check_inputs
from original_dsp_voice import original_records
from original_fetch_reset_start import NATIVE_HELD
from original_fetch_write_decoder import PAIRS, dsp_check, wire_check
from original_owned_clock import record
from original_reset_start import ASSERT_TICK, RELEASE_TICK, single, verify_mcu
from original_scheduled_coupling import clock_ledger

ROOT = Path(__file__).resolve().parents[1]
SUCCESS = 'Original reset-net source, logical GPIO and NAND-ready frontier passed.\n'
GPIO = [dict(port=index, contract=1, control=control, direction=direction)
        for index, (control, direction) in enumerate(
            [(0xff, 0xf00), (0x838, 0x8bf), (0x300, 0x4ff), (0, 0)])]
PINS = dict(unselected=184320, bcr=0x1fffff, c_mask=0xf00, c_known=0xf00,
            c_levels=0x300, f_mask=0x4ff, f_known=0x4ff, f_levels=0)


def endpoint(observed, gpio, pins):
    expected = dict(replay=0, unused_pulldowns=0, release=RELEASE_TICK,
                    dsp=0xbb, r4=PAIRS, lc=0, omr=0x600389,
                    frames=97, writes=7346, pairs=1, nvm=1)
    if any(observed.get(key) != value for key, value in expected.items()):
        raise ValueError('Original GPIO/reset endpoint differs')
    if gpio != GPIO or pins != PINS:
        raise ValueError('Actual native pin configuration, output or BCR state differs')


def controls(observed, gpio, pins):
    mutations = []
    for key in ('dsp', 'r4', 'lc', 'frames'):
        mutations.append(({**observed, key: observed[key] + 1}, gpio, pins))
    for index in range(len(gpio)):
        altered = [dict(item) for item in gpio]
        altered[index]['direction'] ^= 1
        mutations.append((observed, altered, pins))
    for key in ('unselected', 'bcr', 'c_levels', 'f_known'):
        mutations.append((observed, gpio, {**pins, key: pins[key] ^ 1}))
    for changed in mutations:
        try:
            endpoint(*changed)
        except ValueError:
            continue
        raise ValueError('A fixed GPIO endpoint corruption escaped the oracle')
    return len(mutations)


def gpio_instructions(path):
    # Pinned original MOVEP immediate forms; each consumes its extension word.
    required = {0x73: 0x75, 0x75: 0x77, 0x77: 0x79, 0x79: 0x7b,
                0xa1: 0xa3, 0xa3: 0xa5, 0xa5: 0xa7}
    retired = {}
    for line in path.open():
        if not line.startswith('DSP '):
            continue
        item = record(line)
        if item['retired'] and item['native_pc'] in required:
            pc = item['native_pc']
            if pc in retired or item['pc'] != required[pc]:
                raise ValueError('Original GPIO instruction retirement/extension differs')
            retired[pc] = item['cycle']
    if set(retired) != set(required) or list(retired) != list(required):
        raise ValueError('A required original GPIO setup instruction was skipped or reordered')
    return {hex(pc): cycle for pc, cycle in retired.items()}


def validate(trace):
    lines = trace.read_text().splitlines()
    if len(lines) != 7:
        raise ValueError('Incomplete original GPIO result')
    held = single(lines[0], 'HELD_CUT')
    if held != NATIVE_HELD:
        raise ValueError('Original held-reset prefix differs')
    observed = record(lines[1])
    gpio = [single(line, 'GPIO') for line in lines[2:6]]
    pins = single(lines[6], 'PINS')
    endpoint(observed, gpio, pins)
    fault = lines[1].split(' fault=', 1)[1]
    if ('native bit X:0xffff8b pc=0x0000bb value=0x00000c' not in fault
            or not fault.endswith('Unknown DSP timer DI read at X:0xffff8b')):
        raise ValueError('Actual unknown NAND-ready input must remain visible')
    prefix = Path(str(trace) + '.prefix.trace').read_text()
    if single(prefix, 'ASSERT') != dict(tick=ASSERT_TICK, completed=ASSERT_TICK + 18,
                                       pc=0xb947, retired=90322):
        raise ValueError('Original MCU reset assertion differs')
    events_path = Path(str(trace) + '.0.trace')
    complete = events_path.read_text()
    if not complete.startswith(prefix):
        raise ValueError('The actual initial reset prefix is missing')
    events = complete[len(prefix):]
    ticks_at, _ = clock_ledger(prefix + events)
    return dict(passed=True, observed=observed, gpio=gpio, pins=pins,
                native=dsp_check(events_path, observed),
                gpio_retirements=gpio_instructions(events_path),
                mcu=verify_mcu(Path(str(trace) + '.0'), observed, ticks_at, 5),
                wire=wire_check(events, ticks_at, observed),
                fixed_oracle_controls=controls(observed, gpio, pins),
                scope='Original MCU/DSP reset, SPI, full clear, GPIO and first NAND control '
                      'setup. All selected RAM and original stage words checked; NVM unchanged. '
                      'No NAND peer, ready response, completed NAND load or instrument audio.',
                complete_instrument_boot=False, fuzz_execution=False, physical_traffic=False)


def main():
    binary, firmware, rom, destination = map(Path, sys.argv[1:])
    if not check_inputs(firmware, rom, ROOT):
        return 77
    original_records(firmware, json.loads((ROOT / 'test/reference/dsp-voice-reference.json').read_text()))
    frozen = {str(path): identity(path) for path in (binary, firmware, rom)}
    destination.parent.mkdir(parents=True, exist_ok=True)
    trace = destination.with_suffix('.trace')
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(rom.resolve()),
                             str(trace.resolve()), 'writable-pins'],
                            capture_output=True, text=True, timeout=540, check=False)
    destination.with_suffix('.stdout').write_text(result.stdout)
    destination.with_suffix('.stderr').write_text(result.stderr)
    if (result.returncode or result.stdout != SUCCESS or len(result.stderr.splitlines()) != 1
            or not result.stderr.startswith('Original logical GPIO ')
            or any(marker in result.stderr for marker in
                   ('AddressSanitizer', 'LeakSanitizer', 'runtime error:', 'ThreadSanitizer'))):
        raise ValueError('Original logical-GPIO run failed: ' + result.stdout + result.stderr)
    report = validate(trace)
    if any(identity(Path(path)) != digest for path, digest in frozen.items()):
        raise ValueError('Original logical-GPIO binary/input changed during execution')
    report['inputs'] = frozen
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('Original GPIO initialization reaches its genuine, unsupported NAND-ready input.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
