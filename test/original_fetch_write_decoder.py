# SPDX-License-Identifier: GPL-3.0-only
"""Check complete original first-stage clear with a functional write decoder."""

import collections
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_clock import identity
from original_dsp_clocked_frontier import check_inputs
from original_dsp_voice import original_records
from original_fetch_reset_start import NATIVE_HELD
from original_owned_clock import record
from original_reset_start import ASSERT_TICK, RELEASE_TICK, single, verify_mcu
from original_scheduled_coupling import clock_ledger

ROOT = Path(__file__).resolve().parents[1]
PAIRS, SELECTED = 0x20000, 0x3000 + 0x4000 + 0x2000 + 0xa000
SUCCESS = 'Original reset-net source, functional decoder and complete clear passed.\n'


def summary_check(observed, decoder):
    expected = dict(replay=0, r4=PAIRS, lc=0, omr=0x600389, dsp=0x73,
                    frames=97, writes=7346, pairs=1, nvm=1)
    if any(observed.get(key) != value for key, value in expected.items()):
        raise ValueError('Original complete-clear native endpoint differs')
    if decoder != dict(pairs=PAIRS, selected=SELECTED, unselected=2 * PAIRS - SELECTED):
        raise ValueError('Actual decoder write census differs from independent RAM intervals')


def negative_controls(observed, decoder):
    for key in ('r4', 'lc', 'dsp', 'frames'):
        changed = {**observed, key: observed[key] + 1}
        try:
            summary_check(changed, decoder)
        except ValueError:
            continue
        raise ValueError('Native endpoint oracle accepted a changed ' + key)
    for key in ('pairs', 'selected', 'unselected'):
        changed = {**decoder, key: decoder[key] + 1}
        try:
            summary_check(observed, changed)
        except ValueError:
            continue
        raise ValueError('Decoder oracle accepted a changed ' + key)


def dsp_check(path, observed):
    periods = collections.Counter()
    end = RELEASE_TICK + 2 * 45
    pctl, cycles, retired, cleared = 0x04401d, 0, 0, 0
    for line in path.open():
        if not line.startswith('DSP '):
            continue
        item = record(line)
        period = 45 if pctl == 0x04401d else 10
        cycles += 1
        if item['cycle'] != cycles or item['end'] != end + period or item['period'] != period:
            raise ValueError('Native cycle/PLL timeline differs at the actual reference boundary')
        periods[period] += 1
        end = item['end']
        if item['pctl'] != pctl:
            if pctl != 0x04401d or item['pctl'] != 0x02e112:
                raise ValueError('Unexpected original PLL transition')
            pctl = item['pctl']
        retired += item['retired']
        if item['native_pc'] == 0x72 and item['retired']:
            cleared += 1
            expected_pc = 0x72 if cleared < PAIRS else 0x73
            if (cleared > PAIRS or item['r4'] != cleared or item['lc'] != PAIRS - cleared
                    or item['pc'] != expected_pc):
                raise ValueError('Original REP count, long-word cursor or exit differs')
    # A native callback rejection completes its real reference period, then the
    # runtime quarantines before publishing a successful-cycle observer event.
    # It does not retire the failing GPIO instruction or fault the time counter.
    if (cleared != PAIRS or cycles + 1 != observed['units']
            or end + 10 != observed['dsp_ticks'] or observed['dsp_attempts'] != cycles
            or retired != observed['dsp_retired'] or observed['clk2'] != (cycles + 1) // 2):
        raise ValueError('Complete original long-store retirement/clock census differs')
    return dict(cycles=cycles, retirements=retired, complete_long_stores=cleared,
                reference_periods=dict(periods), last_observed_tick=end,
                rejected_native_periods=1, final_tick=end + 10)


def wire_check(events, ticks_at, observed):
    stores = [(int(cycle), int(value, 16)) for cycle, value in re.findall(
        r'^cycle=(\d+) write=0x0001:(0x[0-9a-f]+)$', events, re.M)]
    wires = [(int(tick), int(value, 16), int(frame)) for tick, value, frame in re.findall(
        r'^B (\d+) 0x[0-9a-f]+ (0x[0-9a-f]+) frames=(\d+)$', events, re.M)]
    if len(stores) != observed['writes'] or len(wires) != len(stores):
        raise ValueError('Actual native Port-B writes and delivered observations differ')
    for (cycle, value), (tick, received, _) in zip(stores, wires):
        if value != received or tick != ticks_at(cycle) * 9:
            raise ValueError('A native Port-B effect differs in data or delivery time')
    # After the97 complete frames, later MCU activity changes unrelated B6/B7.
    # All SPI pins remain low: selection changes, but no further clock edge or
    # complete frame is delivered before the DSP reaches Port C.
    if (wires[7274][2] != 97 or len(wires[7275:]) != 71
            or any(frame != 97 or value & 0x16 for _, value, frame in wires[7275:])):
        raise ValueError('Later original Port-B stores must not invent another SPI frame')
    return dict(actual_native_stores=len(stores), frames=97,
                subsequent_unclocked_stores=len(wires[7275:]))


def validate(trace):
    lines = trace.read_text().splitlines()
    if len(lines) != 3 or not lines[1].startswith('STOP ') or not lines[2].endswith(
            ' policy=internal-writable-storage'):
        raise ValueError('Missing actual functional-decoder result or explicit contract')
    held = single(lines[0], 'HELD_CUT')
    if held != NATIVE_HELD:
        raise ValueError('Original reset/input prefix changed before the decoder experiment')
    observed = record(lines[1])
    decoder = record(lines[2].split(' policy=', 1)[0])
    summary_check(observed, decoder)
    fault = lines[1].split(' fault=', 1)[1]
    if ('Unqualified DSP GPIO reserved-bit write' not in fault
            or 'pc=0x000073' not in fault or 'Unmapped DSP memory write' in fault):
        raise ValueError('Original execution must expose its actual next Port-C access')
    prefix = Path(str(trace) + '.prefix.trace').read_text()
    if single(prefix, 'ASSERT') != dict(tick=ASSERT_TICK, completed=ASSERT_TICK + 18,
                                       pc=0xb947, retired=90322):
        raise ValueError('Original MCU reset assertion differs')
    events_path = Path(str(trace) + '.0.trace')
    complete_events = events_path.read_text()
    if not complete_events.startswith(prefix):
        raise ValueError('The functional run must retain its actual held-reset prefix')
    events = complete_events[len(prefix):]
    ticks_at, _ = clock_ledger(prefix + events)
    native = dsp_check(events_path, observed)
    mcu = verify_mcu(Path(str(trace) + '.0'), observed, ticks_at, 5)
    wire = wire_check(events, ticks_at, observed)
    negative_controls(observed, decoder)
    return dict(passed=True, decoder=decoder, native=native, mcu=mcu, wire=wire,
                fixed_oracle_controls=7, complete_instrument_boot=False,
                scope='Original MCU/DSP reset and SPI, complete unmodified first-stage clear; '
                      'functional low-128K X/Y write decoder. All77824 selected words checked '
                      'zero and184320 actual unselected writes counted. Stops at the original '
                      'Port-C direction access; no GPIO/NAND completion or instrument audio.',
                fuzz_execution=False, physical_traffic=False)


def main():
    binary, firmware, rom, destination = map(Path, sys.argv[1:])
    if not check_inputs(firmware, rom, ROOT):
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    frozen = {str(p): identity(p) for p in (binary, firmware, rom)}
    destination.parent.mkdir(parents=True, exist_ok=True)
    trace = destination.with_suffix('.trace')
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(rom.resolve()),
                             str(trace.resolve()), 'writable-storage'],
                            capture_output=True, text=True, timeout=540, check=False)
    trace.with_suffix('.stdout').write_text(result.stdout)
    trace.with_suffix('.stderr').write_text(result.stderr)
    if (result.returncode or result.stdout != SUCCESS or len(result.stderr.splitlines()) != 1
            or not result.stderr.startswith('Original functional decoder ')
            or any(marker in result.stderr for marker in
                   ('AddressSanitizer', 'LeakSanitizer', 'runtime error:', 'ThreadSanitizer'))):
        raise ValueError('Original functional-decoder run failed: ' + result.stdout + result.stderr)
    report = validate(trace)
    if any(identity(Path(p)) != h for p, h in frozen.items()):
        raise ValueError('Original functional-decoder input/binary changed during execution')
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('Complete original131072-pair clear and actual next GPIO boundary verified.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
