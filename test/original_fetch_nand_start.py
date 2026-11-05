# SPDX-License-Identifier: GPL-3.0-only
"""Check actual original reset/NAND reads, stopping at explicit unknown storage."""

import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

from original_dsp_clock import identity
from original_dsp_clocked_frontier import check_inputs
from original_dsp_voice import original_records
from original_fetch_reset_start import NATIVE_HELD
from original_fetch_write_decoder import wire_check
from original_owned_clock import record
from original_reset_start import RELEASE_TICK, single, verify_mcu
from original_scheduled_coupling import clock_ledger

ROOT = Path(__file__).resolve().parents[1]
UNKNOWN = 'NAND read page=0x00006a column=0x10e command=0x00: unknown contents'


def source(image):
    reference = json.loads((ROOT / 'test/reference/own-dsp-nand-reference.json').read_text())
    if identity(image) != dict(bytes=reference['sparse_image_bytes'],
                               sha256=reference['sparse_image_sha256']):
        raise ValueError('Expected the pinned sparse original-derived NAND source')
    encoded = image.read_bytes()
    if struct.unpack_from('<I', encoded, 16)[0] != 107:
        raise ValueError('Unexpected sparse NAND page count')
    result = []
    for row in range(107):
        for column in range(528):
            offset = 32 + row * 1064 + 8 + column * 2
            value, known = encoded[offset:offset + 2]
            if known == 255:
                result.append(value)
            elif row != 106 or column < 270 or value != 0 or known != 0:
                raise ValueError('The sparse NAND source has unexpected unknown cells')
    if (len(result) != 56238
            or hashlib.sha256(bytes(result)).hexdigest() != reference['source_sha256']):
        raise ValueError('Original-derived NAND source differs')
    return result


def transfers(log, events, expected):
    protocol = [int(v, 16) for v in re.findall(
        r'^NAND_BYTE ticks=\d+ pc=0x000093 value=0x([0-9a-f]+) next-column=\d+$',
        log, re.M)]
    cpu = [int(v, 16) for v in re.findall(
        r'^CPU_BYTE tick=\d+ value=0x([0-9a-f]+) r5=\d+$', events, re.M)]
    if protocol != expected or cpu != expected:
        raise ValueError('NAND protocol and actual retired CPU samples must match original source')
    latches = [(int(t), int(k), int(v, 16)) for t, k, v in re.findall(
        r'^NAND_LATCH ticks=(\d+) pc=0x[0-9a-f]+ kind=(\d+) value=0x([0-9a-f]+)$',
        log, re.M)]
    if len(latches) != 107 * 5:
        raise ValueError('Expected all 107 native command/address sequences')
    for row in range(107):
        if [(k, v) for _, k, v in latches[row * 5:row * 5 + 5]] != [
                (0, 0), (1, 0), (1, row), (1, 0), (1, 0)]:
            raise ValueError('Native NAND command or row-address sequence differs')
    complete = [(int(t), int(op)) for t, op in re.findall(
        r'^NAND_AUTONOMOUS_COMPLETION ticks=(\d+) operation=(\d+)$', log, re.M)]
    if len(complete) != 107 or any(
            op != 1 or t - latches[index * 5 + 4][0] != 8100
            for index, (t, op) in enumerate(complete)):
        raise ValueError('Each modeled read must complete at its actual offered deadline')
    return dict(protocol_bytes=len(protocol), retired_cpu_bytes=len(cpu),
                commanded_rows=107, completion_deadlines=107)


def controls(log, events, expected):
    # Fixed corruptions of observed evidence; no generated input or device traffic.
    changes = [(log, 'value=0x00 next-column=1', 'value=0x01 next-column=1', True),
               (events, 'value=0x000000 r5=1', 'value=0x000001 r5=1', False),
               (log, 'kind=1 value=0x6a\n', 'kind=1 value=0x6b\n', True)]
    for text, old, new, protocol in changes:
        if old not in text:
            raise ValueError('Missing fixed NAND evidence control')
        changed = text.replace(old, new, 1)
        try:
            transfers(changed if protocol else log, events if protocol else changed, expected)
        except ValueError:
            continue
        raise ValueError('A fixed NAND data/address corruption escaped the oracle')
    first = re.search(r'^NAND_AUTONOMOUS_COMPLETION ticks=(\d+) operation=1$', log, re.M)
    if first is None:
        raise ValueError('Missing fixed NAND deadline control')
    altered = log[:first.start(1)] + str(int(first[1]) - 1) + log[first.end(1):]
    try:
        transfers(altered, events, expected)
    except ValueError:
        return 4
    raise ValueError('An early NAND completion escaped the independent deadline oracle')


def validate(trace, log, expected):
    lines = trace.read_text().splitlines()
    if (len(lines) != 3 or single(lines[0], 'HELD_CUT') != NATIVE_HELD
            or UNKNOWN not in lines[1] or 'NAND faulted=1 runtime=0' != lines[2].split(' r0=')[0]):
        raise ValueError('Actual original reset must reach the explicit unknown-tail frontier')
    observed = record(lines[1])
    for key, value in dict(release=RELEASE_TICK, dsp=0x93, r4=0xc001, lc=86,
                           omr=0x600389, frames=97, writes=7346, pairs=1).items():
        if observed[key] != value:
            raise ValueError('Actual native NAND refill endpoint differs: ' + key)
    events = Path(str(trace) + '.0.trace').read_text()
    prefix = Path(str(trace) + '.prefix.trace').read_text()
    if not events.startswith(prefix) or '\nHANDOFF ' in events:
        raise ValueError('Original reset prefix or incomplete loader scope differs')
    transfer = transfers(log, events, expected)
    count = last_cycle = last_tick = 0
    for line in events.splitlines():
        if not line.startswith('DSP '):
            continue
        item = record(line)
        if not item['retired'] or item['cycle'] <= last_cycle or item['end'] <= last_tick:
            raise ValueError('Native retirement-only trace is incomplete or reordered')
        count += 1
        last_cycle, last_tick = item['cycle'], item['end']
    if count != observed['dsp_retired']:
        raise ValueError('Native retirement-only trace is missing a completed instruction')
    ticks_at, _ = clock_ledger(events)
    return dict(passed=True, transfer=transfer, observed=observed,
                fixed_oracle_controls=controls(log, events, expected),
                mcu=verify_mcu(Path(str(trace) + '.0'), observed, ticks_at, 6),
                wire=wire_check(events[len(prefix):], ticks_at, observed),
                scope='Original MCU reset/SPI, original DSP execution and causal NAND reads '
                      'through every known source byte. Unknown unused tail remains unknown.',
                nvm_unchanged_qualified=False, loader_handoff=False,
                complete_instrument_boot=False, physical_traffic=False)


def main():
    binary, firmware, rom, nand, destination = map(Path, sys.argv[1:])
    if not nand.exists() or not check_inputs(firmware, rom, ROOT):
        return 77
    expected = source(nand)
    original_records(firmware, json.loads((ROOT / 'test/reference/dsp-voice-reference.json').read_text()))
    frozen = {str(p): identity(p) for p in (binary, firmware, rom, nand)}
    destination.parent.mkdir(parents=True, exist_ok=True)
    trace = destination.with_suffix('.trace')
    with destination.with_suffix('.stdout').open('w') as out, \
            destination.with_suffix('.stderr').open('w') as err:
        result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(rom.resolve()),
                                 str(trace.resolve()), 'nand', str(nand.resolve())],
                                stdout=out, stderr=err, timeout=3300, check=False)
    log = destination.with_suffix('.stderr').read_text()
    if (result.returncode != 1 or UNKNOWN not in log
            or any(marker in log for marker in
                   ('AddressSanitizer', 'LeakSanitizer', 'runtime error:', 'ThreadSanitizer'))):
        raise ValueError('Original connected NAND execution failed before its expected frontier')
    report = validate(trace, log, expected)
    if any(identity(Path(p)) != digest for p, digest in frozen.items()):
        raise ValueError('Connected NAND binary or original input changed during execution')
    report['inputs'] = frozen
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print('Original reset/NAND reads match all 56238 known bytes; unknown tail still refuses.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
