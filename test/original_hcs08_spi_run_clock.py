# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original CPU/SPI periods, actual serial pins and concurrent ADC polling."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def expected_rows(pages):
    image = {}
    for start in range(0, len(pages), 514):
        address = int.from_bytes(pages[start:start+2], 'big')
        image.update((address+i, byte) for i, byte in enumerate(pages[start+2:start+514]))
    vector = (image[0xffe0] << 8) | image[0xffe1]
    if vector != 0xea29:
        raise ValueError('Changed original SPI vector')
    poll = 0xe33a
    destination = (poll+3+int.from_bytes(bytes([image[poll+2]]), 'big', signed=True)) & 0xffff
    if image[poll] >> 1 != 7 or image[poll+1] != 0x51 or destination != poll+6:
        raise ValueError('Changed original ADC status polling instruction')
    cpu_cuts = {0,11,18,26,29,37,42,44,45,46,47,49,50,55,56,59,60,65,66,67,70,71,77,85,88,96,
                102,103,104,105,109,114,115,118,119,120,123}

    def cuts(delay):
        result = cpu_cuts.copy()
        for edge in range(1,10):
            period = 45+delay+8*edge-1
            result.update((period-1,period))
        return sorted(result)

    def endpoint(seed, order, delay):
        low = 255-seed
        return 'CLOCK', dict(seed=seed, order=order, delay=delay, periods=123, pc=poll+3, sp=0x47f,
            a=low, hx=(seed << 8) | low, ccr=0x60, known=0xef, retired=29, irq=2, ack=2,
            irq_periods=22, program_periods=101, spi_periods=123, launches=1,
            edges=9, active_periods=79-delay, remaining=1+delay, tx=9, samples=4, shift_in=0x10,
            rx_full=0, queued=0x3c, adc_remaining=53, reads=78, data_accesses=34, events=5,
            ready=2, inflight=1, next=poll+6, head=poll+3)

    def frontier(order,delay):
        return 'FRONTIER', dict(order=order,delay=delay,pc=poll+3,attempted=124,completed=123,
            spi_periods=123+int(order==2),adc_remaining=52)

    rows=[]
    periods=cut_count=replays=0
    for order in (1,2):
        for delay in (1,2):
            for seed in range(8):
                rows.extend([endpoint(seed,order,delay),frontier(order,delay)]);periods+=123
            for cut in cuts(delay):
                periods+=cut;cut_count+=1
                for replay in range(2):
                    rows.extend([('REPLAY',dict(order=order,delay=delay,cut=cut,index=replay)),
                                 endpoint(7,order,delay),frontier(order,delay)])
                    periods+=123-cut;replays+=1
    rows.append(('SUMMARY',dict(periods=periods,contexts=32,cuts=cut_count,replays=replays)))
    return rows


def validate(text, pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated original CPU/SPI clock trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for item in items:
            key, value = item.split('=')
            if key in fields:
                raise ValueError('Duplicate original CPU/SPI clock field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed original SPI source reentry, archived queue or native second frame')


def controls(text, pages):
    lines = text.splitlines(keepends=True)
    invalid = [text[:-1], text+lines[-1], ''.join(lines[1:]), lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first = lines[0].split()
        at = next(i for i, item in enumerate(first) if item.startswith(field+'='))
        first[at] = field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('REPLAY order=1 delay=1 cut=123 index=1', 'REPLAY order=1 delay=1 cut=120 index=1'))
    for candidate in invalid:
        try:
            validate(candidate, pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed original CPU/SPI clock corruption')
    return len(invalid)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original CPU/SPI clock cycles: pinned local archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    command = [str(p.resolve()) for p in (binary, supplied, trace)]
    result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=180)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed = 'Original CPU/SPI/ADC periods preserve genuine serial prefixes and the strict foreground frontier.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original CPU/SPI clock failed: '+result.stdout+result.stderr)
    text = trace.read_text()
    validate(text, pages)
    negative = controls(text, pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original CPU/SPI clock inputs changed')
    summary = expected_rows(pages)[-1][1]
    report = dict(passed=True, reference_profiles=4, foreground_contexts=32,
        checkpoint_cuts=summary['cuts'], fresh_joint_replays=summary['replays'],
        checked_endpoints=32+summary['replays'], successful_shared_periods=summary['periods'],
        actual_native_SPI_writes=True, automatic_reference_launch=True,
        actual_MOSI_MISO_edges=True, real_ADC_polling=True, CPU_SPI_ADC_each_once=True,
        completed_serial_bytes=0, serial_edges=9, sampled_MOSI_prefix=9, sampled_MISO_high=0x10, queued_successor=0x3c,
        strict_original_JMP_frontier=True, refused_period_attempts=32+summary['replays'],
        RTI_IRQ_contract='reference_immediate', collector_controls=negative,
        physical_launch_and_event_order_qualified=False, active_BDC_clocking_qualified=False,
        general_instruction_pipeline_qualified=False, full_controller_boot=False,
        host_injected_handler_PC_frame_instruction_or_SPI_byte=False,
        native_DSP_firmware_executed=False, firmware_modified=False,
        physical_traffic=False, fuzz_execution=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): digest for p, digest in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original clocked CPU/SPI bytes and concurrent ADC polling.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
