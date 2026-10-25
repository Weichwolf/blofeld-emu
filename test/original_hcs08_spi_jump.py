# SPDX-License-Identifier: GPL-3.0-only
"""Pinned native JMP/scanner continuation with real concurrent serial/ADC prefixes."""

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
    target = (image[0xe33e] << 8) | image[0xe33f]
    scan = (image[target+1] << 8) | image[target+2]
    if target != 0xe7ae or scan != 0xe20e or image[scan+1] != 0xa8:
        raise ValueError('Changed pinned original JMP/scanner path')
    cpu_cuts = {0,11,18,26,29,37,42,44,45,46,47,49,50,55,56,59,60,65,66,67,70,71,77,85,88,96,
                102,103,104,105,109,114,115,118,119,120,123}

    def cuts(delay):
        result = cpu_cuts | set(range(124,135))
        for edge in range(1,10):
            period = 45+delay+8*edge-1
            result.update((period-1,period))
        return sorted(result)

    def endpoint(seed,order,delay):
        return 'JUMP', dict(seed=seed,order=order,delay=delay,periods=134,pc=scan+2,sp=0x47f,
            a=255-seed,hx=(seed << 8) | seed,ccr=0x62 if seed==0 else 0x60,known=0xef,
            retired=32,irq=2,spi_periods=134,edges=11,active_periods=90-delay,remaining=6+delay,
            tx=18,samples=5,shift_in=0x10,queued=0x3c,adc_remaining=42,reads=88,data_accesses=35,
            events=5,ready=2,inflight=1,next=scan+5,head=scan+2)

    def frontier(order,delay):
        return 'FRONTIER',dict(order=order,delay=delay,pc=scan+2,attempted=135,completed=134,
            spi_periods=134+int(order==2),adc_remaining=41)

    rows=[]
    periods=cut_count=replays=0
    for order in (1,2):
        for delay in (1,2):
            for seed in range(8):
                rows.extend([endpoint(seed,order,delay),frontier(order,delay)])
                periods+=134
            for cut in cuts(delay):
                periods+=cut
                cut_count+=1
                for replay in range(2):
                    rows.extend([('REPLAY',dict(order=order,delay=delay,cut=cut,index=replay)),
                                 endpoint(7,order,delay),frontier(order,delay)])
                    periods+=134-cut
                    replays+=1
    rows.append(('SUMMARY',dict(periods=periods,contexts=32,cuts=cut_count,replays=replays)))
    return rows


def validate(text,pages):
    if not text.endswith('\n'):
        raise ValueError('Truncated native JMP/scanner trace')
    rows=[]
    for line in text.splitlines():
        tag,*items=line.split()
        fields={}
        for item in items:
            key,value=item.split('=')
            if key in fields:
                raise ValueError('Duplicate native JMP/scanner field')
            fields[key]=int(value)
        rows.append((tag,fields))
    if rows != expected_rows(pages):
        raise ValueError('Changed native JMP/scanner, actual serial/ADC prefix or strict ASLX frontier')


def controls(text,pages):
    lines=text.splitlines(keepends=True)
    invalid=[text[:-1],text+lines[-1],''.join(lines[1:]),lines[0]+text]
    for field in expected_rows(pages)[0][1]:
        first=lines[0].split()
        at=next(i for i,item in enumerate(first) if item.startswith(field+'='))
        first[at]=field+'='+str(int(first[at].split('=')[1])+1)
        invalid.append(' '.join(first)+'\n'+''.join(lines[1:]))
    invalid.append(text.replace('REPLAY order=1 delay=1 cut=134 index=1',
                                'REPLAY order=1 delay=1 cut=133 index=1'))
    for candidate in invalid:
        try:
            validate(candidate,pages)
        except ValueError:
            continue
        raise ValueError('Collector accepted fixed native JMP/scanner corruption')
    return len(invalid)


def main():
    binary,archive,specification,trace=map(Path,sys.argv[1:])
    if not archive.is_file():
        print('SKIP original JMP/scanner cycles: pinned local archive absent')
        return 77
    _,pages=reference_inputs(archive)
    trace.parent.mkdir(parents=True,exist_ok=True)
    supplied=trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen={p:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (binary,archive,specification,supplied)}
    command=[str(p.resolve()) for p in (binary,supplied,trace)]+['--jump']
    result=subprocess.run(command,capture_output=True,text=True,check=False,timeout=180)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    allowed='Original CPU/SPI/ADC periods preserve genuine serial prefixes and the strict foreground frontier.'
    if result.returncode or result.stderr or result.stdout.strip()!=allowed:
        raise ValueError('Original native JMP/scanner failed: '+result.stdout+result.stderr)
    text=trace.read_text()
    validate(text,pages)
    negative=controls(text,pages)
    if any(hashlib.sha256(p.read_bytes()).hexdigest()!=d for p,d in frozen.items()):
        raise ValueError('Original native JMP/scanner inputs changed')
    summary=expected_rows(pages)[-1][1]
    report=dict(passed=True,reference_profiles=4,foreground_contexts=32,
        checkpoint_cuts=summary['cuts'],fresh_joint_replays=summary['replays'],
        checked_endpoints=32+summary['replays'],successful_shared_periods=summary['periods'],
        refused_period_attempts=32+summary['replays'],collector_controls=negative,
        two_native_original_JMPs=True,actual_scanner_LDX=True,strict_original_ASLX_frontier=True,
        actual_native_SPI_writes=True,automatic_reference_launch=True,real_ADC_polling=True,
        completed_serial_bytes=0,serial_edges=11,sampled_MOSI_prefix=18,sampled_MISO_prefix=2,
        sampled_MISO_high=0x10,
        queued_successor=0x3c,RTI_IRQ_contract='reference_immediate',
        physical_launch_event_order_or_JMP_phases_qualified=False,full_controller_boot=False,
        general_instruction_pipeline_qualified=False,active_BDC_clocking_qualified=False,
        host_injected_handler_PC_frame_instruction_or_SPI_byte=False,
        native_DSP_firmware_executed=False,firmware_modified=False,
        physical_traffic=False,fuzz_execution=False,hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p):d for p,d in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified two native original JMPs, scanner LDX and concurrent serial/ADC prefixes.')
    return 0


if __name__=='__main__':
    sys.exit(main())
