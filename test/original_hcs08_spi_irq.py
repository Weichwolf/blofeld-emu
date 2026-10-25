# SPDX-License-Identifier: GPL-3.0-only
"""Original SPI handler through actual architecture IRQ entry, with unknown CCR H."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def expected_rows():
    rows = []
    for seed in range(8):
        for pipeline in (0, 1):
            rows.append(('CASE', dict(seed=seed, pipeline=pipeline,
                                      hx=(seed << 8) | (255-seed), a=seed ^ 255)))
            results = ((14, 1, 0, 0, 0), (14, 2, 0, 0, 0),
                       (25, 3, 1, 0x12, 0), (19, 3, 2, 0x34, 0x12)) if pipeline else (
                       (14, 1, 0, 0, 0), (25, 2, 1, 0x12, 0),
                       (25, 3, 2, 0x34, 0x12), (19, 3, 3, 0x56, 0x34))

            def irq(call):
                instructions, tx, rx, latest, previous = results[call]
                return ('IRQ', dict(call=call, instructions=instructions, tx=tx, rx=rx,
                                    latest=latest, previous=previous,
                                    control=0x54 if call == 3 else 0x74, ccr=0x61, known=0xef))

            rows.append(irq(0))
            for index in range(3):
                if pipeline:
                    rows.append(irq(index+1))
                rows.append(('BYTE', dict(index=index, edges=16, samples=8,
                                           tx=(0x90, 0x3c, 0x64)[index],
                                           rx=(0x12, 0x34, 0x56)[index])))
                if not pipeline:
                    rows.append(irq(index+1))
            rows.append(('END', dict(retired=72 if pipeline else 83, entries=4, ack=4,
                                    dsp=0x903c64, pending_rx=pipeline, receive=0x56,
                                    **{'return': 0xe000}, sp=0x47f)))
    return rows


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original SPI IRQ trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for pair in items:
            key, value = pair.split('=')
            if key in fields:
                raise ValueError('Duplicate original SPI IRQ field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows():
        raise ValueError('Changed original IRQ/frame, wire data, ring cursor or unread RX state')


def controls(text):
    lines = text.splitlines(keepends=True)
    examples = [text[:-1], text+'EXTRA value=1\n', ''.join(lines[1:]), text+lines[-1]]
    changes = [('CASE ', 'hx=255', 'hx=0'), ('IRQ call=0 ', 'instructions=14', 'instructions=13'),
               ('IRQ call=1 ', 'rx=1', 'rx=0'), ('IRQ call=1 ', 'latest=18', 'latest=0'),
               ('IRQ call=2 ', 'previous=18', 'previous=52'),
               ('IRQ call=3 ', 'control=84', 'control=116'),
               ('IRQ call=0 ', 'known=239', 'known=255'), ('IRQ call=0 ', 'ccr=97', 'ccr=105'),
               ('BYTE ', 'edges=16', 'edges=15'), ('BYTE ', 'rx=18', 'rx=0'),
               ('END ', 'entries=4', 'entries=3'), ('END ', 'ack=4', 'ack=0'),
               ('END ', 'retired=83', 'retired=82'), ('END ', 'pending_rx=0', 'pending_rx=1')]
    for prefix, before, after in changes:
        changed = list(lines)
        at = next(i for i, line in enumerate(lines) if line.startswith(prefix) and before in line)
        changed[at] = changed[at].replace(before, after, 1)
        examples.append(''.join(changed))
    for changed in examples:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed original SPI IRQ corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original SPI IRQ: local pinned Keyboard archive absent')
        return 77
    _, pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    supplied = trace.with_suffix('.pages.bin')
    supplied.write_bytes(pages)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary, supplied, trace)],
                         capture_output=True, text=True, timeout=60, check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    allowed = 'Original HCS08 SPI interrupts preserve partial CCR through actual IRQ and RTI.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original SPI IRQ execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original SPI IRQ inputs changed during execution')
    report = dict(passed=True, declared_foreground_contexts=16, original_ISR_calls=64,
        actual_native_retirements=1240, hardware_frame_writes=320, master_clock_transitions=768,
        transmitted_bytes=48, received_bytes=48, stored_RX_bytes=40, unread_final_RX_bytes=8,
        DSP24_bit_words=16, collector_controls=negative, actual_CPU_IRQ_entry=True,
        injected_handler_PC_or_frame=False, unknown_CCR_H_preserved=True,
        physical_wiring_qualified=False, physical_timing_qualified=False,
        RTI_irq_contract='reference_immediate', physical_RTI_latency_qualified=False,
        IRQ_cycle_timing_qualified=False, ICG_clock_supplied=False, full_boot=False,
        native_DSP_firmware_executed=False, firmware_modified=False, fuzz_execution=False,
        physical_traffic=False, hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): sha for p, sha in frozen.items()},
        scope='Actual hardware IRQ/frame/vector/native RTI with unknown CCR H, '
        '64 original SPI handlers and two explicit untimed digital schedules. '
        'The prefetch schedule retains the genuine unread final RX byte after original IRQ masking. '
        'Foreground/RAM inputs are declared fixtures. No full controller boot,9-cycle IRQ '
        'timing, oscillator/transfer latency, actual PCB wiring or native DSP event consumption.')
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original SPI IRQ frames, partial CCR, wire bytes and scheduling-dependent RX.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
