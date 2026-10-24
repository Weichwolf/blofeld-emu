# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original SPI ISR with an own frame and actual master/DSP serial edges."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def expected_rows():
    rows = []
    for seed in range(8):
        rows.append(('CASE', dict(seed=seed, hx=(seed << 8) | (255-seed), a=seed ^ 255)))
        for call, (instructions, tx, latest, previous) in enumerate(
                ((14, 1, 0, 0), (25, 2, 0x12, 0), (25, 3, 0x34, 0x12), (19, 3, 0x56, 0x34))):
            rows.append(('ISR', dict(call=call, instructions=instructions, tx=tx, rx=call,
                                     latest=latest, previous=previous, control=0x54 if call == 3 else 0x74)))
            if call < 3:
                rows.append(('BYTE', dict(index=call, edges=16, samples=8,
                                          tx=(0x90, 0x3c, 0x64)[call], rx=(0x12, 0x34, 0x56)[call])))
        rows.append(('END', dict(retired=83, dsp=0x903c64, **{'return': 0xe000}, sp=0x47f, ccr=0x71)))
    return rows


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated original SPI trace')
    rows = []
    for line in text.splitlines():
        tag, *items = line.split()
        fields = {}
        for pair in items:
            key, value = pair.split('=')
            if key in fields:
                raise ValueError('Duplicate original SPI field')
            fields[key] = int(value)
        rows.append((tag, fields))
    if rows != expected_rows():
        raise ValueError('Changed original ISR, wire bits, ring cursor or native return')


def controls(text):
    lines = text.splitlines(keepends=True)
    examples = [text[:-1], text + 'EXTRA value=1\n', ''.join(lines[1:]), text + lines[-1]]
    changes = [('CASE ', 'hx=255', 'hx=0'), ('ISR call=0 ', 'instructions=14', 'instructions=13'),
               ('ISR call=1 ', 'rx=1', 'rx=0'), ('ISR call=1 ', 'latest=18', 'latest=0'),
               ('ISR call=2 ', 'previous=18', 'previous=52'),
               ('ISR call=3 ', 'control=84', 'control=116'),
               ('BYTE ', 'edges=16', 'edges=15'), ('BYTE ', 'samples=8', 'samples=7'),
               ('BYTE ', 'tx=144', 'tx=0'), ('BYTE ', 'rx=18', 'rx=0'),
               ('END ', 'retired=83', 'retired=82'), ('END ', 'ccr=113', 'ccr=121')]
    for prefix, before, after in changes:
        changed = list(lines)
        at = next(i for i, line in enumerate(lines) if line.startswith(prefix))
        if before not in changed[at]:
            raise ValueError('Invalid fixed SPI collector control')
        changed[at] = changed[at].replace(before, after, 1)
        examples.append(''.join(changed))
    for changed in examples:
        try:
            validate(changed)
        except ValueError:
            continue
        raise ValueError('Collector accepted a fixed original SPI corruption')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original SPI: local pinned Keyboard archive absent')
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
    allowed = 'Original HCS08 SPI ISR and master edges deliver actual DSP words and receive peer data.'
    if run.returncode or run.stderr or run.stdout.strip() != allowed:
        raise ValueError('Original SPI execution failed: '+run.stdout+run.stderr)
    text = trace.read_text()
    validate(text)
    negative = controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != sha for p, sha in frozen.items()):
        raise ValueError('Original SPI inputs changed during execution')
    report = dict(passed=True,declared_direct_entry_contexts=8,original_ISR_calls=32,
        actual_native_retirements=664,master_clock_transitions=384,
        transmitted_bytes=24,received_bytes=24,DSP24_bit_words=8,collector_controls=negative,
        own_IRQ_frame=True,actual_CPU_IRQ_entry=False,physical_wiring_qualified=False,
        physical_timing_qualified=False,ICG_clock_supplied=False,full_boot=False,
        native_DSP_firmware_executed=False,firmware_modified=False,fuzz_execution=False,
        physical_traffic=False,hardware_equivalence=False,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        input_sha256={str(p): sha for p, sha in frozen.items()},
        scope='Original32 native SPI ISR bodies with explicit own entry/frame/RAM inputs, '
        'actual SPIS/SPID flag acknowledgements, three16-edge HCS08 master bytes per context '
        'and actual DSP SHI24-bit receive/known return data. Native PSHH/PULH/RTI restore '
        'own foreground registers. This does not establish CPU IRQ admission, full controller '
        'boot, clock/transfer/completion latency, PCB wiring or original DSP event consumption.')
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original SPI ISR ring/data/return state and actual DSP serial words.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
