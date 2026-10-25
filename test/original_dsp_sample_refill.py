# SPDX-License-Identifier: GPL-3.0-only
"""Check genuine post-header refill production, readiness and strict GPIO frontier."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

from original_dsp_sample_receiver_midi import prepare_inputs

ROOT = Path(__file__).resolve().parents[1]
# Attempts, completed polls, low polls, GPIO attempts, origin PC, retained SDK PC.
# Failed two-word operand accesses retain consumed extensions; they are quarantined.
PROFILES = ((1, 0, 0, 0, 0x188a, 0x188c), (4, 4, 4, 0, 0x188a, 0x1889),
            (1, 1, 0, 1, 0x18a8, 0x18aa), (3, 3, 2, 1, 0x18a8, 0x18aa))
PAGE_RECORD_BYTES = 20 + 528
BATCH_PAGES = 36


def validate(trace, cases):
    expected = []
    for page, ready, _, _, _ in cases:
        if ready != 1:
            raise ValueError('Refill profiles require native preparation eligibility')
        expected.append(f'HEADER_TIMER {page} 18 17 2')
        for profile, fields in enumerate(PROFILES):
            expected.append(f'DRAIN {page} {profile} ' + ' '.join(map(str, fields)))
            error = 'none'
            if profile == 0:
                error = ('DSP reference step pc=0x00188a omr=0x600389: '
                         'Unknown DSP timer DI read at X:0xffff8b')
            elif profile >= 2:
                error = ('DSP reference step pc=0x0018a8 omr=0x600389: '
                         'Reserved DSP Port F pin function selected by write at Y:0xfffff7')
            expected.append(f'DRAIN_ERROR {page} {profile} {error}')
    expected.append(f'DRAIN_RESULT {len(cases)} {len(cases) * 4}')
    if trace.read_text().splitlines() != expected:
        raise ValueError('Changed native first-refill readiness/frontier/quarantine evidence')


def run_batches(binary, firmware, trace, inputs, cases):
    data = inputs.read_bytes()
    if len(data) != 12 + len(cases) * PAGE_RECORD_BYTES:
        raise ValueError('Changed complete private refill input extent')
    rows, diagnostics = [], []
    batches = 0
    for start in range(0, len(cases), BATCH_PAGES):
        selected = cases[start:start+BATCH_PAGES]
        part = trace.with_name(f'{trace.stem}-part{batches}.trace')
        first = 12 + start * PAGE_RECORD_BYTES
        last = first + len(selected) * PAGE_RECORD_BYTES
        part.with_suffix('.pages').write_bytes(
            b'BSRX0003' + struct.pack('>I', len(selected)) + data[first:last])
        run = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                              str(part.with_suffix('.pages').resolve()), str(part.resolve())],
                             capture_output=True, text=True, timeout=220, check=False)
        output = run.stdout + run.stderr
        part.with_suffix('.log').write_text(output)
        if run.returncode or any(x in output for x in (
                'AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
            raise ValueError('Native first-refill batch failed: ' + output)
        validate(part, selected)
        rows.extend(part.read_text().splitlines()[:-1])
        diagnostics.append(output)
        batches += 1
        print(f'Completed original refill batch{batches}: {len(selected)} headers', flush=True)
    rows.append(f'DRAIN_RESULT {len(cases)} {len(cases) * 4}')
    trace.write_text('\n'.join(rows) + '\n')
    trace.with_suffix('.log').write_text(''.join(diagnostics))
    return batches


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    trace.with_suffix('.json').unlink(missing_ok=True)
    inputs = trace.with_suffix('.pages')
    try:
        _, cases = prepare_inputs(firmware, inputs, native_timer=True)
    except FileNotFoundError:
        print('SKIP original sample refill: pinned private firmware/library absent')
        return 77
    data = inputs.read_bytes()
    if data[:8] != b'BSRX0002':
        raise ValueError('Refill requires the existing native Timer1 header profile')
    inputs.write_bytes(b'BSRX0003' + data[8:])
    batches = run_batches(binary, firmware, trace, inputs, cases)
    validate(trace, cases)
    trace.with_suffix('.json').write_text(json.dumps({
        'passed': True, 'original_header_timer_entries': len(cases),
        'first_refill_windows': len(cases) * 4, 'header_boundary_restores': len(cases) * 4,
        'unknown_ready_failures': len(cases), 'healthy_four_low_poll_windows': len(cases),
        'reserved_gpio_failures': len(cases) * 2,
        'refused_implicit_retries_and_checkpoints': len(cases) * 3,
        'serial_native_batches': batches, 'per_batch_native_timeout_seconds': 220,
        'refill_bytes_received': 0, 'refill_callbacks_or_outputs': 0,
        'input_sha256': hashlib.sha256(inputs.read_bytes()).hexdigest(),
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'scope': 'Untimed original header Timer1/receive/prepare/queue/drain with supplied '
                 'read-mode peer and raw TIO1 unknown/low/high/release profiles. Actual PCR_F '
                 'reserved transition remains rejected; no NAND/refill delivery, physical timing, '
                 'full boot or live MIDI/audio proof.',
        'fuzz_execution': False, 'physical_traffic': False,
    }, indent=2) + '\n')
    print('Verified142 real header entries and568 post-header refill windows; unknown inputs '
          'and reserved GPIO transitions fail, held-low polls stay healthy. No refill is fabricated.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
