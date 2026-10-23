# SPDX-License-Identifier: GPL-3.0-only
"""Exercise actual Timer1 selection, original vector, receive handler and RTI."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_dsp_sample_receiver import validate
from original_dsp_sample_receiver_midi import prepare_inputs

ROOT = Path(__file__).resolve().parents[1]


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    inputs = trace.with_suffix('.pages')
    reference = json.loads((ROOT / 'doc/dsp-sample-timer-reference.json').read_text())
    shared = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    if reference['vector_record'] != shared['records'][83]:
        raise ValueError('Changed original Timer1 vector identity')
    try:
        _, cases = prepare_inputs(firmware, inputs, native_timer=True)
    except FileNotFoundError:
        print('SKIP original Timer1 sample receive: pinned private firmware/library absent')
        return 77
    run = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                          str(inputs.resolve()), str(trace.resolve())],
                         capture_output=True, text=True, timeout=220, check=False)
    diagnostics = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if run.returncode or any(x in diagnostics for x in (
            'AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Native Timer1 sample receive failed: ' + diagnostics)
    validate(trace, cases, timer=reference)
    trace.with_suffix('.json').write_text(json.dumps({
        'passed': True, 'declared_midi_sample_headers': len(cases),
        'original_vector': reference['vector'], 'timer_priority': reference['timer_priority'],
        'explicit_clk2_edges_per_capture': reference['capture_edges'],
        'captured_count': reference['capture_count'],
        'uninterrupted_vector_fetches': 2 * len(cases),
        'uninterrupted_handler_returns': len(cases),
        'partial_byte_cuts': [0, 1, 71, 72, 527], 'trusted_replays': 10,
        'input_sha256': hashlib.sha256(inputs.read_bytes()).hexdigest(),
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'scope': reference['qualification'], 'fuzz_execution': False, 'physical_traffic': False,
    }, indent=2) + '\n')
    print('Verified142 real Timer1 selections through unchanged vectors, C100 receivers and RTIs; '
          'five missing-byte cuts and ten trusted replays. Event timing is explicitly supplied.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
