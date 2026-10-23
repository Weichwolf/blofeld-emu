# SPDX-License-Identifier: GPL-3.0-only
"""Exercise unchanged sample receive/prepare/render on declared original MIDI pages."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

from original_dsp_sample import controls, predict
from original_dsp_sample_receiver import validate
from original_dsp_voice import original_records
from sample_midi_pages_test import reference_inputs

ROOT = Path(__file__).resolve().parents[1]


def prepare_inputs(firmware, inputs, native_timer=False):
    reference, pages, (_, samples, unresolved) = reference_inputs()
    shared = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    xdata, ydata = original_records(firmware, shared)
    controls()
    assert len(samples) == 142 and sum(x.high-x.low+1 for x in unresolved) == 265
    cases = []
    with inputs.open('wb') as output:
        output.write((b'BSRX0002' if native_timer else b'BSRX0001') +
                     struct.pack('>I', len(samples)))
        for sample in samples:
            page = pages.page(sample.header_page)
            end = int.from_bytes(page[39:42], 'big')
            pitch = int.from_bytes(page[30:33], 'big')
            assert end == sample.native_end_page and page[71] == 1
            assert pitch and 0x3400 <= end <= 0x20000 and sample.first_data_page < end
            window = tuple((int.from_bytes(page[i:i+2], 'big', signed=True) * 256) & 0xffffff
                           for i in range(72, 90, 2))
            phase, result, _, _ = predict(0, 0x0c0000, ydata[0xb1f], 0, 0, window, xdata)
            row = (sample.header_page, 1, ydata[0xb1f], phase, result)
            output.write(struct.pack('>5I', *row) + page)
            cases.append(row)
    return reference, cases


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    inputs = trace.with_suffix('.pages')
    try:
        reference, cases = prepare_inputs(firmware, inputs)
    except FileNotFoundError:
        print('SKIP original sample MIDI receive: pinned private firmware/library absent')
        return 77
    run = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                          str(inputs.resolve()), str(trace.resolve())],
                         capture_output=True, text=True, timeout=220, check=False)
    diagnostics = run.stdout + run.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if run.returncode or any(x in diagnostics for x in ('AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Native declared-MIDI sample chain failed: ' + diagnostics)
    validate(trace, cases)
    trace.with_suffix('.json').write_text(json.dumps({
        'passed': True, 'declared_midi_sample_headers': 142, 'native_active_refill_render_paths': 142,
        'unresolved_catalogue_keys_retained': 265, 'native_page_bytes': 528,
        'partial_byte_cuts': [0, 1, 71, 72, 527], 'trusted_replays': 10,
        'input_sha256': hashlib.sha256(inputs.read_bytes()).hexdigest(),
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'scope': reference['qualification'] + ' Native caller/receiver conditions: doc/dsp-sample-receiver.txt.',
        'fuzz_execution': False, 'physical_traffic': False,
    }, indent=2)+'\n')
    print('Verified142 original MIDI-page receives, preparation/refill/active sample callbacks, '
          'five missing-byte cuts and ten trusted replays; unresolved catalogue keys remain explicit.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
