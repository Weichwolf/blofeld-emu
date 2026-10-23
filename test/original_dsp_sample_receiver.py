# SPDX-License-Identifier: GPL-3.0-only
"""Pin original logical pages and independently check the native sample chain."""

import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

from original_dsp_voice import original_records
from original_dsp_sample import controls, predict

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from inspect_wpc import inspect
from wpc_catalogue import catalogue, page_at, word


def prepare(firmware, destination, limit=None):
    reference = json.loads((ROOT / 'doc/dsp-sample-receiver-reference.json').read_text())
    shared = json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text())
    xdata, ydata = original_records(firmware, shared)
    if (reference['records'] != [shared['records'][i] for i in (39, 40, 41, 79, 84, 114)] or
            reference['midi_sha256'] != shared['midi_sha256']):
        raise ValueError('Changed original receive/prepare/queue/sampler input records')
    decoded, _ = inspect()
    _, samples = catalogue(decoded)
    controls()
    assert len(samples) == 136
    cases = []
    with destination.open('wb') as output:
        count = len(samples) if limit is None else limit
        output.write(b'BSRX0001' + struct.pack('>I', count))
        for sample in samples[:count]:
            page = page_at(decoded, sample.header_page)
            end = word(page, 39)
            pitch = word(page, 30)
            assert end == sample.last_data_page and word(page, 24) == sample.first_data_page
            assert word(page, 69) & 1 and word(page, 66) == 0
            ready = int(pitch != 0 and 0x3400 <= end <= 0x20000)
            window = tuple((int.from_bytes(page[i:i + 2], 'big', signed=True) * 256) & 0xffffff
                           for i in range(72, 90, 2))
            phase, result, _, _ = predict(0, 0x0c0000, ydata[0xb1f], 0, 0, window, xdata)
            if not ready:
                phase, result = 0x0c0000, 0
            assert not ready or sample.first_data_page < end
            row = (sample.header_page, ready, ydata[0xb1f], phase, result)
            output.write(struct.pack('>5I', *row) + page)
            cases.append(row)
    return reference, cases


def validate(trace, cases, timer=None):
    lines = trace.read_text().splitlines()
    expected = []
    for page, ready, _, phase, result in cases:
        expected.append(f'PAGE {page} {ready} {phase} {result}')
        if timer is not None:
            expected.append(f"TIMER {page} {timer['capture_edges']} {timer['capture_count']} 2")
    for cut in (0, 1, 71, 72, 527):
        expected.append(f'CUT {cut} replays=2')
        if timer is not None:
            expected.append(f"TIMER_CUT {cut} {timer['capture_edges']} {timer['capture_count']} 2")
    active = sum(row[1] for row in cases)
    expected += [f'RESULT {len(cases)} {active} 5 10']
    if lines != expected:
        raise ValueError('Missing or changed original page/gate/output/cut/replay evidence')


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    trace.parent.mkdir(parents=True, exist_ok=True)
    inputs = trace.with_suffix('.pages')
    try:
        reference, cases = prepare(firmware, inputs)
    except FileNotFoundError:
        print('SKIP original sample receive: pinned local firmware/library/client references absent')
        return 77
    assert len(cases) == 136 and sum(row[1] for row in cases) == 100
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()),
                             str(inputs.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=220, check=False)
    diagnostics = result.stdout + result.stderr
    trace.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(marker in diagnostics for marker in (
            'AddressSanitizer', 'LeakSanitizer', 'runtime error:')):
        raise ValueError('Native sample chain failed: ' + diagnostics)
    validate(trace, cases)
    trace.with_suffix('.json').write_text(json.dumps({
        'passed': True, 'headers': len(cases), 'ready_and_first_refill': 100,
        'raw_relative_headers_unready': 36, 'page_bytes_per_case': 528,
        'native_header_words': 24, 'native_high_aligned_payload_words': 228,
        'fault_cuts': [0, 1, 71, 72, 527], 'trusted_replays': 10,
        'input_sha256': hashlib.sha256(inputs.read_bytes()).hexdigest(),
        'trace_sha256': hashlib.sha256(trace.read_bytes()).hexdigest(),
        'scope': reference['qualification'], 'fuzz_execution': False, 'physical_traffic': False,
    }, indent=2) + '\n')
    print('Verified136 original GPIO page receives,100 real preparation/refill/sample paths, '
          '36 native silent callbacks and five partial-byte faults with ten trusted replays.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
