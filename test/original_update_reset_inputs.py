# SPDX-License-Identifier: GPL-3.0-only
"""Original COP reset/bootstrap under explicit laboratory pad/PHY observations."""

import copy
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import original_update_coupled as coupled
from original_dsp_download import fixture
from original_gpio_pair import fields


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            result.update(block)
    return result.hexdigest()


def legacy_trace(path):
    destination = Path(str(path) + '.legacy')
    active = False
    sections = 0
    with path.open('rb') as source, destination.open('wb') as output:
        for line in source:
            if line == b'RESET_INPUTS_BEGIN\n':
                if active:
                    raise ValueError('Nested reset input section')
                active = True
                sections += 1
            elif line == b'RESET_INPUTS_END\n':
                if not active:
                    raise ValueError('Unmatched reset input section')
                active = False
            elif not active:
                output.write(line)
            else:
                raise ValueError('Suffix events must retain their separate raw trace')
    if active or sections != 4:
        raise ValueError('Exactly four complete reset input sections required')
    return destination


def validate_result(row, high, word):
    expected = dict(high=high, instructions=90312, cycles=233742, pc=0x8200, sp=0x2600,
                    d=0, q=(word >> (0 if high else 12)) & 63,
                    **{'dsp-pc': 0x2e61 if high else 0x2e6b,
                       'cursor': 0x144bb if high else 0x144ba, 'irqa': 101,
                       'pending-vector': 0x10, 'pending-level': 1,
                       'passive-samples': row['passive-samples'],
                       'raw-unresolved': row['raw-unresolved'],
                       'storage': 1, 'ram': 1, 'retained': 1})
    if row != expected or row['passive-samples'] <= 0 or row['raw-unresolved'] <= 0:
        raise ValueError('Original reset/bootstrap independent result differs: ' + str(row))


def suffixes(trace, blocks):
    word = blocks[4]['words'][0x144b9 - 0xd000]
    records = []
    for order, phase in coupled.MATRIX:
        for high in (0, 1):
            paths = [Path(str(trace) + f'.{order}.{phase}.q{high}.{replay}') for replay in (0, 1)]
            hashes = [digest(path) for path in paths]
            if hashes[0] != hashes[1] or paths[0].stat().st_size != paths[1].stat().st_size:
                raise ValueError('Complete native reset/bootstrap suffix replay differs')
            rows = []
            irqa = []
            with paths[0].open() as source:
                for line in source:
                    if line.startswith('RESET_INPUT_RESULT '):
                        rows.append(fields(line))
                    elif line.startswith('RESET_INPUT_IRQA '):
                        irqa.append(fields(line)['level'])
            if len(rows) != 1 or irqa != [1, 0, 1]:
                raise ValueError('Actual PJ0/IRQA history or native result absent')
            validate_result(rows[0], high, word)
            for key in ('cycles', 'q', 'cursor', 'pending-vector', 'instructions'):
                changed = copy.deepcopy(rows[0])
                changed[key] += 1
                try:
                    validate_result(changed, high, word)
                except ValueError:
                    continue
                raise ValueError('Reset input oracle accepted fixed ' + key + ' corruption')
            records.append(dict(order=order, phase=phase, **rows[0],
                                trace_sha256=hashes[0], trace_bytes=paths[0].stat().st_size))
    return records


def main():
    binary, firmware, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        coupled.ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm')))
    if not firmware.is_file() or not assembler.is_file():
        print('SKIP reset inputs: local firmware or independent assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    blocks, texts, frame, counts = coupled.inputs(firmware, trace.with_suffix('.packets'))
    templates, provenance = fixture(assembler, trace)
    packets = trace.with_suffix('.packets')
    frozen = {path: digest(path) for path in (binary, firmware, templates, packets)}
    native = subprocess.run([str(path.resolve()) for path in (binary, firmware, packets, templates, trace)] +
                            ['--reset-inputs'], capture_output=True, text=True, timeout=840, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Four native downloads and jointly clocked COP reset continuations verified.'
    if native.returncode or any(line and line != allowed and not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                               for line in output.splitlines()):
        raise ValueError('Native reset input continuation failed: ' + output)
    legacy = legacy_trace(trace)
    cases = coupled.collect(legacy)
    totals = coupled.validate(cases, blocks, texts, counts)
    corruptions = coupled.controls(cases, blocks, texts, counts, frame)
    for order, phase in coupled.MATRIX:
        base = str(trace) + f'.{order}.{phase}'
        coupled.check_frame(Path(base + '.pbm').read_bytes(), frame)
        if not Path(base + '.png').read_bytes().startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('Native original error screenshot missing')
    records = suffixes(trace, blocks)
    if any(digest(path) != value for path, value in frozen.items()):
        raise ValueError('Reset input execution inputs changed')
    receipt.write_text(json.dumps(dict(passed=True, **totals, suffixes=records,
        native_bootstrap_runs=16, suffix_replay_pairs=8, fixed_oracle_corruptions=corruptions + 40,
        independent_assembler=provenance, legacy_trace_sha256=digest(legacy),
        input_sha256={str(path.resolve()): value for path, value in frozen.items()},
        laboratory_inputs=['opposed passive Q7 low/high', 'PE2/PE3 loopback',
                           'PE5/PE6 weak pull-downs', 'fresh PHY edges15 at8049', 'PR2 idle high'],
        modeled_irqa_route='PJ0; MCU drivers/biases and documented DSP internal pull-up',
        full_boot=False, physical_pin_routing_qualified=False, whole_board_reset=False,
        live_midi_audio=False, complete_update_recovery=False, fuzz_execution=False,
        physical_traffic=False), indent=2) + '\n')
    print('Verified16 original joint reset/bootstrap suffixes,8 exact replays and strict baseline.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
