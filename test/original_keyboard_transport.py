# SPDX-License-Identifier: GPL-3.0-only
"""Execute original keyboard-update packet bodies with the stock1.25 DSP decoder."""

import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from keyboard_update_reference import LAYOUTS, verified_blocks
from original_dsp_download import fixture
from original_dsp_voice import ROOT, original_records
from original_update_transport import collect, controls, packet_references, validate


def main():
    binary, firmware, keyboard, trace = map(Path, sys.argv[1:])
    assembler = Path(os.environ.get('BLOFELD_DSP_ASSEMBLER', str(
        ROOT / 'doc/local/dsp-tools-release/dsp56300-1.0.0-x86_64-unknown-linux-gnu/bin/dsp56300-asm'))).resolve()
    if not binary.is_file():
        raise ValueError('Original keyboard transport test binary is missing')
    if not all(path.is_file() for path in (firmware, keyboard, assembler)):
        print('SKIP original keyboard transport: local archives or assembler absent')
        return 77
    trace.parent.mkdir(parents=True, exist_ok=True)
    trace.unlink(missing_ok=True)
    receipt = trace.with_suffix('.json')
    receipt.unlink(missing_ok=True)
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    blocks, reference, message_corruptions = verified_blocks(keyboard)
    packets = trace.with_suffix('.packets')
    expected = packet_references(blocks, LAYOUTS, packets)
    hashes = [hashlib.sha256(b''.join(word.to_bytes(3, 'big') for word in row['words'])).hexdigest()
              for row in expected]
    if hashes != [row['stored_sha256'] for row in reference['blocks']]:
        raise ValueError('Independent complete keyboard packet words differ from pinned identities')
    templates, provenance = fixture(assembler, trace)
    frozen = {path: hashlib.sha256(path.read_bytes()).hexdigest()
              for path in (binary, firmware, keyboard, packets, templates)}
    native = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(packets.resolve()),
                             str(templates.resolve()), str(trace.resolve())],
                            capture_output=True, text=True, timeout=90, check=False)
    output = native.stdout + native.stderr
    trace.with_suffix('.log').write_text(output)
    allowed = 'Four original keyboard transfers and their complete padding stores verified.'
    if native.returncode or any(line and line != allowed and
                               not re.fullmatch(r'Jit@\d+: No profiler detected', line)
                               for line in output.splitlines()):
        raise ValueError('Original keyboard transfer execution failed: ' + output)
    observed = collect(trace)
    totals = validate(observed, expected, LAYOUTS)
    corruptions = controls(observed, expected, LAYOUTS)
    if any(hashlib.sha256(path.read_bytes()).hexdigest() != digest for path, digest in frozen.items()):
        raise ValueError('Original keyboard transfer input changed during execution')
    receipt.write_text(json.dumps(dict(
        passed=True, **totals, fixed_oracle_corruptions=corruptions,
        fixed_message_corruptions=message_corruptions, stored_block_sha256=hashes,
        independent_assembler=provenance,
        unclassified_short_message_positions=reference['unclassified_short_message_positions'],
        unclassified_short_message_effects_qualified=False,
        smf_channel_events_executed=False, hc12_objects=reference['hc12_objects'],
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        full_boot=False, complete_update=False, helper_bodies_executed=False,
        keyboard_controller_executed=False, fuzz_execution=False, physical_traffic=False,
        scope='Four isolated original keyboard update transfers through the stock1.25 DSP '
              'command handler with declared input buffer and own linear/unscaled native caller. '
              'Actual destination stores, complete padding, live instruction patches, sequence '
              'wrap, interrupted prefixes and final JMP frames. Short messages/channel events, '
              'wire/dispatch, helper bodies, external controller architecture/image completeness, '
              'flash programming/recovery, normal Keyboard boot/scanning and hardware remain '
              'unqualified.'), indent=2) + '\n')
    print('Verified4 keyboard transfers,1705 native stores and1 complete padding word.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
