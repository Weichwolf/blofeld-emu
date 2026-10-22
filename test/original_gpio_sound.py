# SPDX-License-Identifier: GPL-3.0-only
"""Check original sound normalization, callback effects and notification lookup."""
import json
from pathlib import Path
import re
import subprocess
import sys

from original_dsp_voice import original_records
from original_gpio_pair import fields

ROOT = Path(__file__).resolve().parents[1]
NOISE = (0, 1, 7, 63, 64, 126, 127, 42)
OCTAVE = (17, 18, 27, 29, 63, 65, 113, 127)
MATRIX = tuple((reject, order, phase) for reject in (0, 1) for order in (0, 1) for phase in (0, 18))


def rows(body, label):
    return [fields(line) for line in body.splitlines() if line.startswith(label + ' ')]


def validate(report, trace):
    sections = re.findall(r'^CASE reject=(\d+) order=(\d+) phase=(\d+)\n([\s\S]*?)(?=^CASE |\Z)',
                          trace, re.M)
    lines = report.splitlines()
    if len(lines) != 8 or len(sections) != 8:
        raise ValueError('CASE incomplete original sound matrix')
    queries = unresolved = 0
    for line, section, expected in zip(lines, sections, MATRIX, strict=True):
        reject, order, phase = expected
        if tuple(map(int, section[:3])) != expected:
            raise ValueError('CASE changed sound ordering')
        result = fields(line)
        fixed = {k: v for k, v in result.items() if k not in ('queries', 'unresolved-events')}
        count = 0 if reject else 8
        if (fixed != dict(reject=reject, order=order, phase=phase, normalizations=8, decisions=8,
                          stores=count, callbacks=count, flushes=count, produced=40, consumed=40,
                          **{'policy-reads': 1, 'source-loads': 9, 'unknown-queries': 0,
                             'program-unchanged': 1, 'nvm-unchanged': 1, 'caller-unchanged': 1,
                             'cache-valid': 1, 'board-rejected': 1})
                or not result.get('queries') or not result.get('unresolved-events')):
            raise ValueError('CASE missing sound validation/callback/cache/preservation evidence')
        body = section[3]
        values, parameter = (OCTAVE, 1) if reject else (NOISE, 69)
        normalized = tuple(16 + ((min(112, max(16, v)) - 16) // 12) * 12 for v in values)
        if not reject:
            normalized = values
        if rows(body, 'NORMALIZER') != [dict(index=n, parameter=parameter, input=v)
                                        for n, v in enumerate(values)]:
            raise ValueError('NORMALIZER original inputs differ')
        if rows(body, 'DECISION') != [dict(index=n, normalized=v, accepted=1 - reject)
                                     for n, v in enumerate(normalized)]:
            raise ValueError('DECISION original normalized values/branch differ')
        if rows(body, 'SOUND_STORE') != ([] if reject else
                                        [dict(index=n, value=v) for n, v in enumerate(values)]):
            raise ValueError('STORE accepted/rejected sound effect differs')
        if rows(body, 'CALLBACK') != ([] if reject else
                                     [dict(index=n, target=0xc0be, value=v << 16)
                                      for n, v in enumerate(values)]):
            raise ValueError('CALLBACK actual redirected target/cache differs')
        if rows(body, 'FLUSH') != ([] if reject else
                                  [dict(index=n, address=0xe900 if n == 0 else 0xc0be,
                                        value=(42 if n == 0 else values[n - 1]) << 16)
                                   for n in range(8)]):
            raise ValueError('FLUSH actual old-target effect differs')
        readbacks = (64,) * 8 if reject else values
        expected_sources = [dict(address=0x74c, x0=0)]
        expected_sources += [dict(address=0x936 + parameter, x0=v) for v in readbacks]
        if rows(body, 'SOURCE_REGISTER') != expected_sources:
            raise ValueError('SOURCE real policy request and sound readbacks differ')
        if rows(body, 'RESULT') != [dict(index=n, value=v) for n, v in enumerate(readbacks)]:
            raise ValueError('RESULT original MCU replies differ')
        message = [b for v in values for b in (245, 0, parameter, v, 247)]
        if rows(body, 'QUEUE_BYTE') != [dict(index=n, byte=b, cursor=0x2d01 + n)
                                       for n, b in enumerate(message)]:
            raise ValueError('QUEUE original sound producer bytes differ')
        wire = [int(v, 16) for v in re.findall(r'^M-W \d+ 0x[0-9a-f]+ 0x0260 (0x[0-9a-f]+)$', body, re.M)]
        packet = []
        for n, v in enumerate(values):
            packet += [245, 0, parameter, v, 247]
            if n == 0:
                packet += [244, 96, 172, 44]
            packet += [244, 0, parameter | 128, parameter]
        if wire != packet:
            raise ValueError('PACKET real F5/policy/F4 traffic differs')
        pads = rows(body, 'PAD_QUERY')
        if len(pads) != result['queries'] or any(p['known'] != p['requested'] or p['conflicts']
                                                or p['levels'] & ~p['known'] for p in pads):
            raise ValueError('PAD unresolved original sound consumer')
        if len(re.findall(r'^NET_UNQUALIFIED ', body, re.M)) != result['unresolved-events']:
            raise ValueError('NET unresolved sound interval audit differs')
        queries += len(pads)
        unresolved += result['unresolved-events']
    return dict(streams=8, normalizations=64, accepted_stores=32, rejected_writes=32,
                native_sound_readbacks=64, policy_reads=8, callback_results=32, old_target_flushes=32,
                published_consumed_bytes=320, known_pad_queries=queries, unresolved_queries=0,
                unresolved_events=unresolved, board_rejections=8)


def negative_controls(report, trace):
    controls = [
        (report.replace('normalizations=8', 'normalizations=7', 1), trace),
        (report.replace('policy-reads=1', 'policy-reads=0', 1), trace),
        (report.replace('cache-valid=1', 'cache-valid=0', 1), trace),
        (report.replace('board-rejected=1', 'board-rejected=0', 1), trace),
        (report, trace.replace('accepted=1', 'accepted=0', 1)),
        (report, trace.replace('target=49342', 'target=49343', 1)),
        (report, trace.replace('FLUSH index=0 address=59648', 'FLUSH index=0 address=0', 1)),
        (report, trace.replace('NORMALIZER index=0 parameter=69 input=0',
                               'NORMALIZER index=0 parameter=69 input=1', 1)),
        (report, trace.replace('QUEUE_BYTE index=0 byte=245', 'QUEUE_BYTE index=0 byte=244', 1)),
        (report, trace.replace('RESULT index=0 value=0', 'RESULT index=0 value=1', 1)),
    ]
    for altered_report, altered_trace in controls:
        if (altered_report, altered_trace) == (report, trace):
            raise ValueError('Missing fixed sound oracle corruption')
        try:
            validate(altered_report, altered_trace)
        except ValueError:
            continue
        raise ValueError('Sound oracle accepted fixed corrupted evidence')
    return len(controls)


def main():
    binary, firmware, report = map(Path, sys.argv[1:])
    if not firmware.is_file():
        print('SKIP original sound writes: local firmware absent')
        return 77
    original_records(firmware, json.loads((ROOT / 'doc/dsp-voice-reference.json').read_text()))
    report.parent.mkdir(parents=True, exist_ok=True)
    trace, receipt = report.with_suffix('.trace'), report.with_suffix('.json')
    for path in (report, trace, receipt):
        path.unlink(missing_ok=True)
    result = subprocess.run([str(binary.resolve()), str(firmware.resolve()), str(report.resolve()),
                             str(trace.resolve())], capture_output=True, text=True, timeout=270, check=False)
    diagnostics = result.stdout + result.stderr
    report.with_suffix('.log').write_text(diagnostics)
    if result.returncode or any(row and not re.fullmatch(r'Jit@\d+: No profiler detected', row)
                                for row in diagnostics.splitlines()):
        raise ValueError('Original sound execution failed: ' + diagnostics)
    text, pin_trace = report.read_text(), trace.read_text()
    totals = validate(text, pin_trace)
    controls = negative_controls(text, pin_trace)
    receipt.write_text(json.dumps(dict(passed=True, **totals, fixed_oracle_corruptions=controls,
        board_qualified=False, full_boot=False, fuzz_execution=False, physical_traffic=False,
        scope='Declared continuous original MCU/DSP sound write/parser/callback/read path. '
              'One own matching entry, Noise Color accepted, off-grid Octave rejected; actual '
              'notification policy lookup/cache insertion with Ctrl Send disabled. '
              'No real voice allocation/audio, full boot, physical timing or safety proof.'), indent=2) + '\n')
    print('Verified32 original sound stores/callbacks,32 rejections and64 sound readbacks.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
