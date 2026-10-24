# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original normal reset/GPIO/FLASH/ICG startup; no live clock is fabricated."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs


def registers(pc, accumulator, flags, stack=0x475, hx=0x12c, known=0xef):
    return dict(pc=pc, hx=hx, sp=stack, a=accumulator, ccr=flags, known=known)


def expected_rows():
    result = []
    unknown = ('UNKNOWN HCS08 execution pc=0xe04e: HCS08 GPIO read address=0x0010 '
               'pc=0xe04e ccr=0x006c: Detached resolved GPIO pad source mask=0x00003c')
    status = ('STATUS HCS08 execution pc=0xe1b0: HCS08 ICG read address=0x004a '
              'pc=0xe1b0 ccr=0x0069 agent=CPU: live clock/status evolution is '
              'unqualified; reset flags are not ready')
    for seed in range(2):
        for trim in (0x80, 0, 0x5a, 0xff):
            for observation in [-1] + [n << 2 for n in range(16)]:
                result.append(('CASE', dict(seed=seed, trim=trim, observation=observation)))
                warm = registers(0xe000, (0xa5, 0x5a)[seed], 0x68, stack=0xff,
                                 hx=(0x22, 0xbb)[seed], known=0x68)
                result.append(('RESET', dict(reads=2, **{'ram-known': 0, 'flash-known': 3584,
                    'live-known': seed, 'live': 0 if seed == 0 else 0xa7}, **warm)))
                result.append(('GPIO', dict(retired=6176, writes=11,
                    **registers(0xe04e, 3, 0x6c, stack=0x47d))))
                if observation < 0:
                    result.append(unknown)
                elif observation & 0x38 == 0:
                    result.append(('BACKGROUND', dict(retired=6181,
                        **registers(0xe060, observation | 3, 0x6a, stack=0x47d))))
                else:
                    flags = 0x6b if trim == 0 else (0x6d if trim & 0x80 else 0x69)
                    result.append(('LOAD', dict(retired=6185, **registers(0xe1a8, trim, flags))))
                    for at, pc in ((6186, 0xe1aa), (6187, 0xe1ad), (6188, 0xe1b0)):
                        result.append(('STORE', dict(retired=at,
                            **registers(pc, trim, flags if at == 6186 else 0x69),
                            trim=trim, control1=0x44 if at == 6186 else 0x28,
                            **{'control1-known': 0xff if at == 6186 else 0xdf},
                            control2=0x30 if at == 6188 else 0, writes=at-6173)))
                    result.append(status)
                if seed == 0 and trim == 0x80 and observation in (-1, 0, 8):
                    cuts = [6176, 6186, 6188] if observation == 8 else [6176]
                    for cut in cuts:
                        for index in range(2):
                            result.append(('REPLAY', dict(cut=cut, index=index, identical=1)))
    return result


def row(line):
    if line.startswith(('UNKNOWN ', 'STATUS ')):
        return line
    tag, *pairs = line.split()
    fields = {}
    for pair in pairs:
        key, value = pair.split('=')
        if key in fields:
            raise ValueError('Duplicate original startup field')
        fields[key] = int(value)
    return tag, fields


def validate(lines):
    if [row(line) for line in lines] != expected_rows():
        raise ValueError('Changed original startup state, chronology or strict failure')


def controls(lines):
    locate = lambda prefix: next(i for i, line in enumerate(lines) if line.startswith(prefix))
    examples = [lines[:-1], lines + ['EXTRA value=1'], lines[1:],
                lines[:2] + [lines[2]] + lines[2:]]
    changes = [('CASE ', 'seed=0', 'seed=1'), ('RESET ', 'reads=2', 'reads=0'),
               ('RESET ', 'ram-known=0', 'ram-known=1'),
               ('RESET ', 'live-known=0', 'live-known=1'),
               ('LOAD ', 'retired=6185', 'retired=6184'),
               ('STORE retired=6186 ', 'ccr=109', 'ccr=105'),
               ('STORE retired=6187 ', 'control1=40', 'control1=8'),
               ('STORE retired=6187 ', 'control1-known=223', 'control1-known=255'),
               ('STATUS ', 'address=0x004a', 'address=0x004b'),
               ('STATUS ', 'pc=0xe1b0', 'pc=0xe1b2'),
               ('REPLAY ', 'identical=1', 'identical=0'),
               ('BACKGROUND ', 'retired=6181', 'retired=6182')]
    for prefix, before, after in changes:
        changed = list(lines)
        index = locate(prefix)
        if before not in changed[index]:
            raise ValueError('Invalid fixed collector control')
        changed[index] = changed[index].replace(before, after, 1)
        examples.append(changed)
    for changed in examples:
        try:
            validate(changed)
        except (ValueError, IndexError):
            continue
        raise ValueError('Collector accepted a fixed wrong-state control')
    return len(examples)


def main():
    binary, archive, specification, trace = map(Path, sys.argv[1:])
    if not archive.is_file():
        print('SKIP original HCS08 clock startup: local Keyboard archive absent')
        return 77
    _, supplied = reference_inputs(archive)
    trace.parent.mkdir(parents=True, exist_ok=True)
    pages = trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    frozen = {p: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in (binary, archive, specification, pages)}
    result = subprocess.run([str(p.resolve()) for p in (binary, pages, trace)],
                            capture_output=True, text=True, timeout=90, check=False)
    trace.with_suffix('.log').write_text(result.stdout + result.stderr)
    allowed = 'Original HCS08 GPIO/FLASH/ICG startup reaches strict clock status with unknown rates.'
    if result.returncode or result.stderr or result.stdout.strip() != allowed:
        raise ValueError('Original clock startup failed: '+result.stdout+result.stderr)
    lines = trace.read_text().splitlines()
    validate(lines)
    rejected_controls = controls(lines)
    if any(hashlib.sha256(p.read_bytes()).hexdigest() != digest for p, digest in frozen.items()):
        raise ValueError('Original clock startup input changed during execution')
    report = dict(passed=True, cases=136, unknown_pad_cases=8, actual_BGND_cases=16,
        actual_clock_status_failures=112, fresh_joint_replays=10, collector_controls=rejected_controls,
        declared_NVM_trim_profiles=4, warm_register_histories=2, physical_straps_inferred=False,
        original_code_modified=False, full_boot=False, complete_update=False,
        physical_timing_qualified=False, hardware_equivalence=False, cold_reset=False,
        fuzz_execution=False, physical_traffic=False, input_sha256={str(p): h for p,h in frozen.items()},
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
    trace.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
    print('Verified original warm reset, actual FLASH/GPIO/ICG stores and strict status frontier.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
