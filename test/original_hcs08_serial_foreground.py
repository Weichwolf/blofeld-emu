# SPDX-License-Identifier: GPL-3.0-only
"""Original reset/ADC/TIM/FSTAT oracle through the production foreground/SPI owner."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_board import validate as validate_one, controls as controls_one


def sections(text):
    end='END orders=2\n'
    if not text.endswith(end):
        raise ValueError('Missing complete original foreground order matrix')
    parts=text.removesuffix(end).split('ORDER index=')
    if len(parts)!=3 or parts[0]:
        raise ValueError('Unexpected original foreground order records')
    bodies=[]
    for index,part in enumerate(parts[1:]):
        header,body=part.split('\n',1)
        if header!=str(index):
            raise ValueError('Changed original foreground order')
        validate_one(body)
        bodies.append(body)
    return bodies


def main():
    binary,archive,specification,trace=map(Path,sys.argv[1:])
    if not archive.is_file():
        print('SKIP original foreground:pinned local Keyboard archive absent')
        return 77
    _,supplied=reference_inputs(archive)
    trace.parent.mkdir(parents=True,exist_ok=True)
    pages=trace.with_suffix('.pages.bin')
    pages.write_bytes(supplied)
    pins={path:hashlib.sha256(path.read_bytes()).hexdigest()
          for path in (binary,archive,specification,pages)}
    result=subprocess.run([str(binary.resolve()),str(pages.resolve())],capture_output=True,text=True,timeout=900,check=False)
    trace.with_suffix('.log').write_text(result.stdout+result.stderr)
    if result.returncode or result.stderr:
        raise ValueError('Original foreground native execution failed:'+result.stdout+result.stderr)
    bodies=sections(result.stdout)
    controls=sum(controls_one(body) for body in bodies)
    variants=(result.stdout.replace('ORDER index=1','ORDER index=0',1),
              result.stdout.removesuffix('END orders=2\n'),result.stdout+'EXTRA\n',
              result.stdout.replace('ORDER index=0\n','',1))
    for variant in variants:
        try:sections(variant)
        except ValueError:controls+=1
        else:raise ValueError('Collector accepted a fixed own-trace matrix corruption')
    if any(hashlib.sha256(path.read_bytes()).hexdigest()!=value for path,value in pins.items()):
        raise ValueError('Original foreground input changed during native execution')
    trace.write_text(result.stdout)
    report=dict(passed=True,orders=2,original_normal_resets=10,fresh_bounded_joint_continuations=4,
                strict_FSTAT_frontiers=4,collector_controls=controls,
                native_SPI_edges=0,original_code_modified=False,full_boot=False,original_SPI_consumer=False,
                hardware_equivalence=False,physical_timing_qualified=False,actual_ICG_rate_established=False,
                declared_ADC=dict(bus_hz=12000000,supply_uv=3300000,channel0_uv=0),declared_timer_phases=[0,127],
                trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                input_sha256={str(path):value for path,value in pins.items()},fuzz_execution=False,physical_traffic=False)
    trace.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified unmodified original reset through production foreground/SPI ownership with strict inputs and FSTAT refusal.')
    return 0


if __name__=='__main__':
    sys.exit(main())
