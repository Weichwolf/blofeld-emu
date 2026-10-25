# SPDX-License-Identifier: GPL-3.0-only
"""Independent complete batch/calibration oracle; real samples, no seeded analog history."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from original_hcs08 import reference_inputs
from original_hcs08_gpio import row

SAMPLES = 4817


def expected():
    counts, sums = [0,5,10], [0,0,0]
    center, minimum, maximum, remaining = 0,2047,0,300
    filtered, lower, upper, previous, tx, pressure = 0,0,0,0,0,0
    rows = []
    for sample in range(1,SAMPLES+1):
        for channel, code in enumerate((512,0,0)):
            sums[channel] = (sums[channel]+code) & 65535
            old = counts[channel]
            counts[channel] = (old+1) & 255
            if old & 15:
                continue
            average, sums[channel] = sums[channel]//16, 0
            if channel == 2:
                # Selected pressure voltage is zero: original inverted endpoint127.
                if pressure != 127:
                    pressure,tx = 127,(tx+3)&255
                continue
            if channel == 1:
                continue  # Zero modulation has no changed output for this history.
            filtered = (filtered+average-filtered//16) & 65535
            working = filtered//4
            derive = False
            if remaining:
                center = working if center == 0 else (center+((working-center)&65535)//8)&65535
                remaining -= 1
                if remaining == 0:
                    maximum = max(maximum,(center+500)&65535)
                    minimum = min(minimum,(center-500)&65535)
                    derive = True
            if not derive and remaining <= 1000:
                if working > maximum:
                    maximum,derive = working,True
                elif working < minimum:
                    minimum,derive = working,True
            if derive:
                upper_denominator = (maximum-center-128)&65535
                lower_denominator = (center-minimum-128)&65535
                if not upper_denominator or not lower_denominator:
                    raise ValueError('Declared sample history unexpectedly reaches an undefined division')
                upper = (8792//upper_denominator+1)&255
                lower = (8792//lower_denominator+1)&255
            elif remaining == 0:
                if working <= center:
                    value = 0 if working <= ((minimum+64)&65535) else min(8192,((working-minimum-64)*lower)&65535)
                elif working <= ((center+64)&65535):
                    value = 8192
                else:
                    product = ((working-center-64)*upper)&65535
                    value = 8192 if product <= 600 else min(16383,(product+7592)&65535)
                changed = ((previous+32-value)&65535)>63 or (previous != value and value in (0,8192,16383))
                if changed:
                    previous,tx = value,(tx+3)&255
            rows.append(dict(sample=sample,batch=(sample-1)//16+1,remaining=remaining,center=center,
                min=minimum,max=maximum,lower=lower,upper=upper,filter=filtered,sum=sums[0],
                counter=counts[0],last=previous,tx=tx))
    # Literal checkpoints constrain the independently expressed whole-history oracle.
    literal = [(1,32,8,2047,8,299,1,1),(17,542,23,2047,135,298,1,1),
               (33,1021,52,2047,255,297,1,118),(4785,8192,2041,1541,2541,0,24,24)]
    for sample,f,c,lo,hi,left,glo,ghi in literal:
        got = next(r for r in rows if r['sample']==sample)
        if (got['filter'],got['center'],got['min'],got['max'],got['remaining'],got['lower'],got['upper']) != (f,c,lo,hi,left,glo,ghi):
            raise ValueError('Independent literal calibration controls changed')
    if len(rows)!=302 or rows[-1]['last']!=8192 or tx!=6:
        raise ValueError('Independent whole calibration extent/event controls changed')
    return rows


def validate(text):
    if not text.endswith('\n'):
        raise ValueError('Truncated native calibration trace')
    lines = iter(text.splitlines())
    for wanted in expected():
        if row(next(lines)) != ('CAL',wanted):
            raise ValueError('Changed original sample batch/filter/center/range/division/event result')
    tag,values = row(next(lines))
    fixed = dict(windows=SAMPLES*3,conversions=SAMPLES*3,remaining=135,channel=0,
                 modulation_min=0,modulation_max=0,modulation_gain=1,pressure=127,tx=6,irqs=0,sp=0x475,pc=0xe7ae)
    if tag!='FINAL' or set(values)!=set(fixed)|{'retired','executing','offered'}:
        raise ValueError('Missing native complete calibration endpoint/counter schema')
    if (any(values[k]!=v for k,v in fixed.items()) or not values['retired']>SAMPLES*3 or
            not values['executing']>=values['retired'] or not values['offered']>values['executing']):
        raise ValueError('Changed actual channel progress, retained SPI ring or common clock')
    for kind in range(2):
        for index in range(2):
            if row(next(lines)) != ('REPLAY',dict(kind=kind,index=index,identical=1)):
                raise ValueError('Missing fresh late calibration/divide-helper joint continuation')
    if next(lines,None) is not None:
        raise ValueError('Extra native calibration suffix')


def controls(text):
    examples = [text.rstrip('\n'),text+'EXTRA value=1\n']
    lines = text.splitlines()
    for tag,key in [('CAL',k) for k in ('sample','batch','remaining','center','min','max','lower','upper','filter','sum','counter','last','tx')]+[
            ('FINAL','conversions'),('FINAL','remaining'),('FINAL','modulation_gain'),('FINAL','pressure'),
            ('FINAL','tx'),('FINAL','irqs'),('FINAL','pc'),('REPLAY','identical')]:
        n = next(n for n,l in enumerate(lines) if l.startswith(tag+' '))
        name,values = row(lines[n])
        values[key] += 1
        changed = list(lines)
        changed[n] = name+' '+' '.join(f'{k}={v}' for k,v in values.items())
        examples.append('\n'.join(changed)+'\n')
    for changed in examples:
        try:
            validate(changed)
        except (ValueError,StopIteration):
            continue
        raise ValueError('Collector accepted a directed native calibration corruption')
    return len(examples)


def main():
    binary,archive,specification,trace = map(Path,sys.argv[1:])
    if not archive.is_file():
        print('SKIP original calibration: pinned local Keyboard archive absent')
        return 77
    _,pages = reference_inputs(archive)
    trace.parent.mkdir(parents=True,exist_ok=True)
    supplied = trace.with_suffix('.pages.bin');supplied.write_bytes(pages)
    frozen = {p:hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary,archive,specification,supplied)}
    run = subprocess.run([str(p.resolve()) for p in (binary,supplied,trace)],capture_output=True,text=True,timeout=285,check=False)
    trace.with_suffix('.log').write_text(run.stdout+run.stderr)
    if (run.returncode or run.stderr or run.stdout.strip()!=
            'Original cold RAM,300-step pitch calibration,real ADC history and joint replay pass.'):
        raise ValueError('Original calibration failed: '+run.stdout+run.stderr)
    text = trace.read_text();validate(text);negative=controls(text)
    if any(hashlib.sha256(p.read_bytes()).hexdigest()!=d for p,d in frozen.items()):
        raise ValueError('Original calibration inputs changed during execution')
    report = dict(passed=True,calibration_steps=300,pitch_samples=SAMPLES,analog_windows=SAMPLES*3,
                  checked_pitch_batches=302,reference_pitch_code=512,host_seeded_analog_RAM_bytes=0,
                  queued_packets=2,fresh_joint_replays=4,checkpoint_cuts=2,collector_controls=negative,
                  final_center=2041,final_lower_gain=24,final_upper_gain=24,actual_IRQ_entries=0,
                  cold_RAM_origin=True,complete_board_boot=False,physical_clock_qualified=False,
                  native_DSP_firmware_executed=False,physical_traffic=False,fuzz_execution=False,
                  trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),input_sha256={str(p):d for p,d in frozen.items()})
    trace.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified original300-step calibration from real ADC sample history and native cold RAM.')
    return 0


if __name__=='__main__':
    sys.exit(main())
