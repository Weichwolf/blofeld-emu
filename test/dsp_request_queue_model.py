# SPDX-License-Identifier: GPL-3.0-only
"""Independent request descriptor equations; provenance in dsp-request-queue.txt."""

import re
from dsp_scheduler_model import prefix_model, initial_ram, signed, W, H, MASK, FULL_MASK, ROUNDING_MODE

TAIL,SERVICE,CALLBACK,DEFAULT,DATA=0x22d,0x28f2,0x28fe,0x1853,0x1856
FIELDS=('target','seed','tick','timer','threshold','count','status','flags','mode','a','b',
        'x0','y0','y1','x1','r0','m0','head','tail','scan')


def directed_cases():
    formats=(0,1,255,256,257,511,512,513,767,768,H-1,H,MASK,0x123456)
    heads=(0x2700,0x2704,0x27f8,0x27fc)
    cases=[]
    def add(target,mode,head,profile,window,fmt=0,scan=0):
        i=len(cases)
        bases=([0x100000]*16 if profile==0 else
               [((H if profile==2 else 0)+0x100*(j+1))&MASK for j in range(16)])
        address=((0x11e0,0,0xfffff,H,MASK)[i%5] if profile==0 else
                 ((H if profile==2 else 0)+0x100+0xe0*window+(-1,0,1)[i%3])&MASK)
        c=dict(zip(FIELDS,[target,0x123457,0xffffff,0x7fffff,H,1,MASK,0,mode,
               (0,0x00ffffffe12345,0xff800000012345)[i%3],
               (0,0x01abcdeffedcba,0xfe012345654321)[i%3],
               (0,0x48,0x210,H,MASK)[i%5],(0,0x1c00,0x200,MASK,H)[i%5],address,fmt,
               (0x720,0x28fe,0xffffff)[i%3],(MASK,0x1f,0x80ff)[i%3],head,
               (head,0x2700+((head-0x2700+4)&255),0x27f8,0x2700)[i%4],scan]))
        c['bases']=bases;cases.append(c)
    for mode in (0,ROUNDING_MODE):
        for h,head in enumerate(heads):
            for j,fmt in enumerate(formats):add(DATA,mode,head,(j+h)%3,(j+3*h)%14,fmt)
    for mode in (0,ROUNDING_MODE):
        for h,head in enumerate(heads):
            for profile in range(3):add(DEFAULT,mode,head,profile,(3*h+profile)%14,0x123456)
    for target in (TAIL,CALLBACK):
        for mode in (0,ROUNDING_MODE):
            for head in heads:add(target,mode,head,0,0,0x654321)
    for mode in (0,ROUNDING_MODE):
        for address in (0x100000,H-1):
            add(DATA,mode,heads[len(cases)%4],0,0,0x201,1);cases[-1]['y1']=address
    if len(cases)!=156 or len({tuple(c[k] for k in FIELDS)+tuple(c['bases']) for c in cases})!=156:
        raise ValueError('Incomplete or duplicated queue corpus')
    return cases


def scheduler_case(c):
    return [c[n] for n in ('target','seed','tick','timer','threshold','count','status','flags','mode','a','b','x0','y0','y1')]


def state(values,c,pc,sp,ret,registers,r0,r1,r2,m2,phase):
    expected=[pc,sp,ret,*registers]
    if len(values)!=24 or values[:7]!=expected:
        raise ValueError(f"Queue {phase} full state differs: {values[:7]} != {expected}")
    if values[8:16]!=[r0,r1,r2,0x700,0x800,0xc500,0,0]:
        raise ValueError('Queue address registers differ: '+phase)
    if values[16:]!=[c['m0'],MASK,m2,MASK,MASK,MASK,MASK,MASK]:
        raise ValueError('Queue modifier restore differs: '+phase)
    if values[7]&(ROUNDING_MODE|0xc00)!=c['mode']:
        raise ValueError('Queue rounding/scaling domain differs')


def queue_model(fmt,command,address,pointer,payload,head,tail,bases):
    residual=signed(fmt);tag=0
    if residual>=256:
        residual-=256;tag=1
        if residual>=256:residual-=256;tag=0x50
    logical=tag|command
    header=((((residual&MASK)<<16)&MASK)+logical)&MASK
    translated=signed(address);skips=0
    while skips<15 and translated>=signed(bases[skips]):
        translated+=32;skips+=1
    scan=skips==15
    count=1 if scan else 4
    writes=[(0x2700+((head-0x2700+i)&255),v) for i,v in
            enumerate([header,translated&MASK,pointer,payload][:count])]
    next_head=0x2700+((head-0x2700+count)&255)
    full=(translated*W)&FULL_MASK
    if scan:
        accumulator=full;x1=bases[15];r1=0x26b5;published=head
    else:
        accumulator=(full&(0xff<<48))|((head^tail)<<24)
        x1=tail;r1=0x26a5+skips+2;published=next_head
    return dict(header=header,registers=[accumulator,logical*W,(x1<<24)|payload,
                 (address<<24)|command],writes=writes,r1=r1,r2=next_head,published=published,
                scan=scan,skips=skips,band=tag,translated=translated)


def validate_case(block,c,caller_return):
    metrics={k:0 for k in ('complete_descriptors','scan_frontiers','head_wraps','old_head_equals_tail',
           'negative_address_extensions','address_advances','headers_band0','headers_band1','headers_band2',
           'scheduler_queue_calls','continuation_queue_calls','header_add_differs_from_or')}
    steps=0
    rows={k:[int(v,16) for v in text.split()] for k,text in
          re.findall(r'^(ENTRY|HANDOFF|QUEUE|END|LOW|RING|TABLE|CURSORS) ([^\n]+)',block,re.M)}
    regs=[c['a'],c['b'],(c['x1']<<24)|c['x0'],(c['y1']<<24)|c['y0']]
    state(rows['ENTRY'],c,c['target'],1,caller_return,regs,c['r0'],c['target'],0xc500,MASK,'entry')
    low=initial_ram(scheduler_case(c));payload,fmt=c['x0'],c['x1']
    address,command,pointer=c['y1'],c['y0'],c['r0'];sp,ret=1,caller_return
    if c['target']==TAIL:
        low,regs,_,_=prefix_model(scheduler_case(c));regs[2]=(c['x1']<<24)|c['count']
        address,command,pointer=0x11e0,0x1c00,CALLBACK;regs[3]=(address<<24)|command
        sp,ret=2,0x244
        state(rows['HANDOFF'],c,DEFAULT,sp,ret,regs,pointer,TAIL,0xc500,MASK,'scheduler handoff')
        metrics['scheduler_queue_calls']+=1
    elif c['target']==CALLBACK:
        low[0x19]=0;payload,fmt,address,command,pointer=0x48,0,0x11e0,0x200,0x720
        regs=[0,c['b'],0x48,(address<<24)|command]
        state(rows['HANDOFF'],c,DATA,sp,ret,regs,pointer,CALLBACK,0xc500,MASK,'continuation handoff')
        metrics['continuation_queue_calls']+=1
    if c['target'] in (DEFAULT,TAIL):
        payload,fmt=0x210,0;regs=[0,regs[1],payload,(address<<24)|command]
    state(rows['QUEUE'],c,DATA,sp,ret,regs,pointer,c['target'],0xc500,MASK,'queue entry')
    model=queue_model(fmt,command,address,pointer,payload,c['head'],c['tail'],c['bases'])
    if model['scan']!=bool(c['scan']):raise ValueError('Unexpected table domain')
    state(rows['END'],c,0x186a if model['scan'] else 0x1876,sp,ret,model['registers'],pointer,
          model['r1'],model['r2'],0x80ff if model['scan'] else c['m0'],'end')
    ring=[(0x234567+i*0x12345)&MASK for i in range(258)]
    for at,value in model['writes']:ring[at-0x2700+1]=value
    if rows['RING']!=ring or rows['LOW']!=low or rows['TABLE']!=[0x345678,*c['bases'],0x456789]:
        raise ValueError('Queue descriptor or guard memory differs')
    if rows['CURSORS']!=[model['published'],c['tail']]:raise ValueError('Queue cursor publication differs')
    if not model['scan'] and bool(rows['END'][7]&4)!=((c['head']^c['tail'])==0):
        raise ValueError('Old-head XOR flags differ')
    metrics['scan_frontiers' if model['scan'] else 'complete_descriptors']+=1
    metrics['head_wraps']+=not model['scan'] and model['r2']<c['head']
    metrics['old_head_equals_tail']+=c['head']==c['tail']
    metrics['negative_address_extensions']+=model['translated']<0
    metrics['address_advances']+=model['skips']
    metrics[{0:'headers_band0',1:'headers_band1',0x50:'headers_band2'}[model['band']]]+=1
    metrics['header_add_differs_from_or']+=model['header']!=(((((signed(fmt)-(256 if signed(fmt)>=256 else 0)-(256 if signed(fmt)>=512 else 0))&255)<<16)|(model['band']|command))&MASK)
    one=re.findall(r'^STEPS (\d+)$',block,re.M)
    if len(one)!=1 or not 0<int(one[0])<=200:raise ValueError('Queue instruction bound missing')
    steps+=int(one[0])
    return metrics,steps

def validate_windows(result,cases,caller_return):
    blocks=re.findall(r'^CASE ([0-9a-f]+)\n([\s\S]*?)(?=^CASE |^VISITED)',result,re.M)
    if [int(i,16) for i,_ in blocks]!=list(range(len(cases))):
        raise ValueError('Missing queue cases')
    totals={}
    steps=0
    for (_,block),case in zip(blocks,cases):
        metrics,count=validate_case(block,case,caller_return)
        for name,value in metrics.items():
            totals[name]=totals.get(name,0)+value
        steps+=count
    return totals,steps


def validate(result,cases,caller_return):
    metrics,steps=validate_windows(result,cases,caller_return)
    if metrics['complete_descriptors']!=152 or metrics['scan_frontiers']!=4 or any(v==0 for v in metrics.values()):
        raise ValueError('Incomplete queue branches: '+str(metrics))
    return metrics,steps
