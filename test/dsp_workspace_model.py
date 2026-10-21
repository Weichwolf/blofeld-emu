# SPDX-License-Identifier: GPL-3.0-only
"""Independent workspace equations; provenance and scope in dsp-workspace.txt."""

from dsp_scheduler_model import signed, W, MASK, FULL_MASK, ROUNDING_MODE

VOICE, STRIDE, ACTIVE = 0xc500, 0x106, 0x6d2
MASK_ADDRESS, END_INDEX, OWNERS = 0x2f1c, 0x2f1d, 0x2f1e
ALLOCATE, REMOVE, CLEANUP = 0x2b56, 0x2b8c, 0x2b7b
OUTPUTS = [*range(ACTIVE, ACTIVE + 26), *range(MASK_ADDRESS, OWNERS + 24),
           *[VOICE + slot * STRIDE + 62 for slot in range(25)]]


def pointer(slot):return VOICE+slot*STRIDE


def initial(case):
    memory={at:0 for at in OUTPUTS}
    for slot in range(25):
        memory[pointer(slot)+17]=0
        memory[pointer(slot)+62]=0x160f
        memory[pointer(slot)+63]=0x234567
    memory[ACTIVE+26]=0x2468ac  # Readable one-past-terminator prefetch guard.
    memory.update(dict(zip(case[7::2],case[8::2])))
    return memory


def directed_cases():
    cases=[]
    def add(entry,mode,count,memory,selected=24,x0=0x13579b):
        if entry==REMOVE:
            # Direct removal leaves nonzero pool ownership/cursors untouched.
            memory={MASK_ADDRESS:0xa5a5a5,END_INDEX:0x123456,
                    **{OWNERS+i:pointer(i) for i in range(24)},
                    **{pointer(i)+17:0xc800+(i%20)*0x200 for i in range(25)},**memory}
        writes=sorted(memory.items())
        cases.append([len(cases),entry,mode,count<<9,pointer(selected),x0,len(writes),
                      *[v for pair in writes for v in pair]])
    # Every reachable successful first-fit position, odd/even alignment and
    # both prefix-only/full-except-hole occupancy patterns. No eviction needed.
    for count in range(1,7):
        for mode in (0,ROUNDING_MODE):
            for index in range(0,21-count,1 if count%2 else 2):
                for pattern in (0,1):
                    candidate=((1<<count)-1)<<(24-count-index)
                    occupied=((MASK^((1<<(24-index))-1)) if pattern==0 else MASK^candidate)
                    memory={MASK_ADDRESS:occupied,END_INDEX:0x123456}
                    for i in range(24):
                        memory[OWNERS+i]=pointer(i) if occupied&(1<<(23-i)) else 0
                        memory[ACTIVE+i]=pointer(i)
                        memory[pointer(i)+17]=0xc800+i*0x200
                    add(ALLOCATE,mode,count,memory)
    # Saturated pool: reverse eviction, multiple victims, protected list head,
    # skipped zero-workspace voices, and raw-request mask fallback.
    for mode in (0,ROUNDING_MODE):
        for count in range(1,7):
            for scenario in range(7):
                memory={MASK_ADDRESS:MASK,END_INDEX:0x123456}
                if scenario==0:entries=[]
                elif scenario==1:entries=[0]
                elif scenario==2:entries=[0,1]
                elif scenario==3:entries=[0,1,2]
                else:entries=[0,1,2,3,4]
                for i,slot in enumerate(entries):
                    memory[ACTIVE+i]=pointer(slot)
                    memory[pointer(slot)+17]=0xc800+slot*0x200
                if scenario==3:memory[pointer(2)+17]=0
                if scenario==4:memory[pointer(1)+17]=memory[pointer(3)+17]=0
                if scenario==5:
                    for slot in entries:memory[pointer(slot)+17]=0
                for i in range(24):
                    # Scenario6 removes only isolated bits per victim, forcing
                    # several retries before a wide request can be satisfied.
                    slot=(i%5 if scenario==6 else min(4,i//5))
                    memory[OWNERS+i]=pointer(slot)
                add(ALLOCATE,mode,count,memory)
    # Full list with a match at every position; shift all remaining cells,
    # including declared tail words beyond an earlier zero terminator.
    for mode in (0,ROUNDING_MODE):
        for selected in range(25):
            memory={ACTIVE+i:pointer(i) for i in range(25)}
            memory[ACTIVE+25]=0x765432
            add(REMOVE,mode,0,memory,x0=pointer(selected))
        for length in (0,1,12,24,25):
            memory={ACTIVE+i:pointer(i) for i in range(length)}
            memory[ACTIVE+length+1]=0x345678
            # Distinct valid slot pointer is absent even when all25 are present.
            add(REMOVE,mode,0,memory,x0=0xc400)
        for position in (0,1,12,24):
            memory={ACTIVE+i:(0 if i==3 else pointer(i)) for i in range(25)}
            memory[ACTIVE+25]=0x654321
            add(REMOVE,mode,0,memory,x0=pointer(position))
    for mode in (0,ROUNDING_MODE):
        for selected in range(25):
            memory={MASK_ADDRESS:MASK,END_INDEX:0x123456}
            for i in range(24):memory[OWNERS+i]=pointer(selected) if i%3 else pointer((selected+1)%25)
            add(CLEANUP,mode,0,memory,selected=selected)
    if len(cases)!=526:raise ValueError('Incomplete workspace matrix: '+str(len(cases)))
    return cases


def model(case):
    _,entry,mode,request,selected,x0,*_=case
    memory=initial(case)
    r=[0x936,entry,VOICE,0x700,0x800,selected,0,0];n=[0x53,0,0,0,0,0,0,0]
    a=0;b=request*W;x1=0x468ace;y0=0x2468ac;y1=0x3579bd
    shifts=[];resets=[];victims=[]
    def clear_owner(owner):
        mask=memory[MASK_ADDRESS]
        for i in range(24):
            if memory[OWNERS+i]==owner:
                memory[OWNERS+i]=0;mask&=~(1<<(23-i))
        memory[MASK_ADDRESS]=mask
    if entry==ALLOCATE:
        count=request>>9;step=1 if count%2 else 2
        for attempt in range(26):
            occupied=memory[MASK_ADDRESS]
            found=None
            for index in range(0,21-count,step):
                candidate=((1<<count)-1)<<(24-count-index)
                if not candidate&occupied:found=(index,candidate);break
            if found is not None:
                index,candidate=found
                memory[MASK_ADDRESS]=occupied|candidate
                memory[END_INDEX]=index+count
                for i in range(index,index+count):memory[OWNERS+i]=selected
                a=(index+count)*W;b=(0xc800+index*0x200)*W
                x0=count;x1=occupied;y0=step;y1=candidate
                r[7]=OWNERS+index+count;n[7]=index
                break
            n[4]=2
            victim=None
            # The original bound excludes index0. Preserve that head protection.
            for i in range(25,0,-1):
                r[4]=ACTIVE+i-1
                candidate=memory[ACTIVE+i];r[2]=candidate
                if candidate and memory[candidate+17]:victim=(i,candidate);break
                if r[4]<=ACTIVE:break
            if victim is None:
                resets.append(r[4]);memory[MASK_ADDRESS]=request
                # Owner pointers and the head's callback are intentionally retained.
                continue
            index,owner=victim;victims.append(owner)
            memory[owner+62]=0x160e
            cursor=index+1
            while True:
                shifts.append((owner,ACTIVE+cursor))
                value=memory[ACTIVE+cursor];memory[ACTIVE+cursor-1]=value
                cursor+=1
                if not value:break
            r[4]=ACTIVE+cursor;clear_owner(owner)
        else:raise ValueError('Workspace model did not terminate')
    elif entry==REMOVE:
        position=None
        for i in range(26):
            if memory[ACTIVE+i]==x0:position=i;break
            if not memory[ACTIVE+i]:
                a=signed(memory[ACTIVE+i+1])*W;b=(ACTIVE+i+1)*W;r[4]=ACTIVE+i+2
                break
        if position is not None:
            memory[ACTIVE+25]=0
            for i in range(position,25):memory[ACTIVE+i]=memory[ACTIVE+i+1]
            memory[x0+62]=0x160e
            a=MASK*W;b=(25-position)*W;r[1]=ACTIVE+26;r[2]=x0;r[4]=ACTIVE+position+1
            n[1]=2;n[2]=62;x0=0x160e
    elif entry==CLEANUP:
        clear_owner(selected);a=selected*W;b=memory[MASK_ADDRESS]*W
        x0=selected;y1=0xfffffe;r[7]=OWNERS+24
    else:raise ValueError('Unknown model entry')
    regs=[a&FULL_MASK,b&FULL_MASK,(x1<<24)|x0,(y1<<24)|y0]
    return memory,regs,r,n,shifts,resets,victims
