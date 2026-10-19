# SPDX-License-Identifier: GPL-3.0-only
"""Independent S12CPUV2 mnemonic/mode costs for native execution censuses.

The pinned primary manual in doc/hardware/sources.json supplies these costs.
This module neither imports the production cycle decoder nor predicts the path
or physical board timing. Unknown mnemonic/opcode forms fail for manual review.
Runtime census bytes and disassembly are private artifacts, never test payloads.
"""

offset9 = {0xe0, 0xe1, 0xe8, 0xe9, 0xf0, 0xf1, 0xf8, 0xf9}
offset16 = {0xe2, 0xea, 0xf2, 0xfa}
indirect = {0xe3, 0xeb, 0xf3, 0xfb, 0xe7, 0xef, 0xf7, 0xff}
truth = [0xffff, 0, 0x0505, 0xfafa, 0x5555, 0xaaaa, 0x0f0f, 0xf0f0,
         0x3333, 0xcccc, 0x00ff, 0xff00, 0xcc33, 0x33cc, 0x0c03, 0xf3fc]
inherent = {'ASRB': 1, 'NEGA': 1, 'NEGB': 1, 'CLRA': 1, 'CLRB': 1, 'NOP': 1,
            'SEI': 1, 'DEX': 1, 'PSHB': 2, 'PSHA': 2, 'PSHC': 2, 'PSHD': 2,
            'PULD': 3, 'PULC': 3, 'RTS': 5, 'BSR': 4, 'DBNE': 3, 'LEAS': 2}
loads = {'LDAB', 'LDAA', 'LDD', 'LDX', 'LDY', 'LDS', 'CMPB', 'ANDB', 'ADDD', 'SBCA', 'CPX'}
inherent.update({'ANDCC':1,'ASLA':1,'LSRA':1,'INCB':1,'INX':1,'PSHX':2,'PSHY':2,'PULA':3,'PULB':3,'PULX':3,'RTI':8,'TFR':1})
loads.update({'ANDA','CMPA','CPD','ORAA','ORAB'})
loads.update({'CPS','CPY','SUBB','SUBA','SUBD','ADDB','ADDA','EORA','EORB'})
inherent.update({'TBNE':3,'TBEQ':3,'COMB':1,'LSRB':1,'ASLD':1,'INCA':1,'CLI':1,'TSTA':1,'TSTB':1,'EXG':1,'XGDY':1,'SEX':1,'LEAX':2,'LEAY':2,'INS':2,'ABX':2,'SBA':2,'PULY':3,'MUL':1,'IDIV':12,'ASLB':1,'LSRD':1})
word_loads = {'CPS', 'CPY', 'SUBD', 'LDD', 'LDX', 'LDY', 'LDS', 'ADDD', 'CPX', 'CPD'}
stores = {'STAB', 'STAA', 'STD', 'STX', 'STY', 'CLR'}
branch_names = {'BRA','BRN','BHI','BLS','BCC','BCS','BNE','BEQ','BVC','BVS','BPL','BMI','BGE','BLT','BGT','BLE','LBRA','LBRN','LBHI','LBLS','LBCC','LBCS','LBNE','LBEQ','LBVC','LBVS','LBPL','LBMI','LBGE','LBLT','LBGT','LBLE'}
bits = {'BSET', 'BCLR', 'BRSET', 'BRCLR'}


def index_mode(post):
    if post in indirect:
        return 'indirect'
    if post in offset16:
        return 'offset16'
    if post in offset9:
        return 'offset9'
    return 'short'


def cycles(row):
    name = row['instruction'].split()[0]
    code = row['bytes']
    count = sum(row['flags'])
    if name in inherent:
        return count * inherent[name]
    if name in branch_names:
        long_branch=code[0]==0x18
        condition = code[1 if long_branch else 0] - 0x20
        assert 0 <= condition < 16
        taken_cycles,not_taken_cycles=(4,3) if long_branch else (3,1)
        return sum(amount * (taken_cycles if truth[condition] & (1 << flags) else not_taken_cycles)
                   for flags, amount in enumerate(row['flags']))
    if name in loads:
        group = (code[0] // 16) % 4
        if group == 0:
            return count * (2 if name in word_loads else 1)
        if group in (1, 3):
            return count * 3
        return count * {'short': 3, 'offset9': 3, 'offset16': 4, 'indirect': 6}[index_mode(code[1])]
    if name in {'INC','DEC','COM','NEG'}:
        if code[0] in {0x72,0x73,0x71,0x70}: return count*4
        if code[0] in {0x62,0x63,0x61,0x60}: return count*{'short':3,'offset9':4,'offset16':5,'indirect':6}[index_mode(code[1])]
        raise ValueError(row)
    if name in stores:
        group = code[0] // 16
        if group == 5:
            return count * 2
        if group == 7:
            return count * 3
        assert group == 6, row
        return count * {'short': 2, 'offset9': 3, 'offset16': 3, 'indirect': 4}[index_mode(code[1])]
    if name in bits:
        group = code[0] // 16
        branch = name in ('BRSET', 'BRCLR')
        if group == 4:
            return count * 4
        if group == 1:
            return count * (5 if branch else 4)
        assert group == 0, row
        return count * {'short': 4, 'offset9': 5 if branch else 4, 'offset16': 6}[index_mode(code[1])]
    if name in ('JSR', 'JMP'):
        indexed = code[0] in (5, 0x15)
        if not indexed:
            return count * (4 if name == 'JSR' else 3)
        extra = {'short': 0, 'offset9': 0, 'offset16': 1, 'indirect': 3}[index_mode(code[1])]
        return count * ((4 if name == 'JSR' else 3) + extra)
    if name == 'MOVB':
        assert code[:2] == [0x18, 0x0b], row
        return count * 4
    raise ValueError(row)


# S12CPUV2 WAI before interrupt: OSSSSsf, seven completed bus cycles.
inherent['WAI'] = 7
# Additional primary-manual forms potentially reached by the UI continuation:
# DECB O; EMUL ffO; ORCC P; DAA OfO. Unseen forms are not counted as coverage.
inherent.update({'DECB':1, 'EMUL':3, 'ORCC':1, 'DAA':3})
loads.add('TST')
# Independently reviewed in S12CPUV2: INY/ASRA/RORB O; DBEQ PPP/PPO;
# IDIVS OffffffffffO (12); XGDX is the one-cycle EXG D,X spelling.
inherent.update({'INY':1, 'ASRA':1, 'RORB':1, 'DBEQ':3, 'IDIVS':12, 'XGDX':1})
