# SPDX-License-Identifier: GPL-3.0-only
"""Strict collector for the declared Keyboard stage2/LCD/BKGD reference fixture."""

import re

from keyboard_update_reference import LAYOUTS
from original_gpio_pair import fields
from original_keyboard_handoff import CHUNK_PCS
from original_update_transport import validate_block

MATRIX = ((0, 0, 1), (1, 18, 14))
END = dict(cases=2, replays=4, **{'target-retired': 0}, uploaded=0)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def equal(case, tag, expected):
    require(case['rows'].get(tag, []) == expected, 'Changed original chain ' + tag)


class Collector:
    background = {'PIN', 'M-W', 'NET_UNQUALIFIED'}
    transfer = {'SETUP', 'STORE', 'PREFETCH', 'PACKET', 'INTERRUPTED', 'RESULT'}
    permitted = {
        'setup': {'MCU_CLOCK', 'DSP_CLOCK'},
        'copy': {'SOURCE', 'CHUNK', 'CONSUMER'},
        'checksum': {'SOURCE', 'CHUNK', 'HELPER_SAMPLE', 'BYTE'},
        'reset': {'ASSERT', 'RESET', 'TARGET', 'SYNC', 'RX'},
        'drawing': {'TEXT', 'SYNC'},
        'trim': set(),
    }
    transitions = {'COPY_BOUNDARY': ('copy', 'entry'), 'ENTRY': ('entry', 'checksum'),
                   'PREFIX': ('checksum', 'reset'), 'SAVING_BOUNDARY': ('reset', 'drawing'),
                   'TEXT_RETURN': ('drawing', 'trim'), 'DIAGNOSTIC': ('trim', 'fault'),
                   'WIRE_FRONTIER': ('fault', 'frontier'), 'RETAINED': ('frontier', 'replays')}

    def __init__(self):
        self.cases = []
        self.stage = 'closed'
        self.ended = False
        self.preload = None

    def begin(self, row):
        index = len(self.cases)
        require(self.stage == 'closed' and index < len(MATRIX), 'Incomplete/extra chain case')
        require(row == dict(zip(('order', 'phase', 'target-phase'), MATRIX[index])),
                'Changed chain case order or declared clock inputs')
        self.current = dict(case=row, rows={}, preloads={})
        self.cases.append(self.current)
        self.stage = 'setup'

    def consume(self, tag, row):
        require(not self.ended, 'Data after complete chain matrix')
        if tag == 'CASE':
            self.begin(row)
            return
        require(self.stage != 'closed' or tag == 'END', 'Data outside a chain case')
        if tag == 'END':
            require(self.stage == 'closed' and len(self.cases) == len(MATRIX) and row == END,
                    'Incomplete chain matrix completion')
            self.ended = True
            return
        if tag == 'PRELOAD':
            expected = 2 if self.stage == 'setup' else 3
            require(self.stage in ('setup', 'preload') and row == dict(block=expected)
                    and expected not in self.current['preloads'], 'Changed original download order')
            self.preload = expected
            self.current['preloads'][expected] = []
            self.stage = 'preload'
            return
        if self.stage == 'preload' and tag != 'PRELOAD_END':
            require(tag in self.transfer, 'Unknown or misplaced native transfer row')
            self.current['preloads'][self.preload].append((tag, row))
            return
        if tag == 'PRELOAD_END':
            require(self.stage == 'preload' and list(self.current['preloads']) == [2, 3],
                    'Missing complete original downloads')
            self.stage = 'copy'
        elif tag in self.transitions:
            before, after = self.transitions[tag]
            require(self.stage == before, 'Missing/reordered chain stage: ' + tag)
            self.stage = after
        elif tag == 'REPLAY':
            require(self.stage == 'replays', 'Replay outside complete native suffix')
        elif tag == 'CASE_END':
            require(self.stage == 'replays', 'Case ended without native failure and replays')
            self.stage = 'closed'
        else:
            allowed = self.permitted.get(self.stage, set())
            require(tag in allowed or (tag in self.background
                    and self.stage in ('copy', 'checksum', 'reset', 'drawing', 'trim')),
                    'Unknown/misplaced chain trace row: ' + tag)
        rows = self.current['rows']
        if tag == 'RESET':
            require(len(rows.get('ASSERT', [])) == 1 and not rows.get('TARGET'),
                    'Reset phases precede assertion or follow target entry')
        elif tag == 'TARGET':
            require(len(rows.get('RESET', [])) == 4, 'Target entry precedes complete reset')
        elif tag == 'SYNC':
            require(len(rows.get('TARGET', [])) == 1, 'SYNC precedes actual target reset')
            if self.stage == 'drawing':
                require(rows.get('TEXT') and len(rows.get('SYNC', [])) == 6,
                        'Reference long-low recognition outside actual LCD drawing')
        elif tag == 'RX':
            require(len(rows.get('SYNC', [])) == 6, 'Command precedes the original reset/SYNCs')
        elif tag == 'TEXT_RETURN':
            require(len(rows.get('TEXT', [])) == 2 and len(rows.get('SYNC', [])) == 7,
                    'Drawing return precedes recursive measurement or reference SYNC')
        self.current['rows'].setdefault(tag, []).append(row)


def collect(text):
    state = Collector()
    for number, line in enumerate(text.splitlines(keepends=True), 1):
        require(line.endswith('\n') and line.strip(), f'Incomplete chain trace row {number}')
        tag = line.split(' ', 1)[0]
        try:
            row = line.removeprefix('DIAGNOSTIC ').rstrip('\n') if tag == 'DIAGNOSTIC' else fields(line)
            state.consume(tag, row)
        except (ValueError, KeyError) as error:
            raise ValueError(f'Chain trace row {number}: {error}') from error
    require(state.ended, 'Missing chain END record')
    return state.cases


def downloads(case, expected):
    steps = sum(validate_block(dict(rows=case['preloads'][index]), expected[index], LAYOUTS[index],
                               0xff1000, 5 if index == 2 else 1) for index in (2, 3))
    equal(case, 'PRELOAD_END', [dict(pc=0x2e00, epoch=1, steps=steps)])
    equal(case, 'MCU_CLOCK', [dict(instructions=35, cycles=76, ticks=752, divider=2)])
    equal(case, 'DSP_CLOCK', [dict(delay=44000, pctl=172050, **{'ratio-numerator': 18,
                                'ratio-denominator': 1}, timebase=1080000000)])


def handoff(case, expected):
    words = expected[2]['words']
    chunks = [(word >> shift) & 63 for word in words for shift in (18, 12, 6, 0)]
    equal(case, 'COPY_BOUNDARY', [dict(pc=0x1200, d=0xf81b, x=0, y=65535, sp=0x21f4, ccr=0xb8,
          **{'irq-entries': 1, 'irq-returns': 1}, sent=1360, received=1360, loaded=341)])
    equal(case, 'ENTRY', [dict(pc=0x1200, d=0xf81b, sp=0x21f4, chunks=1360, samples=1360, loaded=341)])
    equal(case, 'SOURCE', [dict(index=n, address=0xef00+n, value=value)
                           for n, value in enumerate(words[:342])])
    equal(case, 'CHUNK', [dict(index=n, value=value, pc=CHUNK_PCS[n % 4])
                          for n, value in enumerate(chunks[:1364])])
    equal(case, 'CONSUMER', [dict(index=n, value=value) for n, value in enumerate(chunks[:1360])])
    equal(case, 'HELPER_SAMPLE', [dict(index=n, value=chunks[1360+n] | (128 if n % 2 == 0 else 0))
                                  for n in range(4)])
    equal(case, 'BYTE', [dict(index=n, value=(words[340] >> shift) & 255,
                             sp=0x21e5 if n < 2 else 0x21e7, x=0x21ec if n < 2 else 0x21ed)
                         for n, shift in enumerate((16, 8, 0))])
    prefix = case['rows'].get('PREFIX', [])
    require(len(prefix) == 1 and prefix[0]['steps'] > 0, 'Missing native copy/checksum step count')
    equal(case, 'PREFIX', [dict(mcu=0x121a, sp=0x21e7, sent=1364, loaded=342, steps=prefix[0]['steps'])])


def reset(case):
    rows = case['rows']
    assertion = rows.get('ASSERT', [])
    release = [row for row in rows.get('PIN', []) if row['pc'] == 0x13a3 and row['address'] == 9]
    require(len(assertion) == 1 and assertion[0]['pc'] == 0x1397 and len(release) == 1,
            'Reset lacks original native direction writes')
    phase = case['case']['target-phase']
    def first_after(ticks):
        return phase + ((ticks-phase)//15+1)*15
    first = first_after(assertion[0]['ticks'])
    expected = []
    for index in range(4):
        tick = first + ((34, 72)[index]-1)*15 if index < 2 else (
            first_after(release[0]['ticks']) + (6*15 if index == 3 else 0))
        expected.append(dict(ticks=tick, edges=(tick-first)//15+1, phase=index+1,
                             mode=2 if index < 2 else 1))
    equal(case, 'RESET', expected)
    equal(case, 'TARGET', [dict(pc=0xa5a5, hx=0x22, sp=255, a=0xa5,
                               ccr=0x68, known=0x68, status=0xc8)])
    equal(case, 'SAVING_BOUNDARY', [dict(pc=0xa6ee, sp=0x21e0, rx=1, commands=0)])
    serial = rows.get('SYNC', [])
    require([(r['phase'], r['count']) for r in serial] ==
            [(1, 0), (3, 0), (0, 1), (1, 1), (3, 1), (0, 2), (1, 2)],
            'Missing initial SYNCs or reference-model LCD long-low recognition')
    for index in (1, 4):
        low, high = serial[index:index+2]
        require(high['cycle']-low['cycle'] == 129 and high['ticks']-low['ticks'] == 129*15,
                'Changed independent SYNC response timing')
    received = rows.get('RX', [])
    require(len(received) == 1 and received[0]['value'] == 0x90, 'Invented trim/upload command')


def drawing(case):
    equal(case, 'TEXT', [dict(mode=mode, **{'return':returned}, pointer=0x15d0, sp=sp,
                             x=8, y=16, font=2)
                         for mode, returned, sp in ((0, 0x124b, 0x21e0), (3, 0xa73c, 0x21d3))])
    equal(case, 'TEXT_RETURN', [dict(sp=0x21e2)])


def frontier(case):
    values = case['rows'].get('WIRE_FRONTIER', [])
    require(len(values) == 1, 'Missing reference-model refusal')
    count = values[0]['host-steps']
    require(count > case['rows']['PREFIX'][0]['steps'], 'No native reset/drawing execution')
    equal(case, 'WIRE_FRONTIER', [dict(pc=0x143f, rx=1, commands=1, syncs=2, writes=1172, reads=296,
          **{'host-steps':count, 'target-retired':0}, uploaded=0, buffered=0)])
    diagnostic = case['rows'].get('DIAGNOSTIC', [])
    require(len(diagnostic) == 1 and all(token in diagnostic[0] for token in
            ('HC12 cycle execution pc=0x143f', 'MCU write at 0x0001', 'BDC-cycles=',
             'host falling edge before the preceding bit/SYNC finished')),
            'Missing actual operation/address/native context diagnostic')
    retained = case['rows'].get('RETAINED', [])
    require(len(retained) == 1 and retained[0]['omitted-mcu'] == count+1
            and retained[0]['omitted-dsp'] > 0 and retained[0]['omitted-known-pads'] > 0,
            'Native trace compression lost execution accounting')
    equal(case, 'RETAINED', [dict(nvm=1, helper=1, source=1, unknown=0, transfers=0,
          **{'board-rejected':1, 'omitted-mcu':count+1,
             'omitted-dsp':retained[0]['omitted-dsp'],
             'omitted-known-pads':retained[0]['omitted-known-pads']})])
    require(len(case['rows'].get('NET_UNQUALIFIED', [])) == 1,
            'Declared fixture must retain its unresolved board interval')
    replays = case['rows'].get('REPLAY', [])
    require(len(replays) == 2 and all(row == dict(index=n, bytes=row['bytes'], identical=1)
            and row['bytes'] > 0 for n, row in enumerate(replays)), 'Incomplete fresh joint replays')
    equal(case, 'CASE_END', [dict(**case['case'], **{'host-steps':count, 'target-retired':0}, uploaded=0)])


def validate(text, expected):
    cases = collect(text)
    for case in cases:
        downloads(case, expected)
        handoff(case, expected)
        reset(case)
        drawing(case)
        frontier(case)
    return dict(cases=2, fresh_joint_suffix_replays=4, native_copied_helper_bytes=2036,
                source_prefetches=684, source_output_groups=2728, resident_samples=2720,
                checksum_bytes=6, actual_target_resets=2, target_retired=0, RAM_uploads=0,
                native_HC12_steps=sum(case['rows']['WIRE_FRONTIER'][0]['host-steps'] for case in cases))


def controls(text, expected):
    changes = []
    for tag, key in (('SOURCE', 'value'), ('CHUNK', 'value'), ('COPY_BOUNDARY', 'ccr'),
                     ('TARGET', 'known'), ('TEXT', 'x'), ('TEXT', 'mode'),
                     ('WIRE_FRONTIER', 'uploaded'), ('REPLAY', 'identical')):
        changes.append(re.sub(r'(^'+tag+r' [^\n]*\b'+key+r'=)(\d+)',
                              lambda match:match[1]+str(int(match[2])+1), text, count=1, flags=re.M))
    for tag in ('PREFIX', 'ASSERT', 'RESET', 'SAVING_BOUNDARY', 'TEXT_RETURN',
                'DIAGNOSTIC', 'CASE_END', 'END'):
        changes.append(re.sub(r'^'+tag+r' [^\n]*\n', '', text, count=1, flags=re.M))
    changes.extend((text+'UNKNOWN after=1\n', text[:-1]))
    for changed in changes:
        require(changed != text, 'Directed chain collector mutation was not applied')
        try:
            validate(changed, expected)
        except (ValueError, KeyError):
            continue
        raise ValueError('Chain collector accepted a fixed own-trace corruption')
    return len(changes)
