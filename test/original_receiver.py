# SPDX-License-Identifier: GPL-3.0-only
"""Fixed ISA/source-derived original receive/return reference; no vendor program bytes."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    digest = hashlib.sha256(bytes(image[n] for n in range(0xea29, 0xea42))).hexdigest()
    if digest != 'f437981e8bfa0b684b1c42c79f82b629532a3bf306a8ea27348831d75d9e18fa':
        raise ValueError('Changed original RX routine identity')
    if image[0xe258] != 0x3f:
        raise ValueError('Changed original strict CLR frontier')
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]

    def endpoint(order, delay, pads):
        return [('END', dict(order=order, delay=delay, pads=pads, periods=268, pc=0xe258,
                             retired=72, reads=177, data=70, edges=27, rx=0x12, rx_write=1)),
                ('FRONTIER', dict(attempted=269, completed=268, pc=0xe258))]

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (0, 0x55, 0x80, 0xff):
                rows.extend(endpoint(order, delay, pads))
            rows.append(('OLD', dict(order=order, delay=delay, attempted=213, completed=212,
                                     pc=0xea39)))
            for cut in range(176, 269):
                for index in range(2):
                    rows.append(('REPLAY', dict(order=order, delay=delay, cut=cut, index=index)))
                    rows.extend(endpoint(order, delay, 0x80))
    rows.append(('SUMMARY', dict(contexts=16, cuts=372, replays=744, old=4)))
    return rows
