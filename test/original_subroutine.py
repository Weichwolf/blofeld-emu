# SPDX-License-Identifier: GPL-3.0-only
"""Independent source/ISA-derived original delay/IRQ/return/GPIO reference."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    if (hashlib.sha256(bytes(image[n] for n in range(0xea9e, 0xeaa4))).hexdigest() != BODY_HASH
            or hashlib.sha256(bytes(image[n] for n in range(0xe2a4, 0xe2a7))).hexdigest() != GPIO_HASH):
        raise ValueError('Changed original delay body or GPIO sample instruction')
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]

    def endpoint(order, delay, pads):
        active = 415 - (299 + 3*delay) + 1
        payload_bits = (active // 8) // 2
        return ('DELAY', dict(order=order, delay=delay, pads=pads, periods=415, pc=0xe2a7,
                              sp=0x475, retired=124, reads=283, data=101, irq=4, irq_periods=44,
                              launches=3, edges=32+active//8, samples=16+payload_bits,
                              tx=(0x903c << payload_bits) | (0x64 >> (8-payload_bits)),
                              active=256+active, remaining=8-active % 8, gpio=pads,
                              ccr=0x61 | (2 if pads == 0 else 0) | (4 if pads & 0x80 else 0)))

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (0, 0x55, 0x80, 0xff):
                rows.append(endpoint(order, delay, pads))
            rows.append(('OLD', dict(order=order, delay=delay, attempted=296, completed=295, pc=0xea9f)))
            irq = 299 + 3*delay + int(order == 1)
            cuts = [294, irq-1, irq, irq+4, irq+6, irq+10, irq+64, irq+70, irq+73, *range(405, 416)]
            assert len(cuts) == len(set(cuts)) == 20
            for cut in cuts:
                for index in range(2):
                    rows.append(('REPLAY', dict(order=order, delay=delay, cut=cut, index=index)))
                    rows.append(endpoint(order, delay, 0x80))
    rows.append(('SUMMARY', dict(contexts=16, cuts=80, replays=160, old=4)))
    return rows


BODY_HASH = 'ae418862d35d5c71b1a7539104e03c1289c575e343e7cad6a6dba6895d749ab2'
GPIO_HASH = 'ca71639e1b2fab9f664461efb3a6b95a25de4103fe38bc018a3d93ac92e778f2'
