# SPDX-License-Identifier: GPL-3.0-only
"""Literal original CLR bus/endpoint reference; manufacturer code stays local."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    identity = hashlib.sha256(bytes(image[n] for n in range(0xe258, 0xe25c))).hexdigest()
    if identity != '021abfaf476180b6c053319a14c0d05dda13174e11123eb688215b3dbc585816':
        raise ValueError('Changed pinned original CLR/INC frontier')
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]

    def endpoint(order, delay, pads):
        return [('CLEAR', dict(order=order, delay=delay, pads=pads, periods=273, pc=0xe25a,
                               retired=73, reads=179, data=72, edges=28, row=0)),
                ('FRONTIER', dict(attempted=274, completed=273, pc=0xe25a))]

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (0, 0x55, 0x80, 0xff):
                rows.append(('END', dict(order=order, delay=delay, pads=pads, periods=268,
                    pc=0xe258, retired=72, reads=177, data=70, edges=27, rx=0x12, rx_write=1)))
                rows.extend(endpoint(order, delay, pads))
            for cut in range(268, 274):
                for index in range(2):
                    rows.append(('REPLAY', dict(order=order, delay=delay, cut=cut, index=index)))
                    rows.extend(endpoint(order, delay, 0x80))
    rows.append(('SUMMARY', dict(contexts=16, cuts=24, replays=48)))
    return rows
