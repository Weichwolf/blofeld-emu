# SPDX-License-Identifier: GPL-3.0-only
"""Pinned original JSR endpoint and literal ISA stack/reference periods."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    call = bytes(image[n] for n in range(0xe2a1, 0xe2a4))
    body = bytes(image[n] for n in range(0xea9e, 0xeaae))
    if (hashlib.sha256(call).hexdigest() != '1e16be8690b6df8d320c9af34af21b7d732d9d056adebbfef7540090bf32338c'
            or hashlib.sha256(body).hexdigest() != '2f556b213754ee0804a06606624f51481fc9e6970b9acc05a1f524e261691f03'):
        raise ValueError('Changed original call/settle-body identity')
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]

    def endpoint(order, delay, pads):
        return ('CALL', dict(order=order, delay=delay, pads=pads, periods=294, pc=0xea9e,
                             sp=0x473, retired=79, reads=194, data=77,
                             low_period=290, high_period=291, return_pc=0xe2a4))

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (0, 0x55, 0x80, 0xff):
                rows.append(endpoint(order, delay, pads))
            rows.append(('OLD', dict(order=order, delay=delay, attempted=289, completed=288, pc=0xe2a1)))
            for cut in range(288, 295):
                for index in range(2):
                    rows.append(('REPLAY', dict(order=order, delay=delay, cut=cut, index=index)))
                    rows.append(endpoint(order, delay, 0x80))
    rows.append(('SUMMARY', dict(contexts=16, cuts=28, replays=56, old=4)))
    return rows
