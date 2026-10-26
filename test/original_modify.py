# SPDX-License-Identifier: GPL-3.0-only
"""Independent original INC/branch/foreground reference; no manufacturer program bytes."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    prefix = hashlib.sha256(bytes(image[n] for n in range(0xe258, 0xe262))).hexdigest()
    target = hashlib.sha256(bytes(image[n] for n in range(0xe29f, 0xe2a4))).hexdigest()
    if prefix != 'f3297b6d7fadf0e72f4b5362a3c907057043f674bd9cfad1bd0d883404217bda' or target != '9df976caa1d464dae37fe71accc937c9315db80ab65efe0df9455cb1735ccbcf':
        raise ValueError('Changed original INC/dependent branch/JSR identity')
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]

    def endpoint(order, delay, pads):
        return [('MODIFY', dict(order=order, delay=delay, pads=pads, periods=288, pc=0xe2a1,
                                retired=78, reads=190, data=75, edges=30, expiry=8, transmitted=0x481e)),
                ('FRONTIER', dict(attempted=289, completed=288, pc=0xe2a1))]

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (0, 0x55, 0x80, 0xff):
                rows.append(('END', dict(order=order, delay=delay, pads=pads, periods=268, pc=0xe258,
                    retired=72, reads=177, data=70, edges=27, rx=0x12, rx_write=1)))
                rows.extend(endpoint(order, delay, pads))
            rows.append(('OLD', dict(order=order, delay=delay, attempted=274, completed=273, pc=0xe25a)))
            for cut in range(273, 289):
                for index in range(2):
                    rows.append(('REPLAY', dict(order=order, delay=delay, cut=cut, index=index)))
                    rows.extend(endpoint(order, delay, 0x80))
    rows.append(('SUMMARY', dict(contexts=16, cuts=64, replays=128, old=4)))
    return rows
