# SPDX-License-Identifier: GPL-3.0-only
"""Independent fixed original move clock expectations; reference bytes stay local."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    digest = hashlib.sha256(bytes(image[n] for n in range(0xe252, 0xe255))).hexdigest()
    if digest != '6f1bf1f71c43ad390fcaf82fa22568fa808b0220c63898b642ab38dad5e36481':
        raise ValueError('Changed original move instruction identity')
    cpu_cuts = {0, 11, 18, 26, 29, 37, 42, 44, 45, 46, 47, 49, 50, 55, 56, 59, 60, 65, 66,
                67, 70, 71, 77, 85, 88, 96, 102, 103, 104, 105, 109, 114, 115, 118, 119, 120, 123}
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]
    periods = contexts = cut_count = replays = 0

    def endpoint(seed, order, delay, pads):
        moved = pads >= 0
        flags = 0x62 if pads in (-1, 0) else (0x64 if pads & 0x80 else 0x60)
        return ('END', dict(seed=seed, order=order, delay=delay, pads=pads,
            periods=176 if moved else 171, pc=0xe255 if moved else 0xe252, sp=0x475, ccr=flags,
            retired=46 if moved else 45, reads=116 if moved else 113, data=46 if moved else 45,
            events=6 if moved else 5, edges=16 if moved else 15,
            active=133 - 2 * delay if moved else 127 - delay,
            remaining=3 + 2 * delay if moved else delay + 1,
            transfers=int(moved), launches=2 if moved else 1, tx=0x90 if moved else 0x48,
            samples=8 if moved else 7, adc_remaining=0 if moved else 5, adc_conversions=int(moved)))

    def missing(order, delay):
        return ('MISSING', dict(order=order, delay=delay, attempted=172, completed=171,
                                spi_periods=171 + int(order == 2), adc_remaining=4, edges=15))

    for order in (1, 2):
        for delay in (1, 2):
            for pads in (-1, 0, 0x55, 0x80, 0xff):
                end = 171 if pads < 0 else 176
                for seed in range(8):
                    rows.append(endpoint(seed, order, delay, pads))
                    if pads < 0:
                        rows.append(missing(order, delay))
                    periods += end
                    contexts += 1
                cuts = cpu_cuts | set(range(124, end + 1))
                for edge in range(1, 16 if pads < 0 else 17):
                    period = 45 + delay + edge * 8 - 1
                    cuts.update((period - 1, period))
                for cut in sorted(cuts):
                    periods += cut
                    cut_count += 1
                    for replay in range(2):
                        rows.extend((('REPLAY', dict(order=order, delay=delay, pads=pads,
                                                    cut=cut, index=replay)),
                                     endpoint(7, order, delay, pads)))
                        if pads < 0:
                            rows.append(missing(order, delay))
                        periods += end - cut
                        replays += 1
            periods += 171
            rows.extend((('DETACHED', dict(order=order, delay=delay)), missing(order, delay)))
    rows.append(('SUMMARY', dict(periods=periods, contexts=contexts, cuts=cut_count,
                                 replays=replays, detached=4)))
    return rows
