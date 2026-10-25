# SPDX-License-Identifier: GPL-3.0-only
"""Independent fixed original word clock expectations; reference bytes stay local."""
import hashlib


def expected_rows(pages):
    image = {}
    for offset in range(0, len(pages), 514):
        start = int.from_bytes(pages[offset:offset + 2], 'big')
        image.update((start + n, value) for n, value in enumerate(pages[offset + 2:offset + 514]))
    digest = hashlib.sha256(bytes(image[n] for n in range(0xe224, 0xe228))).hexdigest()
    if digest != '07204893d687cc704795b5616f8d8240ee3f2b6b56f0ffea94a9fcb99a60127a':
        raise ValueError('Changed original word instruction identity')
    address = image[0xe225]
    cpu_cuts = {0, 11, 18, 26, 29, 37, 42, 44, 45, 46, 47, 49, 50, 55, 56, 59, 60, 65, 66,
                67, 70, 71, 77, 85, 88, 96, 102, 103, 104, 105, 109, 114, 115, 118, 119, 120, 123}
    rows = [('STARTUP', dict(retired=6188, pc=0xe1b0, sp=0x475, ram_known=1024))]
    periods = cut_count = replays = 0

    def endpoint(seed, order, delay):
        return ('WORD', dict(seed=seed, order=order, delay=delay, periods=171, pc=0xe252, sp=0x475,
                a=0, hx=0, ccr=0x62, known=0xef, retired=45, edges=15, active_periods=127-delay,
                remaining=delay+1, tx=0x48, samples=7, shift_in=0x12, queued=0x3c,
                adc_remaining=5, reads=113, data_accesses=45, events=5, word_address=address,
                ready=2, inflight=1, next=0xe255, head=0xe252))

    def frontier(order, delay):
        return ('FRONTIER', dict(order=order, delay=delay, pc=0xe252, attempted=172, completed=171,
                spi_periods=171+int(order == 2), adc_remaining=4, edges=15, pins=0))

    for order in (1, 2):
        for delay in (1, 2):
            for seed in range(8):
                rows.extend((endpoint(seed, order, delay), frontier(order, delay)))
                periods += 171
            cuts = cpu_cuts | set(range(124, 172))
            for edge in range(1, 16):
                period = 45 + delay + edge * 8 - 1
                cuts.update((period - 1, period))
            for cut in sorted(cuts):
                periods += cut
                cut_count += 1
                for replay in range(2):
                    rows.extend((('REPLAY', dict(order=order, delay=delay, cut=cut, index=replay)),
                                 endpoint(7, order, delay), frontier(order, delay)))
                    periods += 171 - cut
                    replays += 1
    rows.append(('SUMMARY', dict(periods=periods, contexts=32, cuts=cut_count, replays=replays)))
    return rows
