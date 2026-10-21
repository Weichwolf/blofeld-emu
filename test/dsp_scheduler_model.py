# SPDX-License-Identifier: GPL-3.0-only
"""Independent scheduler equations adapted from the source checkpoint in dsp-scheduler.txt."""

import re

W, H, MASK = 1 << 24, 1 << 23, (1 << 24) - 1
FULL_MASK = (1 << 56) - 1
ROUNDING_MODE = 1 << 21
TAIL, SERVICE, CALLBACK = 0x22d, 0x28f2, 0x28fe


def signed(value):
    return value - W if value & H else value


def limited(value):
    return min(H - 1, max(-H, value // W)) & MASK


def directed_cases():
    counts = (0, 1, 2, 0x1000, H - 2, H - 1, H, MASK)
    seeds = (0, 1, 2, 3, H - 2, H - 1, H, MASK)
    ticks = (0, 1, 0x7fefff, 0x7ff000, H - 1, 0xffefff, 0xfff000, MASK)
    timers = (0, 1, 0x47ff, 0x4800, 0x7fb800, H - 1, H, MASK)
    thresholds = (0x4800, 0x4801, 0, 1, H - 1, H, MASK, 0x9000)
    cases = []
    for mode in (0, ROUNDING_MODE):
        for armed in (False, True):
            for index, count in enumerate(counts):
                for scenario in range(8):
                    flags = (0x654321 & ~0x280) | (0x200 if armed else 0)
                    cases.append([TAIL, seeds[(scenario + index) % 8], ticks[scenario], timers[scenario],
                                  thresholds[(scenario + index) % 8], count,
                                  MASK if scenario % 2 else 0x123456, flags, mode])
    for mode in (0, ROUNDING_MODE):
        for armed in (False, True):
            for count in counts:
                cases.append([SERVICE, 0x123457, 0x654321, 0x4800, H - 1, count,
                              MASK, 0x200 if armed else 0, mode])
        for count in counts:
            cases.append([CALLBACK, 0x123457, 0x654321, 0x4800, H - 1, count, MASK, 0, mode])
    for index, case in enumerate(cases):
        case.extend(((0, 0x00ffffffe12345, 0xff800000012345)[index % 3],
                     (0, 0x01abcdeffedcba, 0xfe012345654321)[index % 3],
                     (0x123456 + index * 0x10203) & MASK,
                     (0x654321 - index * 0x203) & MASK, (0xabcdef + index * 0x321) & MASK))
    if len(cases) != 304 or len({tuple(c) for c in cases}) != len(cases):
        raise ValueError("Incomplete or duplicated scheduler input matrix")
    return cases


def initial_ram(case):
    _, seed, tick, timer, threshold, count, status, flags, _, *_ = case
    values = [(0x13579b + at * 0x10203 + bias) & MASK
              for bias in (0, 0x456789) for at in range(64)]
    for at, value in ((0x11, seed), (4, tick), (0x1e, timer), (0x1d, threshold),
                      (0x19, count), (7, status), (6, flags), (0x1c, 0x2468ac)):
        values[at] = value
    return values


def prefix_model(case):
    _, seed, tick, timer, threshold, count, status, _, mode, *_ = case
    # Compare the full signed timer sum; limiting it before the comparison is wrong.
    timer_full = (signed(timer) + 0x4800) * W
    crossed = timer_full >= signed(threshold) * W
    if crossed:
        timer_full -= signed(threshold) * W
    countdown = max(signed(count) - 1, 0)
    ram = initial_ram(case)
    ram[0x11] = (seed >> 1) ^ (0x840000 if seed & 1 else 0)
    ram[4] = (tick + 0x1000) & MASK  # Original raw B1 transfer wraps.
    ram[0x1e] = limited(timer_full)
    ram[0x19] = countdown
    ram[7] = status & 0x7fffff if crossed else status
    ram[0x1c] = H if crossed else 0  # Parallel store takes B BEFORE the TFR.
    registers = [timer_full & FULL_MASK, countdown * W, (mode << 24) | count,
                 (0x840000 << 24) | threshold]
    return ram, registers, crossed, timer_full


def check_state(values, expected, case, label):
    pc, sp, ret, registers, r0 = expected
    target, mode = case[0], case[8]
    if len(values) != 16:
        raise ValueError("Malformed scheduler state: " + label)
    wanted = [pc, sp, ret, *registers]
    if any(values[i] != value for i, value in enumerate(wanted) if value is not None):
        raise ValueError(f"Scheduler {label} state differs: {values[:7]} != {wanted}")
    # Empty-stack SSH contents after a genuine RTS are not an asserted return frame.
    if values[8:] != [r0, target, 0xc500, 0x700, 0x800, 0xc500, 0, 0]:
        raise ValueError("Scheduler address registers differ: " + label)
    if values[7] & (ROUNDING_MODE | 0xc00) != mode:
        raise ValueError("Scheduler rounding/scaling domain changed")


def validate(result, cases, caller_return):
    blocks = re.findall(r"^CASE ([0-9a-f]+)\n([\s\S]*?)(?=^CASE |^VISITED)", result, re.M)
    if [int(index, 16) for index, _ in blocks] != list(range(len(cases))):
        raise ValueError("Missing, duplicated or reordered scheduler cases")
    metrics = {name: 0 for name in ("timer_crossings", "timer_no_crossing", "timer_limits",
               "tick_wraps", "integrated_event_calls", "integrated_rearms", "integrated_queued_requests",
               "direct_service_returns", "direct_queued_requests", "continuation_handoffs",
               "clamped_countdown_without_call")}
    total_steps = 0
    for (_, block), case in zip(blocks, cases):
        rows = {label: [int(v, 16) for v in values.split()] for label, values in
                re.findall(r"^(ENTRY|PREFIX|PREFIX_RAM|CALL|END|END_RAM) ([^\n]+)", block, re.M)}
        target, _, tick, _, _, count, _, flags, mode, initial_a, initial_b, x0, y0, y1 = case
        ram = initial_ram(case)
        registers = [initial_a, initial_b, (mode << 24) | x0, (y1 << 24) | y0]
        check_state(rows["ENTRY"], (target, 1, caller_return, registers, 0x1000), case, "entry")
        r0 = 0x1000
        if target == TAIL:
            ram, registers, crossed, timer_full = prefix_model(case)
            metrics["timer_crossings" if crossed else "timer_no_crossing"] += 1
            metrics["timer_limits"] += timer_full >= H * W or timer_full < -H * W
            metrics["tick_wraps"] += tick + 0x1000 >= W
            if rows["PREFIX_RAM"] != ram:
                raise ValueError("Scheduler prefix RAM/guards differ")
            check_state(rows["PREFIX"], (0x242, 1, caller_return, registers, r0), case, "prefix")
            # IFMI never updates CCR: a clamped result0 is not a fresh zero edge.
            if bool(rows["PREFIX"][7] & 4) != (count == 1):
                raise ValueError("Original SUB/IFMI zero flag differs")
            event = count == 1
            check_state(rows["CALL"], (SERVICE if event else 0x244, 2 if event else 1,
                        0x244 if event else caller_return, registers, r0), case, "conditional call")
            if rows["CALL"][7] != rows["PREFIX"][7]:
                raise ValueError("Conditional call changed the incoming status flags")
            metrics["integrated_event_calls"] += event
            metrics["clamped_countdown_without_call"] += signed(count) <= 0 and not event
            if event and flags & 0x200:
                ram[0x19] |= 0x1000
                metrics["integrated_rearms"] += 1
            if event and not flags & 0x200:
                r0, registers[3] = CALLBACK, (0x11e0 << 24) | 0x1c00
                end = (0x1853, 2, 0x244, registers, r0)
                metrics["integrated_queued_requests"] += 1
            else:
                end = (0x198, 1, caller_return, registers, r0)
        elif target == SERVICE:
            if flags & 0x200:
                ram[0x19] |= 0x1000
                end = (caller_return, 0, None, registers, r0)
                metrics["direct_service_returns"] += 1
            else:
                r0, registers[3] = CALLBACK, (0x11e0 << 24) | 0x1c00
                end = (0x1853, 1, caller_return, registers, r0)
                metrics["direct_queued_requests"] += 1
        else:
            ram[0x19] = 0
            registers = [0, initial_b, 0x48, (0x11e0 << 24) | 0x200]
            end = (0x1856, 1, caller_return, registers, 0x720)
            metrics["continuation_handoffs"] += 1
        check_state(rows["END"], end, case, "end")
        if rows["END_RAM"] != ram:
            raise ValueError("Scheduler end RAM/guards differ")
        steps = re.findall(r"^STEPS (\d+)$", block, re.M)
        if len(steps) != 1 or not 0 < int(steps[0]) <= 100:
            raise ValueError("Missing bounded scheduler instruction count")
        total_steps += int(steps[0])
    if any(value == 0 for value in metrics.values()):
        raise ValueError("Missing scheduler arithmetic/control corner: " + str(metrics))
    return metrics, total_steps
