# SPDX-License-Identifier: GPL-3.0-only
"""Independent byte, wire-order and operation-deadline checks for fixed sample blocks."""

import re


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def wire_expectations(frames, reference):
    expected = []
    rows = []
    group_words = 5640
    for group in range(reference["blocks"]):
        offset = group * (4 + group_words * 3)
        slot = int.from_bytes(frames[offset:offset + 4], "big")
        payload = frames[offset + 4:offset + 4 + 16896]
        row = int.from_bytes(frames[offset + 4 + 16896:offset + 4 + 16899], "big")
        require(slot == reference["staging_addresses"][group], "Changed fixed staging slot")
        rows.append(row)
        expected.extend([(0, 0x60)] + [(1, (row >> shift) & 255) for shift in (0, 8, 16)]
                        + [(0, 0xd0), (0, 0x70)])
        for page in range(32):
            expected.extend([(0, 0), (0, 0x80), (1, 0)]
                            + [(1, ((row + page) >> shift) & 255) for shift in (0, 8, 16)])
            expected.extend((2, value) for value in payload[page * 528:(page + 1) * 528])
            expected.extend([(0, 0x10), (0, 0x70)])
    require(rows == reference["rows"], "Changed fixed rows or intervening gap")
    return expected


def deadlines(trace, reference):
    pending = None
    counts = {"erase": 0, "program": 0}
    for line in trace.splitlines():
        match = re.match(r"NAND_LATCH ticks=(\d+).*kind=0 value=(0x[a-f0-9]+)", line)
        if match and int(match[2], 16) in (0xd0, 0x10):
            require(pending is None, "Overlapping write confirmations")
            pending = (int(match[1]), "erase" if int(match[2], 16) == 0xd0 else "program")
        match = re.match(r"NAND_AUTONOMOUS_COMPLETION ticks=(\d+) operation=(\d+)", line)
        if match:
            require(pending is not None, "Unconfirmed operation completion")
            start, kind = pending
            require(int(match[1]) - start == reference["operation_ticks"][kind],
                    "Changed independently scheduled operation deadline")
            require(int(match[2]) == (4 if kind == "erase" else 3), "Changed completion kind")
            counts[kind] += 1
            pending = None
    require(pending is None and counts == {"erase": 4, "program": 128},
            "Incomplete erase/program lifecycle")


def validate(output, traces, frames, reference):
    pattern = (r"^COMPLETE blocks=(\d+) cycles=(\d+) instructions=(\d+) "
               r"timer-entries=(\d+) inhibited=([01])$")
    results = [tuple(map(int, row)) for row in re.findall(pattern, output, re.M)]
    expected_result = (reference["blocks"], reference["cycles"], reference["instructions"],
                       reference["timer_entries"])
    require(results == [(*expected_result, 0), (*expected_result, 1)],
            "Missing normal/inhibited native completion and original foreground return")
    blocks = [int(value) for value in re.findall(r"^BLOCK count=(\d+) ", output, re.M)]
    require(blocks == [1, 2, 3, 4] * 2, "Changed original block completion order")
    expected = wire_expectations(frames, reference)
    for inhibited, trace in enumerate(traces):
        actual = [(int(kind), int(value, 16)) for kind, value in re.findall(
            r"NAND_LATCH .*?kind=(\d+) value=0x([a-f0-9]+)", trace)]
        require(actual == expected, "Actual command/address/data latches differ from input bytes")
        statuses = re.findall(r"NAND_BYTE .*?value=(0x[a-f0-9]+)", trace)
        expected_status = (["0xc0"] * 262 + ["0xc1"] * 2 if inhibited else ["0xc0"] * 264)
        require(statuses == expected_status, "Actual status reads lost the failed final program")
        deadlines(trace, reference)
    require(traces[1].replace("value=0xc1 next-column=528", "value=0xc0 next-column=528")
            == traces[0], "Cell inhibition changed more than the two final status reads")
    return {"runs": 2, "blocks_per_run": reference["blocks"], "pages_per_run": 128,
            "cell_comparisons_per_run": 67584, "guard_bytes_per_run": 50688,
            "wire_latches_per_run": len(expected), "timer_entries_per_run": 132,
            "native_instructions": 2 * reference["instructions"],
            "negative": "One wrong programmed bit; actual C1 with original block counter4"}
