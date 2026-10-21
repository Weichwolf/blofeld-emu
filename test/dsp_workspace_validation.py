# SPDX-License-Identifier: GPL-3.0-only
"""Compare original workspace traces to independent caller equations."""

import hashlib
import re

from dsp_workspace_model import ALLOCATE, REMOVE, VOICE, OUTPUTS, MASK, W, model


def state(row, pc, sp, returned, regs, r, n, mode, label):
    expected = [pc, sp, returned, *regs]
    if len(row) != 34 or any(row[i] != value for i, value in enumerate(expected)
                             if value is not None):
        raise ValueError(f"{label}: accumulator/frame differs: {row[:7]} != {expected}")
    if (row[8:16] != r or row[16:24] != n or row[24:32] != [MASK] * 8
            or row[7] & 0x200c00 != mode):
        raise ValueError(f"{label}: address/mode state differs: {row[8:24]} != {r+n}")


def rows(block, label):
    return [[int(value, 16) for value in text.split()]
            for text in re.findall(r"^" + label + r" ([^\n]+)$", block, re.M)]


def single(block, label):
    result = rows(block, label)
    if len(result) != 1:
        raise ValueError(f"Missing or duplicate {label} row")
    return result[0]


def check_case(block, case, returned, metrics):
    cid, entry, mode, request, selected, x0, *_ = case
    state(single(block, "ENTRY"), entry, 1, returned,
          [0, request * W, (0x468ace << 24) | x0, 0x3579bd2468ac],
          [0x936, entry, VOICE, 0x700, 0x800, selected, 0, 0],
          [0x53, 0, 0, 0, 0, 0, 0, 0], mode, f"case {cid} entry")
    memory, regs, r, n, shifts, resets, victims = model(case)
    state(single(block, "END"), returned, 0, None, regs, r, n, mode, f"case {cid} return")
    if single(block, "MEMORY") != [memory[at] for at in OUTPUTS]:
        raise ValueError(f"case {cid}: memory differs")
    observed = rows(block, "SHIFT")
    if (any(len(row) != 34 for row in observed)
            or [(row[10], row[12]) for row in observed] != shifts
            or any(row[0] != 0x2bb5 or row[1] != 1 or row[13] != selected
                   for row in observed)):
        raise ValueError(f"case {cid}: genuine victim/shift frames differ")
    observed = rows(block, "RESET")
    if (any(len(row) != 34 for row in observed)
            or [row[12] for row in observed] != resets
            or any(row[0] != 0x2bca or row[1] != 1 or row[23] != request
                   for row in observed)):
        raise ValueError(f"case {cid}: original request-mask fallback frame differs")
    if entry == ALLOCATE:
        metrics["allocations"] += 1
        metrics["evicted_voices"] += len(victims)
        metrics["request_mask_fallbacks"] += len(resets)
        metrics["shifted_list_words"] += len(shifts)
    elif entry == REMOVE:
        metrics["list_removals" if n[1] == 2 else "list_misses"] += 1
    else:
        metrics["complete_cleanups"] += 1
    counts = re.findall(r"^COUNTS (\d+) (\d+)$", block, re.M)
    if (len(counts) != 1 or not 0 < int(counts[0][0]) < 20000
            or int(counts[0][1]) not in (1, 3)):
        raise ValueError("Missing workspace instruction counts/real return")
    metrics["native_instructions"] += int(counts[0][0])


def validate(text, cases, reference):
    blocks = re.findall(r"^CASE ([0-9a-f]+)\n([\s\S]*?)(?=^CASE |^CALLER_RETURN)", text, re.M)
    if [int(i, 16) for i, _ in blocks] != list(range(len(cases))):
        raise ValueError("Missing or reordered workspace cases")
    returned = single(text, "CALLER_RETURN")
    if len(returned) != 1 or not 0xff0000 <= returned[0] < 0xff1000:
        raise ValueError("Missing own native caller return")
    metrics = {key: 0 for key in reference["expected_metrics"]}
    for (_, block), case in zip(blocks, cases):
        check_case(block, case, returned[0], metrics)
    if metrics != reference["expected_metrics"]:
        raise ValueError(f"Incomplete workspace boundary coverage: {metrics}")
    if re.findall(r"^TOTAL (\d+)$", text, re.M) != [str(len(cases))]:
        raise ValueError("Incomplete workspace totals")
    visited = single(text, "VISITED")
    identity = hashlib.sha256("".join(f"{pc:06x}\n" for pc in visited).encode()).hexdigest()
    coverage = reference["coverage"]
    if (visited != sorted(set(visited)) or len(visited) != coverage["instruction_starts"]
            or identity != coverage["sha256"]):
        raise ValueError("Original module instruction coverage differs from static listing")
    if re.findall(r"^SNAPSHOTS (\d+) (\d+)$", text, re.M) != [("2", "4")]:
        raise ValueError("Missing joint snapshot replay")
    if rows(text, "REPLAY") != [[324, 0x2bca], [326, 0x2bb5]]:
        raise ValueError("Wrong native fallback/eviction snapshot boundaries")
    return {**metrics, "instruction_starts": len(visited), "snapshots": 2, "replays": 4}
