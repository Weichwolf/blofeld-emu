#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Align a JACK PCM underfill with DSP-owner dispatch and wakeup records.

Enable BLOFELD_JACK_DEADLINE_TRACE in a diagnostic CMake build and set
BLOFELD_JACK_TRACE to a new output path for a live session. The application
writes its first underfill and recent publications on exit. An optional
perf capture uses --switch-events, --clockid mono and the DSP writer TID.
Supply wakeups only when they were captured on the same clock for that TID.
Scheduler time before the PCM read is reported separately from time until
the missing frames were published.
"""

import argparse
from bisect import bisect_left, bisect_right
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import re
import subprocess

SECOND_NS = 1_000_000_000
SWITCH = re.compile(
    r"\s*\S+\s+(\d+)\s+\[[^\]]+\]\s+(\d+)\.(\d{9}): PERF_RECORD_SWITCH (IN|OUT)(.*)"
)


@dataclass(frozen=True)
class Publication:
    head: int
    ns: int
    cpu_ns: int


def capture(path):
    with path.open() as stream:
        header = next(stream, "").split()
        if len(header) != 4 or header[:2] != ["JACK-DEADLINE", "1"]:
            raise ValueError("Unrecognized JACK deadline capture")
        tid, xruns = map(int, header[2:])
        miss, records = None, []
        for line in stream:
            kind, *columns = line.split()
            values = tuple(map(int, columns))
            if kind == "C" and len(values) == 6 and miss is None:
                miss = values
            elif kind == "P" and len(values) == 3:
                records.append(Publication(*values))
            else:
                raise ValueError("Malformed JACK deadline record")
    for left, right in zip(records, records[1:]):
        if right.head <= left.head or right.ns < left.ns or right.cpu_ns < left.cpu_ns:
            raise ValueError("DSP publication sequence moved backwards")
    return tid, xruns, miss, records


def switches(path, tid):
    events = subprocess.run(
        ["perf", "script", "--ns", "--show-switch-events", "--show-lost-events", "-i", path],
        capture_output=True, text=True, check=True, timeout=30
    )
    if "PERF_RECORD_LOST" in events.stdout + events.stderr:
        raise ValueError("DSP scheduler capture lost records")
    for line in events.stdout.splitlines():
        match = SWITCH.fullmatch(line)
        if match is not None and int(match[1]) == tid:
            seconds, nanoseconds = int(match[2]), int(match[3])
            yield (seconds * SECOND_NS + nanoseconds, match[4],
                   "preempt" in match[5])


def time_by_state(events, wakeups, start, end):
    times = Counter()
    for (left, state, preempted), (right, resume, _) in zip(events, events[1:]):
        begin, stop = max(left, start), min(right, end)
        if begin >= stop:
            continue
        if state == resume:
            times["unknown"] += stop - begin
        elif state == "IN":
            times["running"] += stop - begin
        elif preempted:
            times["runnable"] += stop - begin
        else:
            wake = bisect_left(wakeups, left)
            if wake < len(wakeups) and wakeups[wake] < right:
                split = max(begin, min(stop, wakeups[wake]))
                times["blocked"] += split - begin
                times["runnable"] += stop - split
            else:
                times["off_cpu_unattributed"] += stop - begin
    uncovered = end - start - sum(times.values())
    if uncovered < 0:
        raise ValueError("Invalid or unordered scheduler timestamps")
    times["unknown"] += uncovered
    return dict(times)


def analyze(args):
    tid, xruns, miss, records = capture(args.capture)
    if miss is None:
        return {"pcm_underfill": False, "jack_xruns": xruns}
    frame, head, required, before, after, missing = miss
    heads = [item.head for item in records]
    end_index = bisect_left(heads, required)
    begin_index = min(bisect_right(heads, head),
                      bisect_right([item.ns for item in records], before)) - 1
    if not 0 <= begin_index < end_index < len(records):
        raise ValueError("DSP capture does not bracket the completed PCM publication")
    begin, done = records[begin_index], records[end_index]
    if before > after or done.ns < before:
        raise ValueError("PCM and JACK timestamps don't bracket the underfill")
    available_upper_bound = after - begin.ns
    cpu_ns = done.cpu_ns - begin.cpu_ns
    wakeups = sorted(map(int, args.wakeups.read_text().split())) if args.wakeups else []
    wall_end = done.ns
    events = list(switches(args.switches, tid)) if args.switches else []
    return {
        "pcm_underfill": True,
        "jack_xruns": xruns,
        "jack_frame": frame,
        "missing_frames": missing,
        "queued_frame_debt": required - head,
        "available_ns": before - begin.ns,
        "available_upper_bound_ns": available_upper_bound,
        "publication_observed_late_ns": done.ns - before,
        "dsp_cpu_ns": cpu_ns,
        "dsp_cpu_overrun_lower_bound_ns": max(0, cpu_ns - available_upper_bound),
        "measured_wall_ns": wall_end - begin.ns,
        "scheduler_before_read_ns": time_by_state(events, wakeups, begin.ns, before),
        "scheduler_ns": time_by_state(events, wakeups,
                                     begin.ns, wall_end),
    }


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("capture", type=Path)
    args.add_argument("--switches", type=Path, help="perf record file with CLOCK_MONOTONIC")
    args.add_argument("--wakeups", type=Path, help="DSP TID wake timestamps in monotonic ns")
    try:
        print(json.dumps(analyze(args.parse_args()), indent=2))
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        args.error(str(error))


if __name__ == "__main__":
    main()
