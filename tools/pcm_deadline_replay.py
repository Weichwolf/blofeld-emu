#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Replay accepted WAV frame arrivals against a 44.1 kHz software ring."""

import argparse
import csv
from pathlib import Path
import wave


RATE = 44100
PERIOD = 64
CAPACITY = 3 * PERIOD
NS_PER_SECOND = 1_000_000_000


def arrivals(trace, total):
    blocks = []
    clock = written = source = 0
    with trace.open(newline="") as input_file:
        for row in csv.DictReader(input_file, delimiter="\t"):
            now = int(row["monotonic_ns"])
            count = int(row["frames"])
            after = int(row["written_frames"])
            if (now < clock or int(row["first_frame"]) != source or
                    not 1 <= count <= 16 or not 0 <= after - written <= count or
                    after > total):
                raise ValueError("PCM trace is discontinuous or disagrees with the WAV")
            if after > written:
                blocks.append((now, after))
            elif 0 < written < total:
                raise ValueError("PCM trace has a gap within the written frames")
            clock, written, source = now, after, source + count
    if written != total or total < CAPACITY:
        raise ValueError("PCM trace cannot fill the ring for this WAV")
    return blocks


def elapsed(frames):
    return frames * NS_PER_SECOND // RATE


def replay(blocks):
    start = next(now for now, frames in blocks if frames >= CAPACITY)

    def available(frames, ready):
        if frames <= CAPACITY:
            return ready
        # A block can straddle a playback deadline. Only its prefix needs to
        # fit for that playback period, but the producer waits for the suffix.
        released = (frames - CAPACITY - 1) // PERIOD * PERIOD
        return max(ready, start + elapsed(released))

    previous = accepted = blocks[0][0]
    period_end = PERIOD
    for now, frames in blocks:
        ready = accepted + now - previous
        while period_end <= frames:
            deadline = start + elapsed(period_end - PERIOD)
            yield deadline - available(period_end, ready), period_end
            period_end += PERIOD
        accepted = available(frames, ready)
        previous = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wav", type=Path)
    arguments = parser.parse_args()
    try:
        with wave.open(str(arguments.wav)) as audio:
            if audio.getframerate() != RATE or audio.getnchannels() != 2:
                raise ValueError("Expected a stereo 44.1 kHz WAV")
            total = audio.getnframes()
        trace = Path(str(arguments.wav) + ".timing.tsv")
        slack = list(replay(arrivals(trace, total)))
    except (KeyError, OSError, ValueError, wave.Error) as error:
        parser.error(str(error))
    worst, frame = min(slack)
    late = sum(value < 0 for value, _ in slack)
    print(f"{RATE} Hz, {PERIOD}-frame period, {CAPACITY}-frame ring, "
          f"{late}/{len(slack)} late, minimum slack {worst / 1e6:+.3f} ms "
          f"at WAV frame {frame}, {total % PERIOD} trailing frames")


if __name__ == "__main__":
    main()
