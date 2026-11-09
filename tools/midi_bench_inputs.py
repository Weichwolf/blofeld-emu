#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Generate deterministic authored SMFs; do not execute firmware benchmarks."""

import argparse
import hashlib
import json
from pathlib import Path
import struct

PPQN = 480
END = 36 * PPQN
RELEASE = 32 * PPQN


def vlq(value):
    if not 0 <= value <= 0x0fffffff:
        raise ValueError("MIDI delta exceeds four-byte VLQ")
    result = [value & 127]
    while value > 127:
        value >>= 7
        result.insert(0, (value & 127) | 128)
    return bytes(result)


def track(events):
    result = bytearray()
    previous = 0
    for tick, message in sorted(events, key=lambda event: event[0]):
        if not previous <= tick <= END:
            raise ValueError("MIDI event is outside the scenario timeline")
        result.extend(vlq(tick - previous))
        result.extend(message)
        previous = tick
    result.extend(vlq(END - previous) + b"\xff\x2f\0")
    return b"MTrk" + struct.pack(">I", len(result)) + result


def message(tick, *data):
    return tick, bytes(data)


def setup(channel, program):
    return [message(0, 0xb0 | channel, 0, 0),
            message(0, 0xb0 | channel, 32, 0),
            message(0, 0xc0 | channel, program)]


def note(events, channel, pitch, start, end, velocity):
    events.append(message(start, 0x90 | channel, pitch, velocity))
    events.append(message(end, 0x80 | channel, pitch, 0))


def single():
    events = setup(0, 0)
    for beat in range(0, 32, 2):
        note(events, 0, 60 + (beat % 12), beat * PPQN, (beat + 1) * PPQN, 96)
    return [events]


def chords():
    events = setup(0, 0)
    for bar in range(8):
        start = bar * 4 * PPQN
        events.append(message(start, 0xb0, 64, 127))
        for offset in (0, 4, 7, 12, 16, 19):
            note(events, 0, 48 + (bar % 4) + offset, start, start + 3 * PPQN, 100)
        events.append(message(start + 4 * PPQN, 0xb0, 64, 0))
    return [events]


def multi():
    tracks = []
    for channel in range(16):
        events = setup(channel, channel)
        for bar in range(8):
            start = bar * 4 * PPQN
            pitch = 48 + channel
            note(events, channel, pitch, start, start + 3 * PPQN, 100)
            if channel < 9:
                note(events, channel, pitch + 12, start + PPQN, start + 3 * PPQN, 88)
        tracks.append(events)
    return tracks


def controls():
    events = setup(0, 0)
    for beat in range(32):
        start = beat * PPQN
        if beat % 4 == 0:
            events.append(message(start, 0xc0, (beat // 4) % 16))
        events.append(message(start, 0xb0, 1, (beat * 17) % 128))
        events.append(message(start, 0xb0, 11, 64 + beat % 64))
        bend = 4096 + (beat % 3) * 4096
        events.append(message(start, 0xe0, bend & 127, bend >> 7))
        note(events, 0, 60 + beat % 12, start, start + PPQN // 2, 96)
    # Universal identity request; the renderer must preserve its SysEx boundary.
    payload = b"\x7e\x7f\x06\x01\xf7"
    events.append((16 * PPQN, b"\xf0" + vlq(len(payload)) + payload))
    return [events]


def sl_samples():
    events = []
    for tick, parameter, value in ((0, 15, 1), (PPQN // 8, 8, 0)):
        payload = bytes((0x3e, 0x13, 0x7f, 0x20, 0,
                         parameter >> 7, parameter & 127, value, 0xf7))
        events.append((tick, b"\xf0" + vlq(len(payload)) + payload))
    pitches = (48, 55, 60, 64, 67, 72, 76, 79)
    for index in range(16):
        start = (2 * index + 1) * PPQN
        note(events, 0, pitches[index % len(pitches)], start, start + PPQN, 96)
    return [events]


def write_case(output, name, tracks, requirements, tempo_change=False):
    conductor = [message(0, 0xff, 0x51, 3, 7, 0xa1, 0x20)]
    if tempo_change:
        conductor.append(message(16 * PPQN, 0xff, 0x51, 3, 6, 0x1a, 0x80))
    encoded = b"MThd" + struct.pack(">IHHH", 6, 1, len(tracks) + 1, PPQN)
    encoded += track(conductor) + b"".join(track(events) for events in tracks)
    path = output / (name + ".mid")
    with path.open("xb") as stream:
        stream.write(encoded)
    return {"file": path.name, "bytes": len(encoded),
            "sha256": hashlib.sha256(encoded).hexdigest(),
            "note_on_events": sum(1 for events in tracks for _, data in events
                                  if data[0] & 0xf0 == 0x90),
            "channels": len(tracks),
            "file_duration_us": 16000000 if tempo_change else 18000000,
            "requirements": requirements}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cases = [
        write_case(args.output, "single", single(), ["Single mode, channel 1"]),
        write_case(args.output, "chords-pedal", chords(), ["Single mode, channel 1"]),
        write_case(args.output, "multi-sixteen", multi(),
                   ["Multi mode, sixteen enabled parts mapped individually to channels 1..16",
                    "Twenty-five simultaneously held input notes; record actual voice allocation"]),
        write_case(args.output, "controls-programs", controls(),
                   ["Single mode, channel 1; preserve tempo change, SysEx and program transitions"],
                   tempo_change=True),
        write_case(args.output, "sl-samples", sl_samples(),
                   ["Shared Desktop SL image generated with --init-program; Single mode, channel 1",
                    "No program change: retain the selected Init sound with other oscillators off",
                    "Original SNDP messages select Osc1 sample bank 1 before Shape 0 (Clavinet)",
                    "Verify real sample reads and audible original PCM; selection alone is not proof"]),
    ]
    manifest = {
        "scope": "Authored deterministic MIDI workload inputs, not instrument qualification",
        "ppqn": PPQN,
        "initial_tempo_us_per_quarter": 500000,
        "release_tick": RELEASE,
        "end_tick": END,
        "cases": cases,
        "render_contract": [
            "Establish genuine instrument readiness before musical tick zero",
            "Pin firmware, identity, program and memory-image hashes on both hosts",
            "Require installed and recognized SL samples for sample-dependent qualification",
            "Record actual voice allocation, effects and sample use; MIDI alone cannot prove them",
            "Preserve remaining release/effect tails beyond file EOF when necessary",
            "Measure delivered original stereo frames against monotonic host time",
            "Separate startup, first-use translation, warm throughput and live deadlines",
            "Keep manufacturer/user music assets local; no automatic firmware runs",
        ],
    }
    with (args.output / "manifest.json").open("x") as stream:
        stream.write(json.dumps(manifest, indent=2) + "\n")
    print(f"Created {len(cases)} MIDI inputs under {args.output}; no firmware executed")


if __name__ == "__main__":
    main()
