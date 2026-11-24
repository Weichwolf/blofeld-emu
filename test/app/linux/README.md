# JACK strings regression

Use an idle, real-time JACK server at 44.1 kHz with 64-frame periods. Supply a
prepared original 1.25 Desktop SL profile with factory presets and samples.

```sh
cmake --build --preset release --target blofeld-headless jack-nachtmusik
python3 test/app/linux/nachtmusik_jack.py build/release instrument.profile
```

The test replays the opening of Mozart's K.525 on A013 (BloStrings), captures both
channels and checks the LCD, MIDI delivery and underrun counters. It captures
JACK audio without connecting speakers. The event sequence is adapted from the
[public-domain Mutopia score](https://www.mutopiaproject.org/cgibin/piece-info.cgi?id=900).

`python3 tools/midi_bench_inputs.py --output build/nachtmusik-inputs` exports
the same event sequence as `nachtmusik.mid`, with one SMF tick per 44.1 kHz
frame. Render it in Single mode with the factory presets and `--tail 0`.

Run `blofeld-render-controls` with `BLOFELD_PCM_TIMING=1` to record accepted
WAV frames. Use `python3 tools/pcm_deadline_replay.py output.wav` to report
deadline slack at 44.1 kHz with three 64-frame periods and an ideal callback clock.

For other generated workloads, pass the SMF to
`build/release/jack-nachtmusik build/nachtmusik-inputs/<score>.mid` while the
corresponding instrument profile is already playable. Replay ends at the SMF
end-of-track time and reports how many MIDI events JACK accepted.
