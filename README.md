# blofeld-emu

A standalone Waldorf Blofeld Desktop emulator for Linux x86-64 and macOS
ARM64. It boots the original Blofeld 1.25 firmware and runs the original synth
engine. Play over MIDI, open the LCD viewer, or render a MIDI file to stereo WAV.

[Linux quickstart](doc/quickstart.md) · [Architecture](doc/architecture.md) ·
[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)

## Build

Use LLVM and Clang 19 (including clang-tidy and clang-format), CMake 3.25+,
Ninja, Git, Python 3.11+ and zlib. On Linux, install the JACK development headers
(`libjack-jackd2-dev` on Debian). On macOS, LLVM 19 is available as
`llvm@19` through Homebrew.

```sh
nice -n 15 cmake --preset release
nice -n 15 cmake --build --preset release --target blofeld-headless
```

The core revisions are pinned in [cmake/dependencies.json](cmake/dependencies.json).
To use matching local checkouts, set `BLOFELD_CPU12_ROOT` and
`BLOFELD_DSP56300_ROOT` when configuring.

## Prepare and run

Save the original Waldorf firmware, factory sounds and MIDI samples as
`firmware.zip`, `sounds.zip` and `samples.zip` in `build/local/downloads/`.
Install `pdftotext` (Poppler). The preparation target creates a local Desktop
profile and compiles the firmware for your host.

```sh
nice -n 15 cmake --build --preset release --target blofeld-prepare
build/release/blofeld-headless --profile build/release/desktop/desktop.profile
```

On Linux, connect MIDI and stereo audio through JACK. On macOS, audio goes to
the system output and `--midi-source ID` selects a Core MIDI endpoint. Run
`tools/blofeld-display` for the LCD viewer (GTK 4, PyGObject and Cairo).

Render offline by adding `--midi score.mid --wav render.wav --tail 5`. The
`--tail` option adds seconds after the MIDI file ends.
