# blofeld-emu

A standalone Waldorf Blofeld Desktop SL emulator for Linux x86-64 and macOS
ARM64. It boots the original Blofeld 1.25 firmware and runs the original synth
engine. Play over MIDI, open the LCD viewer, or render a MIDI file to stereo WAV.

Firmware, factory samples and populated images are local inputs.

## Build

Use LLVM and Clang 19 (including clang-tidy and clang-format), CMake 3.25+,
Ninja, Git, Python 3.11+ and zlib. On Linux, install the JACK development headers
(`libjack-jackd2-dev` on Debian). On macOS, LLVM 19 is available as
`llvm@19` through Homebrew.

```sh
cmake --preset release
cmake --build --preset release --target blofeld-headless
```

The core revisions are pinned in [cmake/dependencies.json](cmake/dependencies.json).
To use matching local checkouts, set `BLOFELD_CPU12_ROOT` and
`BLOFELD_DSP56300_ROOT` when configuring.

## Prepare a profile

Use the image generator in `../blofeld-src/tools/blofeld_image.py` for a
Desktop SL device and its sample image. Supply the original update, the boot
ROM and the matching MCU/DSP executable catalogues. Paths in the profile are
relative to the profile file.

```text
firmware "Blofeld_Firmware_V1.25.zip"
boot-rom "boot-rom.bin"
mcu-catalogue "mcu-code.json"
dsp-catalogue "dsp-code.json"
device-state "device.bfnvm"
nand "samples.bfnand"
```

```sh
build/release/blofeld-headless --profile instrument.profile --prepare
build/release/blofeld-headless --profile instrument.profile
```

Run `--prepare` again when the prepared assets no longer match the build or
profile. With no arguments, the emulator reads
`${XDG_CONFIG_HOME:-$HOME/.config}/blofeld-emu/default.profile`.

## Play or render

On Linux, use real-time JACK at 44.1 kHz with a period of 128 frames or less.
Connect your MIDI source to `blofeld-emu:midi_in` and route `midi_out` for
replies. Route stereo audio in your JACK graph or with:

```sh
jack_connect blofeld-emu:audio_l system:playback_1
jack_connect blofeld-emu:audio_r system:playback_2
```

On macOS, audio goes to the system output. Select Core MIDI endpoints with
`--midi-source ID` and `--midi-destination ID`.

```sh
tools/blofeld-display
build/release/blofeld-headless --profile instrument.profile \
  --midi score.mid --wav render.wav --tail 5
```

The LCD viewer requires GTK 4, PyGObject and Cairo bindings. The WAV command
renders offline and `--tail` adds whole seconds after the MIDI file ends.

[Architecture](doc/architecture.md) · [References](doc/README.txt) ·
[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
