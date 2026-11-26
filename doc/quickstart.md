# Linux quickstart

## Build and prepare

On Debian 13, install the build tools and JACK.

```sh
sudo apt install build-essential ca-certificates git cmake ninja-build pkg-config python3 \
  clang-19 clang-format-19 clang-tidy-19 llvm-19-dev lld-19 zlib1g-dev poppler-utils \
  jackd2 libjack-jackd2-dev alsa-utils

git clone https://github.com/Weichwolf/blofeld-emu.git
cd blofeld-emu
for core in cpu12 dsp56300; do
  git clone "https://github.com/Weichwolf/$core.git" "../$core"
  pin=$(python3 -c 'import json, sys; print(json.load(open("cmake/dependencies.json"))["independent_" + sys.argv[1]]["revision"])' "$core")
  git -C "../$core" switch --detach "$pin"
done
```

Create `build/local/downloads/` and save the three Waldorf downloads there.

| Download | Save as |
| --- | --- |
| [Firmware 1.25](https://downloads.waldorfmusic.com/cloud/index.php/s/MMtiTjL3PTgExsg) | `firmware.zip` |
| [Factory Sounds 2012](https://downloads.waldorfmusic.com/cloud/index.php/s/fi4DWR7G2aXnWQ3) | `sounds.zip` |
| [Factory Samples (MIDI)](https://downloads.waldorfmusic.com/cloud/index.php/s/33B3t58F6GMAj4j) | `samples.zip` |

```sh
nice -n 15 cmake --preset release -DLLVM_DIR="$(llvm-config-19 --cmakedir)" \
  -DBLOFELD_CPU12_ROOT="$PWD/../cpu12" -DBLOFELD_DSP56300_ROOT="$PWD/../dsp56300"
nice -n 15 cmake --build --preset release --target blofeld-prepare jack-nachtmusik
```

`blofeld-prepare` builds the emulator, fetches and verifies the NXP manual and
assembler, creates `build/release/desktop/desktop.profile` and its data, then
compiles the original firmware for this host. Wait for `AOT preparation complete`.
The first compilation can take several minutes. Repeating the target keeps the
saved device state and uses the matching native cache. To use another input
folder, configure `BLOFELD_DOWNLOADS` with its absolute path.

## Live playback with JACK and the display

Use JACK at **44.1 kHz and 64 frames per period** for the example. On Debian,
allow realtime scheduling during `jackd2` installation. Add your user to the
`audio` group if needed (`sudo usermod -aG audio "$USER"`), then log out and in.
`ulimit -r` should show a positive limit. If you already have a JACK server,
use it with those settings. Otherwise find your device with `aplay -l`, replace
`hw:USB` with its ALSA card ID, and leave JACK running in its terminal.

```sh
jackd -R -d alsa -d hw:USB -r 44100 -p 64 -n 3 -X seq
```

Start the emulator and wait for `Live MIDI input active`.

```sh
build/release/blofeld-headless --profile build/release/desktop/desktop.profile
```

Open the LCD viewer in a second terminal.

```sh
sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-4.0
./tools/blofeld-display
```

Connect the emulator to the speakers, choosing your playback ports with
`jack_lsp` if needed. The replay plays the opening of Mozart's *Eine kleine
Nachtmusik* on A013 (BloStrings). To play from a controller or DAW, connect its
JACK MIDI output to `blofeld-emu:midi_in`.

```sh
jack_connect blofeld-emu:audio_l system:playback_1
jack_connect blofeld-emu:audio_r system:playback_2
build/release/jack-nachtmusik test/app/linux/nachtmusik-jack.events
```

## Render the same score offline

Run from the checkout with the same prepared profile. Choose a new directory
for the MIDI generator on subsequent runs.

```sh
python3 tools/midi_bench_inputs.py --output build/nachtmusik-inputs
build/release/blofeld-headless --profile build/release/desktop/desktop.profile \
  --midi build/nachtmusik-inputs/nachtmusik.mid --wav build/nachtmusik.wav --tail 0
```
