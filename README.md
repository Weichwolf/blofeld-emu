# blofeld-emu

**Run original Blofeld firmware as a standalone software instrument.**

C++23 · LLVM 19 · GPL-3.0-only · Work in progress

The goal is original DSP sound, MIDI/USB-MIDI, display and persistent storage from
unmodified firmware 1.25. Our independent [cpu12](https://github.com/Weichwolf/cpu12)
and [dsp56300](https://github.com/Weichwolf/dsp56300) cores execute the firmware;
the emulator supplies device memory and peripherals. Compatibility concerns the
playable instrument and its timing. Firmware and manufacturer assets are local
inputs and are not included in this repository.

## Status

Connected firmware execution produces display output and PCM frames, with bounded
MIDI-driven audio evidence. A complete playable instrument, real-time throughput,
full USB behavior, SL sample playback and update/recovery remain unqualified.
The target architecture is under implementation; its proposed worker threads and
performance milestones must not be read as completed features.

## Architecture

```text
                  original firmware + persistent images
                                   |
 MIDI / USB <-> device model <-> CPU12 JIT + DSP56300 JIT
                                   |
                         PCM queue -> host audio
                                   |
                         display / persistent state
```

The cores execute native code over owned memory. The consumer coordinates virtual
time and device effects, with bounded queues for transport and audio. Avoid work
per simulated clock; retain effects that firmware and the instrument can observe.
See the [target design](doc/architecture-design.txt) and [complete objective](OBJECTIVE.md).

## Build

Requires LLVM 19, CMake >=3.25, Ninja, Python 3, Git and the pinned native
dependencies. Current MCU integration also needs Ghidra native SLEIGH sources;
set `GHIDRA_ROOT` if automatic discovery cannot find them. Core checkouts must
match [the dependency manifest](cmake/dependencies.json). Current CMake options
are `BLOFELD_OWN_DSP_ROOT`, `BLOFELD_CPU12_ROOT` and `BLOFELD_NATIVE_MCU`;
transitional build names remain migration work.

```sh
python3 tools/dependencies.py --fetch
python3 tools/dependencies.py --fetch --component pulseaudio
python3 tools/dependencies.py --fetch --component libsndfile
python3 tools/dependencies.py --fetch --component meson
cmake --preset release -DBUILD_TESTING=OFF -DBLOFELD_NATIVE_MCU=ON
cmake --build --preset release --target blofeld-headless
```

Configure/build use verified local dependency inputs. Ccache is selected when
available. Release retains strict compiler and static-analysis checks; `strict`
uses ASan/UBSan and `thread` uses separate TSan. Until real-time operation is
established, execution tests are suspended; only optimized startup/throughput
probes of at most 10 host seconds are allowed.

## Local references

[Manufacturer download manifests](doc/README.txt) record reference identities.
Keep firmware, samples, images and extracted programs untracked. Never publish a
populated device image. Use the CLI's `--help` for current runtime options.

- [Development rules](AGENTS.md)
- [Function acceptance catalogue](test/function-acceptance.tsv)
- [License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)

Concrete cross-project issues belong in private `Weichwolf/blofeld-src`.
