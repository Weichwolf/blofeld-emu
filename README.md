# blofeld-emu

Headless emulator for original Blofeld firmware 1.25, using our
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) JIT cores. It contains models for
memory, peripherals, MIDI/USB, display and persistent storage. Real-time playable
instrument operation is not yet qualified.

Original firmware, samples and other manufacturer assets are separate local
inputs and are not distributed with the source.

## Build

Requires LLVM 19, CMake >=3.25, Ninja, Python 3, Git and Ghidra native SLEIGH
sources for the keyboard controller. The core checkouts must match
[cmake/dependencies.json](cmake/dependencies.json). Select their locations with
`BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`; set `GHIDRA_ROOT` if needed.

```sh
python3 tools/dependencies.py --fetch --component zlib
python3 tools/dependencies.py --fetch --component pulseaudio
python3 tools/dependencies.py --fetch --component libsndfile
python3 tools/dependencies.py --fetch --component meson
cmake --preset release -DBUILD_TESTING=OFF
cmake --build --preset release --target blofeld-headless
```

Configure/build use verified local dependencies. Ccache is selected when
available. `strict` enables ASan/UBSan; `thread` selects separate TSan.

## References

[Manufacturer reference identities](doc/README.txt) record download sources and
hashes. Keep downloaded files, extracted programs and populated device images
untracked. Local working inputs live under ignored `build/local/`.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
