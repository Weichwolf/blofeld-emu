# blofeld-emu

Standalone execution of original Blofeld 1.25 firmware with the independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) JIT cores. The runtime connects
firmware memory, register I/O, interrupts, persistent storage, USB-MIDI,
original DSP audio and LCD output. Playable real-time operation is not qualified.

Requires LLVM 19, CMake 3.25+, Ninja, Python 3 and Git. Core checkouts must match
[cmake/dependencies.json](cmake/dependencies.json); their paths can be selected
with `BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`.

```sh
python3 tools/dependencies.py --fetch --component zlib
python3 tools/dependencies.py --fetch --component pulseaudio
python3 tools/dependencies.py --fetch --component libsndfile
python3 tools/dependencies.py --fetch --component meson
cmake --preset release
cmake --build --preset release --target blofeld-headless
build/release/blofeld-headless --profile /path/to/default.profile
```

Without arguments the runtime reads `$XDG_CONFIG_HOME/blofeld-emu/default.profile`,
falling back to `~/.config/blofeld-emu/default.profile`. A profile contains local
paths, resolved relative to the profile file; quote paths containing spaces:

```text
firmware "firmware.zip"
boot-rom "boot.bin"
device-state "unit.bfnvm"
nand "unit.bfnand"
display "/path/to/existing/private/display-directory"
```

`display` is optional. NAND must already contain the required device data.
The audio consumer uses the default session output; USB/IP listens on loopback
port 3240 and requires a separately provisioned host USB/IP controller.
USB descriptors and endpoint traffic come from firmware execution.
The CLI currently provides the live path; SMF/WAV rendering remains unimplemented.

Release enables optimized execution. `strict` enables ASan/UBSan; `thread`
selects separate TSan. Ccache is used when available. Firmware startup probes
are explicit developer commands with a hard host-time deadline.

Manufacturer firmware, samples and other assets are local inputs and are never
included in source distributions. [Reference manifests](doc/README.txt) retain
source URLs and identities. [Acceptance requirements](OBJECTIVE.md) record the
complete instrument scope.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
