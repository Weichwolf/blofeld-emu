# blofeld-emu

Blofeld firmware VM project using the independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) JIT cores.

The current source contains the LCD register/data contract, image capture,
atomic PBM/text publication, GTK viewer, 1.25 update-container loader, MCU
image codec, direct MCU memory mapping, GPIO, free-running timer capture/compare
flags and interrupt wakeup, and independent MCU/DSP execution threads. The SHI
receive FIFO connects MCU port writes to DSP program loading.
`blofeld-headless` loads local images and executes their native JIT paths;
unimplemented device accesses stop with context. SHI transmit/interrupt delivery,
the remaining device contracts, audio and MIDI are incomplete, so it is not yet
a playable instrument. Complete requirements are in
[OBJECTIVE.md](OBJECTIVE.md) and [ACCEPTANCE.tsv](ACCEPTANCE.tsv).

- `src/vi/`: instrument behavior and firmware device contracts.
- `src/host/`: reusable operating-system and transport backends.
- `src/app/`: composition, publication and UI.
- `test/`: simple unit tests mirroring `src/`; instrument acceptance uses the emulator.

See the [target architecture](doc/architecture-design.txt) for independent core
execution, bounded channels and audio-driven consumption.

Requires Linux, LLVM 19, CMake 3.25+, Ninja, Python 3 and Git. Core checkouts must match
[cmake/dependencies.json](cmake/dependencies.json); select paths with
`BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`.

```sh
python3 tools/dependencies.py --fetch --component zlib
cmake --preset release
cmake --build --preset release --target quality
```

`strict` enables ASan/UBSan; `thread` selects separate TSan. Compiler warnings,
linker hardening, clang-tidy and clang-format are enforced. Ccache is used when
available. Core correctness suites run in their own projects.

```sh
build/release/blofeld-headless --profile /path/to/instrument.profile
```

With no arguments it reads `$XDG_CONFIG_HOME/blofeld-emu/default.profile`,
falling back to `$HOME/.config/blofeld-emu/default.profile`. Profiles contain
one field and quoted path per line; paths are relative to the profile file:

```text
boot-rom "boot.bin"
device-state "device.bfnvm"
```

The DSP boot input contains 192 big-endian 24-bit words. The MCU image uses
the versioned `.bfnvm` format with explicit known-byte and erase metadata.
Images must already exist; startup does not provision or modify files.
Optional `firmware`, `nand` and `display` paths are parsed but not yet connected
to execution. Startup currently exits unsuccessfully because the instrument
is incomplete; successful execution of a short program is not instrument readiness.

The viewer needs `python3-gi`, `python3-gi-cairo` and `gir1.2-gtk-4.0` on Debian:

```sh
tools/blofeld-display
```

It watches `$XDG_RUNTIME_DIR/blofeld-emu/display.pbm`, follows directory
recreation and can save a PNG screenshot. PBM and text publication consume
independent LCD images outside core execution and audio callbacks.

Firmware, samples, manuals and populated images remain local and are not
redistributed. [Reference manifests](doc/README.txt) retain their identities.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
