# blofeld-emu

Blofeld firmware VM project using the independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) JIT cores.

The current source contains the LCD register/data contract, image capture,
atomic PBM/text publication and GTK display viewer. There is currently no
playable instrument executable. Complete instrument requirements are in
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
cmake --preset release
cmake --build --preset release --target quality
```

`strict` enables ASan/UBSan; `thread` selects separate TSan. Compiler warnings,
linker hardening, clang-tidy and clang-format are enforced. Ccache is used when
available. Core correctness suites run in their own projects.

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
