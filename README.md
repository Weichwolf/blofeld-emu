# blofeld-emu

Standalone Blofeld Desktop SL instrument for macOS ARM64 and Linux x86-64,
executing unchanged original firmware through the independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) translators.

Load the selected firmware at startup. Complete Ahead-of-Time Binary Translation
of all known executable code and variants, combine it with eligible host LLVM IR
and optimize it before playback. Incomplete translation fails with code identity,
address and reason. Both cores also support build-time export of linkable LLVM
bitcode or relocatable objects through the same semantic pipeline.

Preserve original boot, synthesis, samples, effects, voice allocation, MIDI/SysEx,
controls, display, persistent images, updates and recovery. Retain original loader
and observable program-write effects while selecting prepared native variants.
Mutable instrument data resides in host RAM; fixed tables use validated constants.
Firmware-derived artifacts remain local and untracked under `build/`.
Development always runs complete AOT and optimization. For operation, reuse a
validated persistent artifact cache; compile it from exact firmware/code variants,
core/LLVM pipeline, host features and binding identity. Rebuild rejected entries
before playback and preserve original boot rather than caching device snapshots.

Live mode uses an external MIDI keyboard, the normal host audio output and a
separate display viewer. Query actual audio/MIDI grants and keep bounded queues,
ordering, musical time, reset and cancellation. Target physical key-to-analog-sound
latency is 2-5 ms with a 10 ms maximum on the qualified complete device chain.
Deliver every audio buffer on time, including first use of every firmware path.

Offline MIDI-to-WAV runs at maximum throughput, without host audio or a viewer.
Target at least ten times real time under the combined sixteen-part,
up-to-twenty-five-voice effects/sample workload; four times is intermediate.
At 44.1 kHz, ten times means 10 * 44100 = 441000 valid original stereo frames/s.
Report compilation, original boot/readiness and warm rendering separately.

The [objective](OBJECTIVE.md), [acceptance catalogue](ACCEPTANCE.tsv) and
[target architecture](doc/architecture-design.txt) define the required behavior
and qualification. Acceptance results remain in the catalogue.

- `src/vi/`: instrument behavior and firmware device contracts.
- `src/host/`: reusable operating-system and transport bindings.
- `src/app/`: composition, publication and UI.
- `test/`: unit tests mirroring `src/`; instrument acceptance uses the emulator.

Generate deterministic MIDI workload inputs with
`python3 tools/midi_bench_inputs.py --output build/local/bench-midi`.
The five cases cover single notes, chords/pedal, sixteen channels,
controllers/program changes/SysEx/tempo and SL samples. Their manifest records
hashes and required instrument setup; generation executes no firmware. Render
comparisons require matching firmware, programs and images, with actual
voice/FX/sample use verified.
For `sl-samples.mid`, generate the common image with
`../blofeld-src/tools/blofeld_image.py --init-program` and its required input arguments.
The file selects Osc1 bank 1, then Shape 0 through ordinary parameter-change SysEx;
it retains the selected Init sound and requires actual sample playback verification.

Hosts are Linux x86-64 and macOS ARM64, with LLVM 19, CMake 3.25+, Ninja,
Python 3.11+ and Git. Core checkouts must match
[cmake/dependencies.json](cmake/dependencies.json); select paths with
`BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`.

On Linux:

```sh
sudo apt install clang-19 clang-tidy-19 clang-format-19 llvm-19-dev cmake ninja-build ccache python3-dev libpulse-dev zlib1g-dev
cmake --preset release
cmake --build --preset release --target quality
```

On macOS, use the Homebrew tools and native CoreAudio backend; PulseAudio is not
needed for this backend:

```sh
brew install llvm@19 llvm ninja ccache python@3.14 gtk4 pygobject3 py3cairo
cmake --preset release
cmake --build --preset release --target quality --parallel 2
```

Presets discover LLVM 19 without replacing system compilers. Both platforms use
packaged third-party libraries without foreign source builds or sanitizer instrumentation.
On current macOS, select the packaged current Clang sanitizer runtime
while retaining the LLVM 19 compiler:

```sh
export LLVM19_RESOURCE_DIR="$("$(brew --prefix llvm)/bin/clang" -print-resource-dir)"
cmake --preset strict
cmake --build --preset strict --target quality --parallel 2
cmake --preset thread
```

`strict` enables ASan/UBSan for our code and generated JIT code; `thread`
selects separate TSan. Packaged third-party libraries remain unmodified. Compiler warnings,
linker hardening, clang-tidy and clang-format are enforced. Ccache is used when
available. Core correctness suites run in their own projects.

```sh
build/release/blofeld-headless --profile /path/to/instrument.profile
build/release/blofeld-headless --profile /path/to/instrument.profile \
  --midi /path/to/score.mid --wav /path/to/render.wav --tail 5
```

`--midi` and `--wav` are paired and render without opening a host audio device.
`--tail` selects whole seconds after the final serialized MIDI event/track end;
it defaults to five seconds. Choose sufficient release/effect time for the programs.
Successful renders report valid frame totals, startup/active host time and throughput.
Report MIDI deadline deviation separately. Require original identity and valid PCM
before accepting playback. Existing output files are never replaced.

With no arguments it reads `$XDG_CONFIG_HOME/blofeld-emu/default.profile`,
falling back to `$HOME/.config/blofeld-emu/default.profile`. Profiles contain
one field and quoted path per line; paths are relative to the profile file:

```text
boot-rom "boot.bin"
device-state "device.bfnvm"
nand "samples.bfnand"
```

The `boot-rom` input contains exactly 192 big-endian 24-bit words. Alternatively,
`boot-prefix` explicitly selects the 55 known words of the published NXP SPI
bootstrap. The two fields are mutually exclusive. The prefix is not a complete
hardware ROM dump; addresses beyond its known words remain unmapped and fail
on access. Do not pad missing words with invented values.

The optional offline assembler in `cmake/dependencies.json` can build this prefix
natively. With that tool and the verified NXP manual available locally:

```sh
python3.14 tools/extract_nxp_spi_bootstrap.py \
  --native-assembler /path/to/dsp56300-asm \
  --manual build/local/reference-inputs/DSP56371UM.pdf \
  --output build/local/spi-prefix
```

The tool requires the existing canonical NXP-assembled 55-word hash, retains
source/listings/provenance locally and also supports the pinned vendor assembler
through `--nxp-assembler`. It introduces no runtime emulator dependency.

The MCU image uses
the versioned `.bfnvm` format with explicit known-byte and erase metadata.
Images must already exist; startup does not provision or modify files.
The media image uses the checksummed `.bfnand` format with explicit known bits.
The emulator and viewer share `$XDG_RUNTIME_DIR/blofeld-emu`, without display-path
overrides. The default runtime root is `/run/user/UID` on Linux and the Darwin
user temporary directory on macOS. The MCU owner supplies controller snapshots;
the app publishes atomic `display.pbm` frames only when pixels change. Unknown
pixels revoke the bitmap until a known frame is available. A live-only
`display.live` pipe stays open for the publisher lifetime. The viewer checks it
once per second and clears stale images when the emulator exits or crashes;
this requires no heartbeat writes or firmware/audio work.
The optional `firmware` path selects the original update container. Preserve the
complete original boot and require MIDI identity, valid PCM and playable sound
before admitting instrument operation.
SIGINT/SIGTERM request coordinated shutdown. Live status reports server-submitted
frames, local queue occupancy, native buffer grants, estimated output latency and host/device
underruns. The approximate producer reserve excludes blocking in the host consumer;
it is not the offline instrument throughput or proof of physical playback.

The viewer needs `python3-gi`, `python3-gi-cairo` and `gir1.2-gtk-4.0` on Debian,
or the Homebrew GTK/Python packages above on macOS:

```sh
tools/blofeld-display
# macOS checkout:
python3.14 tools/blofeld-display
```

The installed launcher uses the Python interpreter selected by CMake.
It watches `$XDG_RUNTIME_DIR/blofeld-emu/display.pbm`, follows directory
recreation. A valid live image hides the status line; publisher absence, missing
LCD state and errors remain visible.
PBM publication consumes independent LCD images outside core execution and audio
callbacks. No automatic text copy is written.

Firmware, samples, manuals and populated images remain local and are not
redistributed. [Reference manifests](doc/README.txt) retain their identities.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
