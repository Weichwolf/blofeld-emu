# blofeld-emu

Standalone Blofeld Desktop SL instrument for macOS ARM64 and Linux x86-64.
Execute unchanged original firmware through our independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) translators.

[OBJECTIVE.md](OBJECTIVE.md) defines complete acceptance,
[doc/architecture.md](doc/architecture.md) the current target, and
[ACCEPTANCE.tsv](ACCEPTANCE.tsv) the separate platform results.
Target >=10x sustained original offline output, >=4x intermediate; physical
keyboard-to-analog latency 2-5 ms, maximum 10 ms. Microbenchmarks do not qualify.

Complete known firmware AOT and joint host-code optimization before playback.
Prefer startup loading; build-time translation uses the same pipeline.
Preparation may reuse code artifacts validated against every input and ABI
identity; device snapshots never replace boot. Manufacturer inputs and derived
artifacts stay local.

## Build

Use C++23, packaged LLVM19/Clang, CMake 3.25+, Ninja, Python 3.11+ and Git.
Core checkouts must match [cmake/dependencies.json](cmake/dependencies.json);
select them with `BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`.

Linux:

```sh
sudo apt install clang-19 clang-tidy-19 clang-format-19 llvm-19-dev cmake ninja-build ccache python3-dev libpulse-dev zlib1g-dev
cmake --preset release
cmake --build --preset release --target quality
```

macOS:

```sh
brew install llvm@19 llvm ninja ccache python@3.14 gtk4 pygobject3 py3cairo
cmake --preset release
cmake --build --preset release --target quality --parallel 2
```

Release uses O3 without sanitizers; strict selects Debug/ASan/UBSan and thread
selects separate TSan. Instrument our code/generated code, never packaged foreign
libraries. Preserve compiler/linker warnings, clang-tidy and format checks.
On current macOS, select the packaged current Clang sanitizer runtime while
keeping the LLVM19 compiler:

```sh
export LLVM19_RESOURCE_DIR="$("$(brew --prefix llvm)/bin/clang" -print-resource-dir)"
cmake --preset strict
cmake --build --preset strict --target quality --parallel 2
```

## Run

```sh
build/release/blofeld-headless
build/release/blofeld-headless --profile /path/to/instrument.profile
build/release/blofeld-headless --profile /path/to/instrument.profile \
  --midi /path/to/score.mid --wav /path/to/render.wav --tail 5
```

Owner attribution uses a separate Release build with `-DBLOFELD_CENSUS=ON`.
Set `BLOFELD_CENSUS=1` for offline rendering. The terminal emits one JSON census
covering the interval from the first accepted PCM block to completion; exclude
that first block. Normalize counter deltas against the source-frame delta,
including the final block beyond the WAV boundary. CPU/PMCs, guest work and WAIT
advancement are separate. Wait-call duration includes scheduling delay and is
not a kernel-blocked-time counter. Session histograms include overlapping edge
sessions. On macOS, PMCs include user and kernel execution per performance level;
on Linux this reader provides CPU time only. `BLOFELD_CENSUS_SYMBOLS=/absolute/file`
adds timestamped original-symbol/native-address JSON lines during linking. Match
sampling addresses against ranges valid at the sample time. Normal builds contain
none of these probes. Report diagnostic perturbation against the plain build.

No arguments select $XDG_CONFIG_HOME/blofeld-emu/default.profile, falling back
to $HOME/.config/blofeld-emu/default.profile. Each profile line contains a field
and quoted path, relative to the profile:

```text
firmware "original-update.zip"
boot-rom "boot.bin"
dsp-catalogue "dsp-code.json"
mcu-catalogue "mcu-code.json"
device-state "device.bfnvm"
nand "samples.bfnand"
```

Supply the original update container, complete bound MCU/DSP executable catalogues and
existing versioned device/NAND images with explicit knowledge metadata.
Startup does not provision or overwrite image files. Use the common preset
producer ../blofeld-src/tools/blofeld_image.py with its required inputs;
`--init-program` selects the shared Init program. Preserve identity, active SL,
authentic programs/samples and the original boot.

`boot-rom` supplies 192 big-endian 24-bit main words, optionally followed by
81 EEPROM words, mapped separately at P:$FF0000 and P:$FFFF00. Alternatively,
`boot-prefix` selects the documented 55-word SPI prefix; the fields are exclusive.
Missing words remain unknown. The catalogue pins decoded firmware, NAND records
and separate ROM inputs and declares executable variants/entries/contracts.
Mismatched inputs or incomplete translation fail before VM execution.
See the [MCU](doc/architecture.md#local-mcu-catalogue-json-format-1) and
[DSP](doc/architecture.md#local-dsp-catalogue-json-format-1) catalogue contracts.

Live mode uses the normal host audio output, an external MIDI keyboard and the
independent viewer. Query actual rates/buffer grants and report native faults,
queues, underruns and disconnects. SIGINT/SIGTERM requests coordinated shutdown.

`--midi` and `--wav` are paired. Offline rendering opens no host audio or viewer;
`--tail` selects whole seconds after serialized MIDI/file end, default five.
Preserve musical time and sufficient release/effect tails. Publish only complete
valid original stereo WAV, never replace existing output, and report startup,
active time, valid frames and throughput separately.

## Display

```sh
tools/blofeld-display
# macOS source checkout with the Homebrew GTK interpreter:
python3.14 tools/blofeld-display
```

Debian requires python3-gi, python3-gi-cairo and gir1.2-gtk-4.0; macOS uses
the GTK/Python packages above. The installed launcher uses CMake's interpreter.
Emulator and viewer share $XDG_RUNTIME_DIR/blofeld-emu without path overrides.
The fallback is /run/user/UID on Linux, the Darwin user temporary directory on Mac.
Publish changed known LCD frames atomically as `display.pbm`, in live mode only.
Unknown state revokes the bitmap. The live-only `display.live` FIFO supplies
one-second presence checks without emulator heartbeat writes. Exit/crash clears
stale images; a valid live image hides the status line. No text dumps or download
controls are required.

## Local inputs and references

Generate deterministic workloads with
python3 tools/midi_bench_inputs.py --output build/local/bench-midi.
Its manifest identifies single-note, chord/pedal, sixteen-channel,
control/SysEx/tempo and SL cases. Verify actual voice/effect/sample use;
matching files alone do not establish instrument acceptance.

The optional offline bootstrap tool retains source/listing/provenance and requires
the canonical verified 55-word hash:

```sh
python3 tools/extract_nxp_spi_bootstrap.py \
  --native-assembler /path/to/dsp56300-asm \
  --manual build/local/reference-inputs/DSP56371UM.pdf \
  --output build/local/spi-prefix
```

Use [doc/README.txt](doc/README.txt) for primary reference indexes and topic
coordination. Keep firmware, samples, manuals and populated images untracked.
[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
