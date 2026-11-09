# blofeld-emu

Blofeld firmware VM project using the independent
[cpu12](https://github.com/Weichwolf/cpu12) and
[dsp56300](https://github.com/Weichwolf/dsp56300) JIT cores.

The current source contains the LCD register/data contract and MCU port binding, image capture,
atomic PBM/text publication, GTK viewer, 1.25 update-container loader, MCU
image codec, direct MCU memory mapping, GPIO, Flash reset/protection/error registers,
free-running timer capture/compare, CRG/USB clock selection, periodic interrupts
and interrupt wakeup, SCI character/register handling, USB configuration and direct
endpoint-buffer windows, and independent
MCU/DSP execution threads and DSP56371 PLL register/nominal ratio handling.
The VM exposes bounded, timed DIN input shared by MCU SCI and DSP Timer0,
with genuine SCI output. The macOS CoreMIDI input backend discovers actual source
identifiers and receives a selected group with native timestamps through a bounded
queue. Application routing and its audio-aligned time mapping remain incomplete.
The host SMF reader streams timestamped events with tempo/SMPTE handling and
separate metadata; independent patterns require explicit selection. An offline
PCM consumer writes signed 24-bit stereo WAV, rejecting underruns, frame/time
discontinuities and drift. Original PCM carries per-frame completion points in
the common instrument clock. The file adapter feeds the VM through a bounded input frontier;
producer delays suspend native execution without advancing device time. The offline
command observes the original MCU application WAI before sending identity queries,
then requires the original reply and valid PCM, selects a musical origin, streams
the file and retains its EOF barrier until the output boundary is
published. Only the musical interval and selected tail enter the WAV; startup
PCM is validated and discarded. Failed or interrupted renders stay unpublished.
Normal file rendering remains gated by incomplete original-firmware readiness.
The viewer receives LCD snapshots while firmware readiness is pending.
Complete instrument acceptance and sustained real-time operation remain unverified.
DSP interrupt arbitration uses configured priorities, SR masks and source ordering.
Timer, DMA and DAX requests enter native vectors with VBA/REP/RTI handling;
timer and DMA acknowledgements occur on accepted admission. SHI and ESAI
interrupt delivery remains incomplete.
DMA registers, A–E counters and address modes handle requested transfers and software blocks.
ESAI_1 transmit registers bind DMA writes to actual words; explicit slot events
preserve masks, word precision, empty requests and retained-data underruns.
ESAI_1 and DAX run from nominal VM clock deadlines, including PLL conversion.
ESAI_1 supplies signed stereo PCM blocks to an application consumer with positions
and explicit underrun bits. Its receive path remains incomplete.
The application connects those blocks to a bounded host queue. Linux uses a dedicated
PulseAudio client owner; macOS uses a CoreAudio output unit. The producer supplies
stereo 24-bit PCM in 32-bit containers. CoreAudio consumes prepared frames through
a lock-free render buffer and converts them exactly to float32; the output unit
handles the granted device format/rate. Actual device formats and buffer grants
are queried. macOS requests a small hardware buffer within its reported range;
its dedicated DSP producer uses a Mach deadline and an asynchronous audio workgroup.
The output owner sizes bounded queues from actual device quanta, including variable
buffer bounds. Host backpressure paces the DSP owner. Reset stops and flushes the old
stream epoch before accepting new frames. Disconnects, endpoint changes and host
underruns stop execution with an error. macOS also rejects missing, invalid or
discontinuous native render timestamps before consuming more queued frames.
Reported output latency combines queued
frames and native device, stream and output-unit latency information; it is not a physical
keyboard-to-sound measurement.
DAX register access and DMA writes retain the distinct XADR/channel buffers,
non-audio metadata, block position, refill requests and observed-underrun clearing.
Its output does not create a second host PCM stream.
DSP timer registers, CLK/2 and shared prescaling implement GPIO timer, event-counter
and capture modes with deadlines, W1C flags and compare DMA requests. The other
timer modes remain incomplete. WAIT advances device time to the next timer event.
The MCU samples a VM-owned pedal input; firmware retains its polarity/change handling.
Low/high X/Y RAM uses direct native grants with initialization tracking and
OMR-selected program-bank sharing. Optional data-ROM views borrow local contents.
DSP GPIO and decoded page-read transactions use the profile's validated media
image directly. Read/reset operations complete at their own virtual deadlines;
PLL configuration converts nominal reference periods to DSP cycles. Confirmed block erases
update the actual image at completion, respecting stored validity/protection. Status reads
retain busy/pass/fail state; interrupted erase contents become unknown and can be recovered
by another complete erase. Serialization refreshes the encoded image checksum on demand.
Page program stages bounded actual bytes, preserves existing zero bits and tracks
independent main/spare program allowances. Area pointers follow 00h/01h/50h semantics;
interrupted writes retain known zeros and mark possibly changed bits unknown.
Unknown protection/history or exceeded qualified allowances stop execution.
Identification and automatic page continuation remain incomplete. The application
does not yet persist changed images to disk.
The SHI receive FIFO connects MCU port writes to DSP program loading; HTX and
IOSR retain actual full-duplex words, with phase-dependent loading and underrun handling.
Completed bootstrap words use bounded FIFO admission outside native callbacks;
a full queue applies VM backpressure, and stop/reset cancel pending admission. MCU U0
assertions retire the previous DSP Core before peripheral reset and ROM restart.
`blofeld-headless` loads local images and executes their native JIT paths;
unimplemented device accesses stop with context. Flash program/erase and interrupts,
watchdog reset/recovery, USB transactions, MCU MISO sampling and SHI interrupt delivery,
the remaining device contracts and host MIDI are incomplete, so it is not yet
a playable instrument. Complete requirements are in
[OBJECTIVE.md](OBJECTIVE.md) and [ACCEPTANCE.tsv](ACCEPTANCE.tsv).

- `src/vi/`: instrument behavior and firmware device contracts.
- `src/host/`: reusable operating-system and transport backends.
- `src/app/`: composition, publication and UI.
- `test/`: simple unit tests mirroring `src/`; instrument acceptance uses the emulator.

See the [target architecture](doc/architecture-design.txt) for independent core
execution, bounded channels and audio-driven consumption.

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
python3 tools/dependencies.py --fetch --component zlib
python3 tools/dependencies.py --fetch --component pulseaudio
python3 tools/dependencies.py --fetch --component libsndfile
python3 tools/dependencies.py --fetch --component meson
cmake --preset release
cmake --build --preset release --target quality
```

On macOS, use the Homebrew tools and native CoreAudio backend; PulseAudio,
libsndfile and Meson are not needed for this backend:

```sh
brew install llvm@19 ninja ccache python@3.14 gtk4 pygobject3 py3cairo
python3.14 tools/dependencies.py --fetch --component zlib
cmake --preset release
cmake --build --preset release --target quality --parallel 2
```

Presets discover LLVM 19 without replacing system compilers. For macOS 26.4+
sanitizer builds, use the pinned core's LLVM 19 runtime bootstrap with its
upstream dyld fixes, then share the generated resource directory:

```sh
export LLVM19_RESOURCE_DIR="$(python3.14 ../cpu12/tools/darwin_runtime.py --jobs 2)"
cmake --preset strict
cmake --build --preset strict --target quality --parallel 2
cmake --preset thread
```

`strict` enables ASan/UBSan; `thread` selects separate TSan. Compiler warnings,
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
MIDI deadline deviation remains unmeasured. Startup fails after ten host seconds
without both original identity and PCM. Existing output files are never replaced.

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
the app publishes atomic `display.pbm` and `display.txt` files. Unknown pixels remain
explicit in text and remove the bitmap until a known frame is available.
The optional `firmware` path is parsed but not yet connected to execution. Startup
currently exits unsuccessfully because the instrument is incomplete; successful execution of a short program is not instrument readiness.
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
recreation. Its header button saves a PNG to the user's Pictures directory.
For an explicit path without a window, use `tools/blofeld-display --screenshot FILE.png`. PBM and text publication
consume independent LCD images outside core execution and audio callbacks.

Firmware, samples, manuals and populated images remain local and are not
redistributed. [Reference manifests](doc/README.txt) retain their identities.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
