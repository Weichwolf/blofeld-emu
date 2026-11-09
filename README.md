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
The application connects those blocks to a bounded host queue and a dedicated
PulseAudio client owner. The stream uses stereo 24-bit PCM in 32-bit containers;
actual server/sink formats and buffer grants are queried, and format/rate conversion
is explicit. Host backpressure paces the DSP owner. Reset corks and flushes the old
stream epoch before accepting new frames. Disconnects, endpoint changes and host
underruns stop execution with an error.
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

Requires Linux, LLVM 19, CMake 3.25+, Ninja, Python 3 and Git. Core checkouts must match
[cmake/dependencies.json](cmake/dependencies.json); select paths with
`BLOFELD_CPU12_ROOT` and `BLOFELD_DSP56300_ROOT`.

```sh
python3 tools/dependencies.py --fetch --component zlib
python3 tools/dependencies.py --fetch --component pulseaudio
python3 tools/dependencies.py --fetch --component libsndfile
python3 tools/dependencies.py --fetch --component meson
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
nand "samples.bfnand"
```

The DSP boot input contains 192 big-endian 24-bit words. The MCU image uses
the versioned `.bfnvm` format with explicit known-byte and erase metadata.
Images must already exist; startup does not provision or modify files.
The media image uses the checksummed `.bfnand` format with explicit known bits.
The optional `display` directory overrides `$XDG_RUNTIME_DIR/blofeld-emu`
(default runtime root: `/run/user/UID`). The MCU owner supplies controller snapshots;
the app publishes atomic `display.pbm` and `display.txt` files. Unknown pixels remain
explicit in text and remove the bitmap until a known frame is available.
The optional `firmware` path is parsed but not yet connected to execution. Startup
currently exits unsuccessfully because the instrument is incomplete; successful execution of a short program is not instrument readiness.
SIGINT/SIGTERM request coordinated shutdown. Live status reports server-submitted
frames, local queue occupancy, server buffer grants, measured latency and host/device
underruns. The approximate producer reserve excludes blocking in the host consumer;
it is not the offline instrument throughput or proof of physical playback.

The viewer needs `python3-gi`, `python3-gi-cairo` and `gir1.2-gtk-4.0` on Debian:

```sh
tools/blofeld-display
```

It watches `$XDG_RUNTIME_DIR/blofeld-emu/display.pbm`, follows directory
recreation. Its header button saves a PNG to the user's Pictures directory.
For an explicit path without a window, use `tools/blofeld-display --screenshot FILE.png`. PBM and text publication
consume independent LCD images outside core execution and audio callbacks.

Firmware, samples, manuals and populated images remain local and are not
redistributed. [Reference manifests](doc/README.txt) retain their identities.

[License](LICENSE) · [Dependency notices](THIRD_PARTY.txt)
