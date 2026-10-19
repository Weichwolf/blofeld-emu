Blofeld firmware emulator — initial headless bring-up

License: GPL-3.0-only. Dependency notices are in THIRD_PARTY.txt. Manufacturer
firmware and reference assets are separate local inputs and are not included.

Objective
Execute original firmware 1.25 for Desktop and Keyboard, with a faithful board
model, MIDI/audio and reproducible firmware-development tests. The complete
acceptance scope is in doc/objective.txt; evidence and gaps are in doc/coverage.txt.
This is not yet a booting Blofeld, a playable synthesizer or a verified
replacement-firmware target.

Build and quality checks
Requirements: LLVM 19 (clang, clang-tidy, clang-format and sanitizer runtime),
CMake >= 3.25, Ninja, Git, GNU patch, Python 3, zlib development files and Ghidra 12.1.2
including its native SLEIGH sources and compiler. Set GHIDRA_ROOT if it is not
installed under ~/.local/opt/ghidra_*_PUBLIC. Native SLEIGH does not need Java at
runtime.
DSP56300_ROOT must point to a clean checkout of the revision in
doc/dependencies.json, including asmjit sources (default: ~/Git/dsp56300).

  cmake --preset strict
  cmake --build --preset strict
  cmake --build build/strict --target quality

clang-tidy runs as a mandatory build step on every project C++ translation unit;
format-check checks formatting without changing files. All diagnostics are
errors, including the 120-line function limit. ASan/UBSan and additional integer
checks are enabled in strict builds; native SLEIGH and DSP sources also receive
ASan/UBSan. The linked system zlib is not rebuilt with project instrumentation;
instrumenting this dependency remains a quality-gate gap.
release retains warning/static-analysis gates and disables sanitizers for timing
measurements. thread selects separate TSan instrumentation. No concurrency is
currently implemented. Native CPU adapters use exceptions; any future audio
callback must keep allocation and exceptions outside its real-time boundary.

Local manufacturer references
Manifests under doc/ record URLs and SHA-256 values. Never commit or redistribute
the downloaded files or their extracted contents. Parser tests that require the
original ZIP are explicitly skipped when that private reference is absent.

  python3 tools/references.py --verify
  python3 tools/references.py --fetch
  python3 tools/extract_runtime.py doc/Blofeld_Firmware_V1.25.zip --output doc/local/runtime-1.25
  build/strict/blofeld-headless --firmware doc/Blofeld_Firmware_V1.25.zip --inspect
  build/strict/blofeld-headless --firmware doc/Blofeld_Firmware_V1.25.zip --trace doc/local/mcu.trace --instructions 1000

Explicit simulated pin inputs can explore the next boot stage; see
doc/mcu-model.txt for the implemented map, GPIO behavior and remaining limits.
doc/mcu-flash-config.txt covers reset security/protection and flash-clock
configuration, including the isolated original updater initialization check.
doc/mcu-flash-commands.txt covers the explicit reference command/milestone model,
interrupted-NVM unknownness and isolated original programming/recovery-vector checks.
doc/mcu-flash-persistence.txt describes private --flash-in/--flash-out MCU NVM
files, cold reconstruction, atomic host publication and process-death tests.
doc/original-update-restarts.txt covers 56 original-command cold restarts and
the original resident SCI receiver loading an unchanged RAM helper with own framing.
doc/original-parallel-update.txt covers the normal RAM-copy path, original caller
ABI and first helper callback under an explicit parallel-input fixture.
doc/mcu-wai.txt covers native WAI, idle/wakeup frames, CLI/TAP ordering and the
standalone driver's explicit wait stop when no scheduler supplies an interrupt.
doc/display.txt describes the separate ST7565S RAM/command/read-pipeline reference;
the candidate display and its board wiring are not hardware-verified.
doc/display-parallel.txt covers logical 6800 pin cycles and the complete original
first helper's display reads/return under an explicit GPIO/peripheral fixture.
doc/original-application.txt covers original reset through main WAI, missing-input
retry and both controlled PM3 levels under an untimed GPIO/USB/LCD/clock fixture.
doc/original-application-update.txt continues those same cold CPUs through two
original XIRQ/helper requests, first-helper return/tail/main WAI and main-helper
initialization up to its unqualified idle FSTAT write.
doc/mcu-sci.txt describes the bus-clocked serial model and its timing limits;
it does not yet provide a host MIDI port or automatic peripheral scheduling.
doc/midi-input.txt describes bounded byte-to-wire serialization, explicit SCI
pad connection and partial-frame replay, including the original MCU receive path.
doc/mcu-timer.txt covers timer captures, compare outputs, flags and their limits.
doc/mcu-interrupts.txt covers CPU exception frames, priority and peripheral requests;
doc/mcu-rti.txt describes the separate oscillator-clocked periodic interrupt.
doc/mcu-usb.txt covers USB registers/buffers, decoder-event inputs, interrupts and CPU operand widths.
doc/mcu-code-cache.txt describes bounded instruction caching and mutation tests.
doc/mcu-checkpoints.txt describes immutable in-process MCU checkpoints, native
WAI/SCI replay, peripheral/flash state and the remaining whole-board snapshot scope.
doc/dsp-checkpoints.txt covers native DSP checkpoints, latched REP/IRQ state,
peripheral phases and coordinated MCU/DSP/link replay during original loading.
doc/dsp-shi.txt covers the DSP-side SPI slave and native MCU GPIO exchange tests.
doc/dsp-interrupts.txt covers SHI/TIM arbitration, native fast/long entry, nested
priority, RTI, relocated vectors and DO/REP behavior at instruction boundaries.
doc/dsp-core.txt covers native DSP execution and the documented SPI bootstrap
loading the unchanged Blofeld first stage. A separate original MCU regression
captures its complete GPIO transmission under explicit pin observations
(doc/mcu-boot.txt). doc/coupled-boot.txt connects the executing original MCU
sender to the native DSP ROM under an explicit pad-order/interleaving experiment.
Measured timing and complete scheduled MCU/DSP board boot remain pending.
doc/dsp-validation.txt covers NORM/accumulator-transfer corrections, native VSL,
independent instruction vectors and pinned external MCPX silicon captures, with
derivative limits. doc/dsp-instructions.txt covers the bounded native instruction
fixture, independent execution-control regressions and opt-in fuzz harness.
doc/dsp-zero-loops.txt covers normal zero REP annulment, retained
zero-DO frames, source/stack/IRQ checks and the remaining compatibility limits.
doc/nand-read.txt covers an explicit small-page NAND read candidate, native own
GPIO reads and isolated original addressing/turnaround frontiers. It does not
establish the target boards' NAND contents or complete original NAND loading.
doc/nand-write.txt covers explicit program/erase outcomes, interrupted-cell
knowledge, program limits, native status/failure/readback and unchanged updater
helper checks. doc/dsp-timers.txt includes explicit mode-six capture and native IRQ.
doc/nand-persistence.txt describes private NAND images, durable write-ahead
projections, fail-stop/reload and native original-helper process-death checks.
Automatic CPU/peripheral time and complete interrupt latency remain unimplemented.
The original MCU bootloader reaches its application entry in the explicit
untimed fixture described in doc/mcu-boot.txt; this does not establish a full boot.
The native HC12 description is patched and recompiled locally for independently
verified branch, indexed-word and exception corrections; no installed Ghidra files are modified.

The execution command must stop visibly on unsupported hardware. An instruction
budget being exhausted does not establish successful boot. Downloading firmware
and extracting its contents does not exercise the real update/recovery path.

Offline parser fuzzing
Run fuzz tests only at the user's explicit request, never as an automatic part of
routine development, validation or quality checks. Manual example:

  cmake --preset fuzz
  cmake --build build/fuzz --target firmware-fuzz
  mkdir -p doc/local/fuzz-corpus doc/local/fuzz-artifacts
  cp doc/Blofeld_Firmware_V1.25.zip doc/local/fuzz-corpus/
  build/fuzz/firmware-fuzz doc/local/fuzz-corpus -artifact_prefix=doc/local/fuzz-artifacts/ -max_len=262144 -max_total_time=60

No fuzzing path opens physical MIDI devices. Preserve minimized regressions for
findings; keep proprietary seeds and derived corpus members under doc/local/.

See AGENTS.md for the binding development rules and THIRD_PARTY.txt for licenses.

The separate mcu-model-fuzz target exercises mapping, pin drives, unknown-byte
tracking, PHY/SCI clocks, accesses and reset sequences using synthetic bytes:
  cmake --build build/fuzz --target mcu-model-fuzz
  mkdir -p doc/local/mcu-fuzz-corpus doc/local/mcu-fuzz-artifacts
  build/fuzz/mcu-model-fuzz doc/local/mcu-fuzz-corpus -artifact_prefix=doc/local/mcu-fuzz-artifacts/ -max_len=1536 -max_total_time=60

The separate nand-image-fuzz target checks untrusted sparse NAND images, CRC-aware
semantic mutations and cold model reconstruction with own generated inputs:
  cmake --build build/fuzz --target nand-image-fuzz
  python3 tools/make_nand_image_fuzz_seeds.py doc/local/nand-image-fuzz-corpus
  mkdir -p doc/local/nand-image-fuzz-artifacts
  build/fuzz/nand-image-fuzz doc/local/nand-image-fuzz-corpus -artifact_prefix=doc/local/nand-image-fuzz-artifacts/ -max_len=65536 -max_total_time=60
