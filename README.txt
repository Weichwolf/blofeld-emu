Blofeld firmware emulator — initial headless bring-up

Objective
Execute original firmware 1.25 for Desktop and Keyboard, with a faithful board
model, MIDI/audio and reproducible firmware-development tests. The complete
acceptance scope and current gaps are in doc/coverage.txt. This is not yet a
booting Blofeld, a playable synthesizer or a verified replacement-firmware target.

Build and quality checks
Requirements: LLVM 19 (clang, clang-tidy, clang-format and sanitizer runtime),
CMake >= 3.25, Ninja, GNU patch, Python 3, zlib development files and Ghidra 12.1.2 including
its native SLEIGH sources. Set GHIDRA_ROOT if it is not installed under
~/.local/opt/ghidra_*_PUBLIC. Native SLEIGH does not need Java at runtime.

  cmake --preset strict
  cmake --build --preset strict
  cmake --build build/strict --target quality

clang-tidy runs as a mandatory build step on every project C++ translation unit;
format-check checks formatting without changing files. All diagnostics are
errors, including the 120-line function limit. ASan/UBSan and additional integer
checks are enabled in strict builds; dependencies also receive ASan/UBSan.
release retains warning/static-analysis gates and disables sanitizers for timing
measurements. thread selects separate TSan instrumentation. No concurrency is
currently implemented. Native SLEIGH requires exceptions; any future audio
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

The execution command must stop visibly on unsupported hardware. An instruction
budget being exhausted does not establish successful boot. Downloading firmware
and extracting its contents does not exercise the real update/recovery path.

Offline parser fuzzing
  cmake --preset fuzz
  cmake --build build/fuzz --target firmware-fuzz
  mkdir -p doc/local/fuzz-corpus doc/local/fuzz-artifacts
  cp doc/Blofeld_Firmware_V1.25.zip doc/local/fuzz-corpus/
  build/fuzz/firmware-fuzz doc/local/fuzz-corpus -artifact_prefix=doc/local/fuzz-artifacts/ -max_len=262144 -max_total_time=60

No fuzzing path opens physical MIDI devices. Preserve minimized regressions for
findings; keep proprietary seeds and derived corpus members under doc/local/.

See AGENTS.md for the binding development rules and THIRD_PARTY.txt for licenses.
