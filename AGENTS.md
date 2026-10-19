# Project context

- Speak German with the user. Code, comments, documentation and commit messages are English.
- The active objective is a headless emulator executing unmodified Blofeld 1.25 firmware on Desktop and Keyboard profiles, with MIDI/audio and extensive boot, persistence, update, recovery and failure tests.
- This is a firmware development test target. Do not replace original MCU or DSP behavior with a host synthesizer or silently successful peripheral stubs.
- Unknown or unsupported hardware accesses must fail visibly with address, operation, PC and execution context. Keep observed behavior, hypotheses and hardware-verified facts distinct.
- Device detection is not hardware validation. Hardware update/programming tests require a separately established recovery procedure; never flash experimental firmware as a routine test.
- Keep the emulator, board model, transport and time-critical audio processing separate. Run as a regular user.

# Quality gates

- Apply the strict quality practices of `~/Git/octa-capture`: warnings as errors, strict compiler/linker flags, clang-format, clang-tidy with zero findings, and a maximum 120 lines per function.
- Use C++23, CMake/Ninja and LLVM 19. Instrument our code and native dependencies with ASan/UBSan; run TSan separately once concurrent code exists.
- Never weaken checks, add diagnostic baselines or add NOLINT markers to hide findings. Third-party source retains its notices and is analyzed separately from project-owned code.
- Native SLEIGH uses exceptions as its error interface. This dependency requirement must stay explicit; no exceptions or allocations belong in a future real-time audio callback.
- Run fuzz tests only when the user explicitly requests them; never run them automatically as part of routine development, validation or quality checks. When requested, fuzz untrusted firmware, MIDI and state parsers offline under sanitizers. Preserve a regression input for every finding. Never send malformed fuzz traffic to physical devices.
- Test CPU flags, addressing and state changes against independent instruction expectations and recovered firmware behavior. Round trips and self-consistency alone are insufficient.
- Recheck current binaries, test coverage and connected hardware before making claims. Do not label absent or untested requirements as complete.
- Commit verified increments. Push to `upstream` only when that remote is configured; never invent a publication destination.

# References and records

- Official firmware, samples, product images, extracted assets and disassemblies remain local references and must not be committed or redistributed with the implementation.
- Keep download URLs, sizes and hashes in tracked manifests. Private analysis and run artifacts belong in ignored `doc/local/`.
- Keep documentation concise: reproducible commands, protocol facts, evidence, material limitations and outstanding requirements. Do not maintain an activity diary.
- Maintain the complete objective and a requirement-by-requirement coverage record. Passing parser tests does not establish boot, MIDI/audio, flash safety or real-device equivalence.
- Original project code is GPL-3.0-only; add matching SPDX identifiers to new source files.
- The DSP dependency is GPL-3.0. Preserve dependency licenses; do not copy OCTA-CAPTURE's GPL-2.0-only implementation into this combined emulator.

# Publication

- The public repository is Weichwolf/blofeld-emu; its original implementation uses GPL-3.0-only.
- Push only reviewed source commits to upstream/main. Never push local/pre-publication-* backup branches or use --all/--mirror.
- Keep detailed manufacturer disassembly analysis under ignored doc/local/. Public tests pin reference hashes instead of embedding manufacturer program bytes.
- Cross-repository coordination through GitHub issues is authorized between Weichwolf/blofeld-emu and private Weichwolf/blofeld-src. Use public issues for emulator interfaces and test summaries, and private source issues for source-specific findings. Raw disassembly and reference assets retain their local-only rules.
