# Project context

- Speak German with the user. Code, comments, documentation and commit messages are English.
- The primary objective is a complete Blofeld Desktop and Keyboard hardware emulator, initially headless and standalone. Unmodified original 1.25 firmware is the known-working compatibility reference; tests verify our emulator, not the manufacturer's firmware.
- Implement the connected device from actual reset through normal instrument operation with original DSP audio, MIDI and persistent storage. Isolated routine fixtures are supporting evidence, never a substitute for that runtime. Alternative-firmware development is a secondary future use.
- Physical disassembly and measurement are unavailable and are not acceptance prerequisites. Independently reconstruct missing hardware through documented constraints and reproducible offline hypotheses/experiments without a preset attempt limit. Missing information remains a research task; completion requires correctly running original firmware and every documented function on both profiles.
- Do not replace original MCU or DSP behavior with a host synthesizer, firmware patches, skipped routines, fabricated success responses or silently successful peripheral stubs.
- Unknown or unsupported hardware accesses must fail visibly with address, operation, PC and execution context. Keep observed behavior, hypotheses and hardware-verified facts distinct.
- Device detection is not hardware validation. Hardware update/programming tests require a separately established recovery procedure; never flash experimental firmware as a routine test.
- Keep the emulator, board model, transport and time-critical audio processing separate. Run as a regular user.
- Query audio and MIDI host capabilities independently and read back actual settings and buffer/pool grants. Adapt bounded buffering and packetization to those limits; report unsupported configurations, overload and disconnects. Development-host limits are not global hardware limits, and host differences must not change emulated clocks.

# Quality gates

- Apply the strict quality practices of `~/Git/octa-capture`: warnings as errors, strict compiler/linker flags, clang-format, clang-tidy with zero findings, and a maximum 120 lines per function.
- Use C++23, CMake/Ninja and LLVM 19. Instrument our code and native dependencies with ASan/UBSan; run TSan separately once concurrent code exists.
- Never weaken checks, add diagnostic baselines or add NOLINT markers to hide findings. Third-party source retains its notices and is analyzed separately from project-owned code.
- Scope validation to the affected behavior. Do not rerun the entire suite after small changes. Run a broader regression once after a coherent CPU/DSP, memory-map or scheduler integration when justified. Avoid duplicate runs across identical source in separate worktrees; verify source identity and evidence scope before reusing results.
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
- Keep `/home/cosmo/Git/blofeld-emu` on `main` and synchronize it after every verified publication. Continue active work in this canonical checkout; preserve open source changes and local references before removing unused worktrees.
- Push only reviewed source commits to upstream/main. Never push local/pre-publication-* backup branches or use --all/--mirror.
- Keep detailed manufacturer disassembly analysis under ignored doc/local/. Public tests pin reference hashes instead of embedding manufacturer program bytes.
- All GitHub issue communication belongs in private Weichwolf/blofeld-src, including emulator interfaces, test summaries and source findings. Do not create or comment on blofeld-emu issues. Existing emulator issues are migrated with their history before the originals are removed. Raw disassembly and reference assets retain their local-only rules.
