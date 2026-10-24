// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_program_fetch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>

namespace blofeld {
struct DspReferenceYield {};

struct DspReferenceRecord {
  DspReferenceEffect effect;
  std::uint32_t value = 0;
};

struct DspReferenceLookahead {
  std::uint32_t address = 0;
  std::uint32_t value = 0;
};

using DspReferenceEntry = std::variant<DspReferenceRecord, DspReferenceLookahead>;
inline constexpr std::size_t dsp_reference_capacity = 64;

struct DspReferenceInterrupt {
  DspInterruptRequest request;
  std::uint32_t address = 0;
};

struct DspReferenceState {
  std::array<DspReferenceEntry, dsp_reference_capacity> entries{};
  std::size_t used = 0;
  std::optional<DspReferenceEffect> pending = std::nullopt;
  std::optional<std::uint32_t> latched_program_word = std::nullopt;
  DspNativeAccess context{};
  std::optional<DspReferenceInterrupt> interrupt;
  bool active = false;
  DspReferenceExecution execution = DspReferenceExecution::unqualified;
  DspProgramFetch fetch;
};

enum class DspReferenceMode : std::uint8_t { ordinary, attempt, restoring };

// Fixed-capacity admitted effects. Native arithmetic may recompute, real bus I/O
// never replays. Decoder lookahead is recorded metadata, never an admitted access.
class DspReferenceTape {
public:
  explicit DspReferenceTape(DspBus &bus) : bus_(bus) {}

  void begin(DspNativeAccess context,
             DspReferenceExecution execution,
             std::optional<DspReferenceInterrupt> interrupt = std::nullopt);
  void clear();
  void finish();
  void rewind();
  void verify_complete() const;
  void stage_retirement(DspInstructionRetire result);

  [[nodiscard]] DspReferenceExecution execution() const { return state_.execution; }

  [[nodiscard]] bool active() const { return state_.active; }

  [[nodiscard]] bool intercepting() const { return mode_ != DspReferenceMode::ordinary; }

  [[nodiscard]] DspReferenceMode mode() const { return mode_; }

  [[nodiscard]] const std::optional<DspReferenceInterrupt> &interrupt() const {
    return state_.interrupt;
  }

  void set_mode(DspReferenceMode mode) noexcept { mode_ = mode; }

  [[nodiscard]] DspReferenceState snapshot() const { return state_; }

  void restore(const DspReferenceState &state);

  [[nodiscard]] const std::optional<DspReferenceEffect> &pending() const { return state_.pending; }

  [[nodiscard]] std::size_t pending_index() const { return state_.used; }

  [[nodiscard]] std::optional<std::uint32_t> latched_program_word() const;

  [[nodiscard]] DspProgramFetch &fetch() { return state_.fetch; }

  [[nodiscard]] const DspProgramFetch &fetch() const { return state_.fetch; }

  void check_admission() const;

  [[nodiscard]] const DspReferenceEffect &check_program_latch() const;
  void latch_program();

  [[nodiscard]] DspReferenceEffect commit();
  [[nodiscard]] std::uint32_t read(DspAddress target, DspNativeAccessKind kind);
  void write(DspStore target);
  [[nodiscard]] bool bit(DspBitAccess target);
  void reset_peripherals();
  void acknowledge_interrupt(DspInterruptRequest request);
  [[nodiscard]] std::uint32_t peek_program(std::uint32_t address);

private:
  [[nodiscard]] std::uint32_t sample(const DspReferenceEffect &effect);
  [[nodiscard]] std::uint32_t perform(const DspReferenceEffect &effect);
  [[nodiscard]] DspNativeAccess decorate(DspNativeAccess access);
  void check_capacity() const;
  DspBus &bus_;
  DspReferenceState state_;
  std::size_t cursor_ = 0;
  std::uint64_t ordinal_ = 0;
  DspReferenceMode mode_ = DspReferenceMode::ordinary;
};

class DspReferenceScope {
public:
  DspReferenceScope(DspReferenceTape &tape, DspReferenceMode mode);
  ~DspReferenceScope();
  DspReferenceScope(const DspReferenceScope &) = delete;
  DspReferenceScope &operator=(const DspReferenceScope &) = delete;

private:
  DspReferenceTape &tape_;
  DspReferenceMode previous_;
};
}
