// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <source_location>
#include <string>
#include <variant>

namespace blofeld::test::reference {
inline constexpr auto boundaries = DspReferenceExecution::access_boundaries;
inline constexpr unsigned first = 0x123456;
inline constexpr unsigned second = 0x234567;
inline constexpr unsigned third = 0x345678;
inline constexpr unsigned fifo = 0xffff94;
inline constexpr unsigned source = 0x660;
inline constexpr unsigned destination = 0x900;
inline constexpr unsigned long_jump = 0x0af080; // FM JMP ea: absolute extension form.

inline const DspNativeAccess &request(const DspReferenceProgress &progress) {
  if (!progress.pending) { throw Fault("Expected a pending native reference access"); }
  const auto *value = std::get_if<DspNativeAccess>(&progress.pending->effect);
  if (value == nullptr) { throw Fault("Expected a word/bit request, not a reset"); }
  return *value;
}

inline DspReferenceTicket ticket(const DspReferenceProgress &progress) {
  if (!progress.pending) { throw Fault("Expected an owner ticket before admission"); }
  return progress.pending->ticket;
}

inline DspReferenceTicket ticket(const DspMachine &machine) {
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("Expected a suspended native instruction"); }
  return pending->ticket;
}

inline const DspNativeAccess &completed(const DspReferenceProgress &progress) {
  if (!progress.completed) { throw Fault("Expected a completed native effect"); }
  const auto *value = std::get_if<DspNativeAccess>(&*progress.completed);
  if (value == nullptr) { throw Fault("Expected a completed word/bit access"); }
  return *value;
}

struct Expected {
  DspNativeAccessKind kind;
  DspAddress target;
  std::optional<unsigned> value = std::nullopt;
  std::uint64_t ordinal = 0;
  unsigned pc = boot;
};

inline void expect(const DspReferenceProgress &progress,
                   const Expected expected,
                   const std::source_location location = std::source_location::current()) {
  const auto &value = request(progress);
  const auto context = std::string(location.file_name()) + ':' + std::to_string(location.line());
  require(!progress.retired && value.stage == DspNativeAccessStage::before,
          context + ": expected a pending before-access boundary");
  require(value.kind == expected.kind, context + ": unexpected native access kind");
  require(value.target.space == expected.target.space &&
              value.target.address == expected.target.address,
          context + ": unexpected native access address " + hex(value.target.address, 6));
  require(value.value == expected.value, context + ": unexpected pending source value");
  require(value.ordinal == expected.ordinal,
          context + ": unexpected ordinal " + std::to_string(value.ordinal));
  require(value.pc == expected.pc, context + ": unexpected instruction PC " + hex(value.pc, 6));
  require(value.omr == reset_omr, context + ": unexpected OMR " + hex(value.omr, 6));
}

inline void send(DspShi &shi, const unsigned value) {
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

inline DspReferenceProgress finish(DspMachine &machine, DspReferenceProgress progress) {
  constexpr unsigned maximum_accesses = 16;
  for (unsigned budget = 0; !progress.retired && budget < maximum_accesses; ++budget) {
    progress = machine.advance_reference_access(ticket(progress));
  }
  require(progress.retired && !progress.pending, "The bounded own instruction actually retires");
  return progress;
}

inline void execute(DspMachine &machine) {
  static_cast<void>(finish(machine, machine.begin_reference_instruction(boundaries)));
}

void core_checks();
void dma_checks();
void interrupt_checks();
}
