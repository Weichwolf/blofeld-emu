// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_interrupts.hpp"
#include "blofeld/dsp_native_access.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <variant>

namespace blofeld {
enum class DspReferenceExecution : std::uint8_t { unqualified, access_boundaries };

struct DspPeripheralReset {};

struct DspInterruptAcknowledge {
  DspInterruptRequest request;
};

using DspReferenceEffect =
    std::variant<DspNativeAccess, DspPeripheralReset, DspInterruptAcknowledge>;

// Trusted owner identity, invalidated by reset/restore. Not a serialized grant.
class DspReferenceTicket {
public:
  DspReferenceTicket() = default;
  bool operator==(const DspReferenceTicket &) const = default;

private:
  friend class DspCore;

  struct Position {
    std::uint64_t generation = 0;
    std::size_t record = 0;
    bool operator==(const Position &) const = default;
  };

  DspReferenceTicket(std::shared_ptr<const bool> owner, const Position position)
      : owner_(std::move(owner)), position_(position) {}

  std::shared_ptr<const bool> owner_;
  Position position_;
};

struct DspReferencePending {
  DspReferenceEffect effect;
  DspReferenceTicket ticket;
  // Actual P-bus read already captured; admission must not read it again.
  std::optional<std::uint32_t> latched_program_word = std::nullopt;
};

// Semantic access boundaries only; no clock, pipeline-stage or allocation bound.
// begin may retire a loaded REP body with no memory access. Admitting a ticket
// performs that effect once and computes until the next access or retirement.
// Explicit interrupt entry retires its fast/long vector operation, possibly two
// native instructions, without executing a foreground instruction.
struct DspReferenceProgress {
  std::optional<DspReferenceEffect> completed = std::nullopt;
  std::optional<DspReferencePending> pending = std::nullopt;
  bool retired = false;
};
}
