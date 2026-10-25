// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_native_access.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <variant>

namespace blofeld {
enum class DspReferenceExecution : std::uint8_t {
  unqualified,
  access_boundaries,
  access_and_retirement_boundaries,
  address_and_retirement_boundaries,
  staged_register_boundaries
};

struct DspPeripheralReset {};

struct DspInterruptAcknowledge {
  DspInterruptRequest request;
};

// Actual completed native result awaiting publication; not a bus operation.
struct DspInstructionRetire {
  std::uint32_t pc = 0;
  std::uint32_t next_pc = 0;
  std::uint32_t omr = 0;
  std::uint32_t next_omr = 0;
  bool redirected = false;
  bool operator==(const DspInstructionRetire &) const = default;
};

// Genuine native address-register update, held before its actual data access.
struct DspAddressUpdate {
  unsigned index = 0;
  std::uint32_t before = 0;
  std::uint32_t value = 0;
  DspSpace space = DspSpace::p;
  std::uint32_t address = 0;
  DspMemoryOperation operation = DspMemoryOperation::read;
  std::uint32_t pc = 0;
  std::uint32_t omr = 0;
  bool operator==(const DspAddressUpdate &) const = default;
};

// X/Y: low/high24; A/B: low24/middle24/extension8; R/N/M: register index.
struct DspMoveFields {
  std::uint8_t x = 0, y = 0, a = 0, b = 0, r = 0, n = 0, m = 0;
  bool operator==(const DspMoveFields &) const = default;
};

// Actual native move result awaiting field-selective publication, not bus I/O.
struct DspMoveWriteback {
  DspMoveFields fields;
  std::uint32_t opcode = 0;
  std::uint32_t pc = 0;
  std::uint32_t omr = 0;
  bool operator==(const DspMoveWriteback &) const = default;
};

using DspReferenceEffect = std::variant<DspNativeAccess,
                                        DspPeripheralReset,
                                        DspInterruptAcknowledge,
                                        DspInstructionRetire,
                                        DspAddressUpdate,
                                        DspMoveWriteback>;

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
    bool address = false;
    bool move = false;
    bool previous = false;
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
  DspReferenceExecution execution = DspReferenceExecution::unqualified;
};

// Semantic access boundaries only; no clock, pipeline-stage or allocation bound.
// begin may retire a loaded REP body with no memory access. Admitting a ticket
// performs that effect once and computes until the next access or retirement.
// Explicit interrupt entry retires its fast/long vector operation, possibly two
// native instructions, without executing a foreground instruction.
// access_and_retirement_boundaries additionally holds the completed native CPU
// result after all admitted effects until its distinct retirement ticket.
struct DspReferenceProgress {
  std::optional<DspReferenceEffect> completed = std::nullopt;
  std::optional<DspReferencePending> pending = std::nullopt;
  bool retired = false;
};

// One explicitly supplied DMA arbitration sample against an actual pending word
// effect. No pipeline timestamp is inferred. A stalled core has no new progress.
struct DspReferenceDmaProgress {
  DspDmaBusResult dma;
  std::optional<DspReferenceProgress> core = std::nullopt;
};
}
