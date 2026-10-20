// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"

#include <cstdint>
#include <functional>
#include <optional>

namespace blofeld {
enum class DspNativeAccessKind : std::uint8_t { opcode, extension, read, write, bit, vector };
enum class DspNativeAccessStage : std::uint8_t { before, after };

struct DspNativeAccessBit {
  unsigned index = 0;
  DspBitOperation operation = DspBitOperation::test;
  bool operator==(const DspNativeAccessBit &) const = default;
};

// Semantic interpreter order, not a hardware cycle or pipeline-stage timestamp.
// A read value exists only after the strict access; writes carry their source
// value in both stages. Bit values are the previous bit (0/1), after access only.
struct DspNativeAccess {
  DspNativeAccessKind kind = DspNativeAccessKind::read;
  DspNativeAccessStage stage = DspNativeAccessStage::before;
  DspAddress target{};
  std::optional<std::uint32_t> value = std::nullopt;
  std::optional<DspNativeAccessBit> bit = std::nullopt;
  std::uint64_t ordinal = 0;
  std::uint32_t pc = 0;
  std::uint32_t omr = 0;
};

// Callbacks observe only; they must not mutate native CPU, clock or board state.
// Copied for one step, never checkpointed or retained after it. Callback errors
// quarantine native execution; committed bus side effects require trusted restore.
// No callback is made for decoder lookahead, cached REP words or timing previews.
struct DspNativeAccessObserver {
  std::function<void(const DspNativeAccess &)> observe;
};
}
