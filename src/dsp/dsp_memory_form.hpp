// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace blofeld {
struct DspMemoryLane {
  DspSpace space = DspSpace::x;
  DspMemoryOperation operation = DspMemoryOperation::read;
  unsigned mode = 0;
  unsigned reg = 0;
  bool dual_encoding = false;
};

struct DspMemoryForm {
  std::array<DspMemoryLane, 2> lanes{};
  std::size_t lane_count = 1;
  bool long_pair = false;
  std::optional<DspAddress> peripheral = std::nullopt;
  bool peripheral_register = false;
  std::optional<DspAddress> absolute_short = std::nullopt;
  std::optional<DspBitAccess> peripheral_bit = std::nullopt;
};

[[nodiscard]] bool is_program_transfer(const DspMemoryForm &form);
[[nodiscard]] bool is_program_memory(const DspMemoryForm &form);
[[nodiscard]] bool full_accumulator_long_store(std::uint32_t opcode);
[[nodiscard]] bool full_accumulator_scalar_store(std::uint32_t opcode);
[[nodiscard]] bool partial_accumulator_scalar_store(std::uint32_t opcode);
// Raw A1/B1 peripheral stores; full accumulators and other parts stay separate.
[[nodiscard]] bool middle_accumulator_peripheral_store(std::uint32_t opcode);
// Ordinary raw X0/X1/Y0/Y1 and N0..N7 sources; no parallel ALU or control register.
[[nodiscard]] bool data_offset_scalar_store(std::uint32_t opcode);

[[nodiscard]] std::optional<DspMemoryForm> decode_memory_stage(std::uint32_t opcode);
}
