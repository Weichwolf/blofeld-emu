// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_program_fetch.hpp"

#include <cstdint>
#include <optional>

namespace blofeld {
struct DspBranchForm {
  unsigned bit = 0;
  bool set = false;
  // A signed short displacement is retained as a sign-extended 24-bit word.
  std::optional<std::uint32_t> displacement = std::nullopt;
  std::optional<std::uint8_t> address_register = std::nullopt;
  bool absolute_extension = false;
  bool relative_extension = false;
  bool subroutine_call = false;
  bool subroutine_return = false;

  [[nodiscard]] unsigned words() const {
    return displacement || address_register || subroutine_return ? 1U : 2U;
  }
};

// An early frontend decision, never an architectural CPU result or bus response.
struct DspBranchDecision {
  DspProgramWord word;
  std::optional<DspProgramWord> extension = std::nullopt;
  std::uint32_t omr = 0;
  std::uint32_t next_pc = 0;
  bool taken = false;
  bool operator==(const DspBranchDecision &) const = default;
};

[[nodiscard]] std::optional<DspBranchForm> decode_branch_form(std::uint32_t opcode);
}
