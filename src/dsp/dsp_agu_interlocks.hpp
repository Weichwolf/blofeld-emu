// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>

namespace blofeld {

struct DspAguEffect {
  unsigned writes = 0;
  unsigned stalls = 0;
};

// FM A.2.4 MOVE-to-address-generation interlocks. This is a nominal history,
// not physical stage placement, instruction fetch arbitration or ALU timing.
class DspAguInterlocks {
public:
  static constexpr unsigned maximum_stall_units = 3;
  [[nodiscard]] DspAguEffect preview(std::uint32_t opcode) const;
  void complete(DspAguEffect effect, unsigned native_units);

private:
  std::array<unsigned, 8> remaining_{};
};

}
