// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace blofeld::detail {
struct Hc12DecimalInput {
  std::uint8_t a;
  bool carry;
  bool half;
};

struct Hc12DecimalResult {
  std::uint8_t a;
  bool carry;
};

[[nodiscard]] Hc12DecimalResult hc12_decimal_adjust(Hc12DecimalInput input);
}
