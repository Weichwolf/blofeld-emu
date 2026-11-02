// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
enum class NandBusContract : std::uint8_t { ordered_pins, functional_transactions };

// Functional shared-bus encoding, not an assertion about electrical CE wiring.
// Both-low strobes denote neutral. A read may release into this shared-bus
// phase; a write must finish, and the next transfer must start inactive.
class NandBusDecode {
public:
  explicit NandBusDecode(NandBusContract contract);
  [[nodiscard]] unsigned observe(unsigned controls);
  [[nodiscard]] NandBusContract contract() const;
  [[nodiscard]] bool neutral() const;
  bool operator==(const NandBusDecode &) const = default;

private:
  NandBusContract contract_;
  std::optional<unsigned> previous_;
};
}
