// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace blofeld {
enum class NandBusContract : std::uint8_t { ordered_pins, functional_transactions };

// Functional shared-bus encoding, not an assertion about electrical CE wiring.
// A full GPIO output byte owns the host bus and pauses NAND independently
// of retained strobe bits. NAND resumes only at inactive controls after release.
// Both-low neutral remains supported; unfinished NAND writes still refuse.
class NandBusDecode {
public:
  explicit NandBusDecode(NandBusContract contract);
  [[nodiscard]] unsigned observe(unsigned controls, bool host_outputs = false);
  [[nodiscard]] NandBusContract contract() const;
  [[nodiscard]] bool neutral() const;
  bool operator==(const NandBusDecode &) const = default;

private:
  NandBusContract contract_;
  std::optional<unsigned> previous_;
  bool host_owned_ = false;
};
}
