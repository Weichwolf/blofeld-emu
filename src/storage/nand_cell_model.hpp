// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "storage/nand_array.hpp"

#include <cstdint>
#include <map>
#include <optional>

namespace blofeld {
enum class NandCellContract : std::uint8_t { unqualified, reference_pulse_inhibit };

struct NandCellInhibit {
  std::uint8_t program = 0;
  std::uint8_t erase = 0;
};

struct NandCellProfile {
  NandCellContract contract = NandCellContract::unqualified;
  // Physical pulse-inhibit properties, not preselected command outcomes.
  std::map<unsigned, NandCellInhibit> cells;
};

struct NandCellResult {
  NandByte after;
  std::optional<bool> verified;
};

class NandCellModel {
public:
  explicit NandCellModel(NandCellProfile profile);
  [[nodiscard]] NandCellInhibit inhibit(NandLocation location) const;
  [[nodiscard]] static NandCellResult
  program(NandByte before, std::uint8_t data, NandCellInhibit inhibit);
  [[nodiscard]] static NandCellResult erase(NandByte before, NandCellInhibit inhibit);

private:
  NandCellProfile profile_;
};
}
