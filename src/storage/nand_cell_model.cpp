// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_cell_model.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"

#include <cstdint>
#include <optional>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned byte_mask = 0xff;

std::optional<bool> verify(const NandByte cell, const unsigned selected, const bool ones) {
  const unsigned wrong = ones ? (byte_mask ^ cell.value) : cell.value;
  if ((wrong & cell.known & selected) != 0) { return false; }
  if ((selected & (byte_mask ^ cell.known)) != 0) { return std::nullopt; }
  return true;
}

void valid(const NandByte cell) {
  if ((cell.value & cell.known) != cell.value) {
    throw Fault("Noncanonical NAND cell transition input");
  }
}
}

NandCellModel::NandCellModel(NandCellProfile profile) : profile_(std::move(profile)) {
  if (profile_.contract != NandCellContract::reference_pulse_inhibit) {
    throw Fault("NAND cell transition model requires its explicit reference contract");
  }
  constexpr auto total = NandArray::pages * NandArray::page_bytes;
  for (const auto &[location, ignored] : profile_.cells) {
    static_cast<void>(ignored);
    if (location >= total) { throw Fault("NAND cell inhibit location exceeds the device"); }
  }
}

NandCellInhibit NandCellModel::inhibit(const NandLocation location) const {
  if (location.page >= NandArray::pages || location.column >= NandArray::page_bytes) {
    throw Fault("NAND cell inhibit query exceeds the device");
  }
  const auto key = (location.page * NandArray::page_bytes) + location.column;
  const auto found = profile_.cells.find(key);
  return found == profile_.cells.end() ? NandCellInhibit{} : found->second;
}

NandCellResult NandCellModel::program(const NandByte before,
                                      const std::uint8_t data,
                                      const NandCellInhibit inhibit) {
  valid(before);
  const unsigned zeros = byte_mask ^ data;
  const unsigned applied = zeros & (byte_mask ^ inhibit.program);
  const NandByte after{.value = static_cast<std::uint8_t>(before.value & (byte_mask ^ applied)),
                       .known = static_cast<std::uint8_t>(before.known | applied)};
  return {.after = after, .verified = verify(after, zeros, false)};
}

NandCellResult NandCellModel::erase(const NandByte before, const NandCellInhibit inhibit) {
  valid(before);
  const unsigned applied = byte_mask ^ inhibit.erase;
  const NandByte after{.value = static_cast<std::uint8_t>(before.value | applied),
                       .known = static_cast<std::uint8_t>(before.known | applied)};
  return {.after = after, .verified = verify(after, byte_mask, true)};
}
}
