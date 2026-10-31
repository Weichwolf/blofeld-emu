// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace blofeld {
void Nand::observe_identification(const NandIdentification identification) {
  if (phase_ != Phase::cold) {
    fail("identification observation", "requires cold device configuration");
  }
  auto next = identification_;
  for (std::size_t index = 0; index < next.bytes.size(); ++index) {
    const auto value = identification.bytes[index];
    auto &before = next.bytes[index];
    if ((value.value & ~value.known) != 0) {
      fail("identification observation", "noncanonical byte=" + std::to_string(index));
    }
    if (((before.value ^ value.value) & before.known & value.known) != 0) {
      fail("identification observation", "conflicting immutable byte=" + std::to_string(index));
    }
    before.value |= value.value;
    before.known |= value.known;
  }
  identification_ = next;
}

void Nand::begin_identification() {
  if (phase_ == Phase::program_address || phase_ == Phase::program_data ||
      phase_ == Phase::erase_address) {
    fail("identification setup", "unqualified replacement of a staged write sequence");
  }
  phase_ = Phase::identification_address;
  address_cycles_ = 0;
  status_mode_ = false;
  identification_index_ = 0;
  // Read-ID is not an area-pointer command and does not establish an unknown pointer.
}

void Nand::address_identification(const std::uint8_t value) {
  if (value != 0 || ready_ != true || status_mode_) {
    fail("identification address", "requires the documented single address 00h");
  }
  phase_ = Phase::identification_data;
}

std::uint8_t Nand::read_identification(const std::uint8_t mask) {
  if (identification_index_ >= identification_.bytes.size()) {
    fail("identification read",
         "byte=" + std::to_string(identification_index_) +
             " exceeds the supplied identification window");
  }
  const auto sample = identification_.bytes.at(identification_index_);
  if ((mask & ~sample.known) != 0) {
    fail("identification read",
         "unknown bits byte=" + std::to_string(identification_index_) + " mask=" + hex(mask, 2));
  }
  ++identification_index_;
  return static_cast<std::uint8_t>(sample.value & mask);
}
} // namespace blofeld
