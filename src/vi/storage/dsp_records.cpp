// SPDX-License-Identifier: GPL-3.0-only
#include "storage/dsp_records.hpp"

#include "storage/update_reader.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

namespace blofeld::vi {
DspRecords DspRecords::read(const std::span<const std::uint8_t> input) {
  if (input.size() > update::input_limit) {
    throw std::runtime_error("DSP record input exceeds size limit");
  }
  update::Reader reader(input);
  DspRecords result;
  constexpr std::uint32_t end_word = 0x100;
  constexpr std::uint32_t address_words = 0x1000000;
  while (!reader.empty()) {
    const auto offset = input.size() - reader.rest().size();
    const auto space = reader.big(3);
    const auto count = reader.big(3);
    const auto address = reader.big(3);
    if (space == end_word && count == end_word && address == end_word) {
      reader.big(3);
      result.bytes_ = input.size() - reader.rest().size();
      update::verify24(input.first(result.bytes_));
      return result;
    }
    if (space > 2 || count == 0 || count > address_words - address) {
      throw std::runtime_error("Invalid DSP memory record at byte " + std::to_string(offset));
    }
    result.records_.push_back({.space = space,
                               .address = address,
                               .byte_offset = offset,
                               .payload = reader.take(static_cast<std::size_t>(count) * 3)});
  }
  throw std::runtime_error("Missing DSP end marker");
}
}
