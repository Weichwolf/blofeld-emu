// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace blofeld::vi {
struct DspRecord {
  std::uint32_t space = 0, address = 0;
  std::size_t byte_offset = 0;
  std::span<const std::uint8_t> payload;
};

// Ordered P/X/Y source writes, not a preloaded device image or executable map.
// Payloads borrow the input bytes. Later records never erase earlier records.
class DspRecords {
public:
  [[nodiscard]] static DspRecords read(std::span<const std::uint8_t> input);

  [[nodiscard]] std::span<const DspRecord> records() const noexcept { return records_; }

  // Includes the original end header and checksum; subsequent input is untouched.
  [[nodiscard]] std::size_t bytes() const noexcept { return bytes_; }

private:
  std::vector<DspRecord> records_;
  std::size_t bytes_ = 0;
};
}
