// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "runtime/instrument.hpp"
#include "storage/private_file.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld::runtime {
std::vector<std::uint32_t> load_rom(const std::filesystem::path &path) {
  const auto bytes = read_private_file(path, std::size_t{65536} * 3U);
  if (bytes.empty() || bytes.size() % 3 != 0) {
    throw Fault("DSP ROM requires nonempty packed big-endian 24-bit words");
  }
  std::vector<std::uint32_t> words;
  words.reserve(bytes.size() / 3);
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    constexpr unsigned high_shift = 16;
    words.push_back((static_cast<std::uint32_t>(bytes[offset]) << high_shift) |
                    (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) | bytes[offset + 2]);
  }
  return words;
}
}
