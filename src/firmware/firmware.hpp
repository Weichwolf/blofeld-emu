// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

namespace blofeld {

struct DspRecord {
  std::uint32_t space = 0;
  std::uint32_t address = 0;
  std::vector<std::uint32_t> words;
};

struct Firmware {
  std::vector<std::uint8_t> mcu;
  std::vector<DspRecord> dsp;
  std::vector<std::vector<std::uint8_t>> update_blocks;
  std::size_t sysex_packets = 0;
  static Firmware load(const std::filesystem::path &path);
  static Firmware parse(const std::vector<std::uint8_t> &bytes);
  static Firmware parse_midi(const std::vector<std::uint8_t> &midi);
};

} // namespace blofeld
