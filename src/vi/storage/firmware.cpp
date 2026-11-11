// SPDX-License-Identifier: GPL-3.0-only
#include "storage/firmware.hpp"

#include "storage/dsp_records.hpp"
#include "storage/update_reader.hpp"
#include "storage/update_transport.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::vi {
namespace {

using update::Reader;
using update::verify24;

constexpr std::uint32_t validation_marker = 0x123123;
constexpr std::size_t flash_size = 0x8000;
constexpr std::size_t boot_size = 0x200;
constexpr std::size_t checksum_end = 0x7f00;
constexpr std::size_t checksum_offset = 0x7f10;
constexpr std::uint32_t checksum_mask = 0xffff;
constexpr std::size_t page_bytes = 528;
constexpr std::size_t page_count = 16;
constexpr std::size_t pages_checksum_end = 3 + (page_count * page_bytes) + 3;
constexpr std::uint32_t first_secondary_page = 0x380;
constexpr std::uint32_t helper_address = 0x2e00;
constexpr std::uint32_t initial_address = 0xef00;
constexpr std::uint32_t secondary_address = 0x10000;
constexpr std::uint32_t main_address = 0xd000;

std::size_t helper_end(const std::span<const std::uint8_t> data) {
  Reader reader(data);
  const auto length = static_cast<std::size_t>(reader.big(2));
  const std::size_t size = (((length + 4) / 3) * 3) + 3;
  if (size > data.size()) { throw std::runtime_error("HC12 helper exceeds its container"); }
  verify24(data.first(size));
  return size;
}

void verify_boot_checksum(const std::span<const std::uint8_t> host) {
  Reader checksum_words(host.subspan(boot_size, checksum_end - boot_size));
  std::uint32_t sum = 0;
  while (!checksum_words.empty()) { sum = (sum + checksum_words.big(2)) & checksum_mask; }
  Reader stored_checksum(host.subspan(checksum_offset));
  if (sum != stored_checksum.big(2)) {
    throw std::runtime_error("MCU application boot checksum mismatch");
  }
}

void verify_dsp_tail(Reader &dsp) {
  for (unsigned marker = 0; marker < 4; ++marker) {
    if (dsp.big(3) != 0x100) { throw std::runtime_error("Invalid firmware trailing marker"); }
  }
  if (dsp.big(3) != validation_marker) {
    throw std::runtime_error("Invalid firmware validation marker");
  }

  const auto tail = dsp.rest();
  if (helper_end(tail) != tail.size()) {
    throw std::runtime_error("Trailing data after firmware error helper");
  }
}

std::size_t extract_runtime(FirmwareUpdate &firmware) {
  const auto main = std::span(firmware.blocks[4]);
  const auto host_begin = helper_end(main);
  const auto dsp_begin = host_begin + (((flash_size + 2) / 3) * 3) + 3;
  if (dsp_begin > main.size()) { throw std::runtime_error("MCU image exceeds firmware container"); }
  verify24(main.subspan(host_begin, dsp_begin - host_begin));
  const auto host = main.subspan(host_begin, flash_size);
  std::ranges::copy(host, firmware.mcu_flash.begin());
  verify_boot_checksum(host);
  const auto records = DspRecords::read(main.subspan(dsp_begin));
  Reader tail(main.subspan(dsp_begin + records.bytes()));
  verify_dsp_tail(tail);
  return dsp_begin;
}

} // namespace

FirmwareUpdate FirmwareUpdate::from_midi(const std::span<const std::uint8_t> midi) {
  FirmwareUpdate result;
  const auto parts = update::decode_midi(midi);
  constexpr std::array<unsigned, 6> modes{3, 0, 3, 0, 3, 0};
  constexpr std::array<std::uint32_t, 6> addresses{initial_address,
                                                   helper_address,
                                                   secondary_address,
                                                   helper_address,
                                                   main_address,
                                                   helper_address};
  if (parts.size() != modes.size()) {
    throw std::runtime_error("Expected the six-block Blofeld 1.25 layout");
  }
  for (std::size_t index = 0; index < parts.size(); ++index) {
    if (parts[index].mode != modes[index] || parts[index].address != addresses[index]) {
      throw std::runtime_error("Unsupported firmware block mode or destination");
    }
  }
  for (std::size_t index = 0; index < parts.size(); ++index) {
    const auto bytes = std::span(parts[index].bytes).first(parts[index].declared_bytes);
    result.blocks[index].assign(bytes.begin(), bytes.end());
  }
  const auto initial = std::span(result.blocks[0]);
  if (helper_end(initial) != initial.size()) {
    throw std::runtime_error("Trailing initial helper data");
  }
  const auto secondary = std::span(result.blocks[2]);
  Reader page_header(secondary);
  if (page_header.big(3) != first_secondary_page || secondary.size() < pages_checksum_end) {
    throw std::runtime_error("Unsupported secondary flash page layout");
  }
  verify24(secondary.subspan(3, pages_checksum_end - 3));
  if (helper_end(secondary.subspan(pages_checksum_end)) != secondary.size() - pages_checksum_end) {
    throw std::runtime_error("Trailing secondary error helper data");
  }
  const auto nand_begin = extract_runtime(result);
  const auto transport = std::span(parts[4].bytes);
  if (nand_begin > transport.size()) {
    throw std::runtime_error("Runtime NAND source exceeds transport");
  }
  const auto nand = std::span(transport).subspan(nand_begin);
  result.nand_program.assign(nand.begin(), nand.end());
  return result;
}

} // namespace blofeld
