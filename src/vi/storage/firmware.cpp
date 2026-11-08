// SPDX-License-Identifier: GPL-3.0-only
#include "storage/firmware.hpp"

#include "storage/update_reader.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

namespace blofeld::vi {
namespace {

using update::Bytes;
using update::input_limit;
using update::Reader;

constexpr unsigned manufacturer = 0x3e;
constexpr unsigned device = 0x13;
constexpr unsigned device_broadcast = 0x7f;
constexpr unsigned header_command = 0x73;
constexpr unsigned data_command = 0x74;
constexpr std::uint32_t dsp_word_mask = 0xffffff;
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

struct Block {
  unsigned mode = 0;
  std::uint32_t length = 0;
  std::uint32_t address = 0;
  unsigned packets = 0;
  Bytes packed;
};

std::uint32_t septets(const std::span<const std::uint8_t> data) {
  std::uint32_t value = 0;
  unsigned shift = 0;
  for (const auto byte : data) {
    value |= static_cast<std::uint32_t>(byte) << shift;
    shift += 7;
  }
  return value;
}

std::vector<Block> parse_blocks(const std::vector<Bytes> &messages) {
  std::vector<Block> result;
  for (const auto &message : messages) {
    if (message.size() < 5 || message[0] != manufacturer || message[1] != device ||
        message[2] != device_broadcast) {
      throw std::runtime_error("Unexpected Blofeld firmware message");
    }
    if (message[3] == header_command && message.size() == 14) {
      const auto data = std::span(message);
      const auto length = septets(data.subspan(5, 4));
      if (length > input_limit || length % 3 != 0) {
        throw std::runtime_error("Invalid firmware block length");
      }
      result.push_back({.mode = message[4],
                        .length = length,
                        .address = septets(data.subspan(9, 4)),
                        .packets = 0,
                        .packed = {}});
    } else if (message[3] == data_command && message.size() >= 8 && !result.empty()) {
      auto &block = result.back();
      if (message.size() != message[5] + 8U || message[4] != block.packets % 128U) {
        throw std::runtime_error("Firmware packet size or sequence mismatch");
      }
      std::uint8_t checksum = 0;
      for (std::size_t index = 0; index + 1 < message.size(); ++index) {
        checksum ^= message[index];
      }
      if (checksum != 0) { throw std::runtime_error("Firmware packet XOR mismatch"); }
      block.packed.insert(block.packed.end(), message.begin() + 6, message.end() - 2);
      ++block.packets;
    } else {
      throw std::runtime_error("Unsupported firmware message command");
    }
  }
  return result;
}

Bytes unpack(const Block &block) {
  if (block.packed.size() % 8 != 0 ||
      block.packed.size() / 8 * 7 != ((static_cast<std::size_t>(block.length) + 6) / 7) * 7) {
    throw std::runtime_error("Firmware block does not match its declared length");
  }
  Bytes little;
  little.reserve(block.packed.size() / 8 * 7);
  for (std::size_t offset = 0; offset < block.packed.size(); offset += 8) {
    const auto flags = block.packed[offset];
    for (unsigned index = 0; index < 7; ++index) {
      little.push_back(static_cast<std::uint8_t>(block.packed[offset + index + 1] |
                                                 (((flags >> (6U - index)) & 1U) << 7U)));
    }
  }
  return little;
}

void reverse_words(Bytes &bytes) {
  if (bytes.size() % 3 != 0) { throw std::runtime_error("Incomplete firmware transport word"); }
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    std::swap(bytes[offset], bytes[offset + 2]);
  }
}

Bytes decode(const Block &block) {
  auto bytes = unpack(block);
  bytes.resize(block.length);
  reverse_words(bytes);
  return bytes;
}

void verify24(const std::span<const std::uint8_t> data) {
  if (data.size() % 3 != 0) { throw std::runtime_error("Incomplete 24-bit checksum words"); }
  Reader reader(data);
  std::uint32_t sum = 0;
  while (!reader.empty()) { sum = (sum + reader.big(3)) & dsp_word_mask; }
  if (sum != 0) { throw std::runtime_error("Firmware object checksum mismatch"); }
}

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
  Reader dsp(main.subspan(dsp_begin));
  std::size_t bytes = 0;
  while (!dsp.empty()) {
    const auto space = dsp.big(3);
    const auto count = dsp.big(3);
    const auto address = dsp.big(3);
    bytes += 9;
    if (space == 0x100 && count == 0x100 && address == 0x100) {
      dsp.big(3);
      bytes += 3;
      verify24(main.subspan(dsp_begin, bytes));
      verify_dsp_tail(dsp);
      return dsp_begin;
    }
    if (space > 2 || count == 0 || address > dsp_word_mask || count > 0x1000000U - address) {
      throw std::runtime_error("Invalid DSP memory record");
    }
    const auto contents = dsp.take(static_cast<std::size_t>(count) * 3);
    bytes += contents.size();
  }
  throw std::runtime_error("Missing DSP end marker");
}

} // namespace

FirmwareUpdate FirmwareUpdate::from_midi(const std::span<const std::uint8_t> midi) {
  if (midi.size() > input_limit) { throw std::runtime_error("Firmware input exceeds size limit"); }
  FirmwareUpdate result;
  const auto parts = parse_blocks(update::sysex_messages(midi));
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
    result.blocks[index] = decode(parts[index]);
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
  auto transport = unpack(parts[4]);
  // The word-oriented main transfer can supply real padding after its declared
  // container. Preserve complete words rather than inventing their values.
  transport.resize((transport.size() / 3) * 3);
  reverse_words(transport);
  if (nand_begin > transport.size()) {
    throw std::runtime_error("Runtime NAND source exceeds transport");
  }
  const auto nand = std::span(transport).subspan(nand_begin);
  result.nand_program.assign(nand.begin(), nand.end());
  return result;
}

} // namespace blofeld
