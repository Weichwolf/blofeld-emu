// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"

#include "blofeld/bus.hpp"
#include "input.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <span>
#include <utility>
#include <vector>

namespace blofeld {
namespace {

using input::Bytes;
using input::input_limit;
using input::Reader;

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

std::vector<Block> blocks(const std::vector<Bytes> &messages) {
  std::vector<Block> result;
  for (const auto &message : messages) {
    if (message.size() < 5 || message[0] != manufacturer || message[1] != device ||
        message[2] != device_broadcast) {
      throw Fault("Unexpected Blofeld firmware message");
    }
    if (message[3] == header_command && message.size() == 14) {
      const auto data = std::span(message);
      const auto length = septets(data.subspan(5, 4));
      if (length > input_limit || length % 3 != 0) { throw Fault("Invalid firmware block length"); }
      result.push_back({.mode = message[4],
                        .length = length,
                        .address = septets(data.subspan(9, 4)),
                        .packets = 0,
                        .packed = {}});
    } else if (message[3] == data_command && message.size() >= 8 && !result.empty()) {
      auto &block = result.back();
      if (message.size() != message[5] + 8U || message[4] != block.packets % 128U) {
        throw Fault("Firmware packet size or sequence mismatch");
      }
      std::uint8_t checksum = 0;
      for (std::size_t index = 0; index + 1 < message.size(); ++index) {
        checksum ^= message[index];
      }
      if (checksum != 0) { throw Fault("Firmware packet XOR mismatch"); }
      block.packed.insert(block.packed.end(), message.begin() + 6, message.end() - 2);
      ++block.packets;
    } else {
      throw Fault("Unsupported firmware message command");
    }
  }
  return result;
}

Bytes decode(const Block &block) {
  if (block.packed.size() % 8 != 0 ||
      block.packed.size() / 8 * 7 != ((static_cast<std::size_t>(block.length) + 6) / 7) * 7) {
    throw Fault("Firmware block does not match its declared length");
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
  little.resize(block.length);
  for (std::size_t offset = 0; offset < little.size(); offset += 3) {
    std::swap(little[offset], little[offset + 2]);
  }
  return little;
}

void verify24(const std::span<const std::uint8_t> data) {
  if (data.size() % 3 != 0) { throw Fault("Incomplete 24-bit checksum words"); }
  Reader reader(data);
  std::uint32_t sum = 0;
  while (!reader.empty()) { sum = (sum + reader.big(3)) & dsp_word_mask; }
  if (sum != 0) { throw Fault("Firmware object checksum mismatch"); }
}

std::size_t helper_end(const std::span<const std::uint8_t> data) {
  Reader reader(data);
  const auto length = static_cast<std::size_t>(reader.big(2));
  const std::size_t size = (((length + 4) / 3) * 3) + 3;
  if (size > data.size()) { throw Fault("HC12 helper exceeds its container"); }
  verify24(data.first(size));
  return size;
}

void verify_boot_checksum(const std::span<const std::uint8_t> host) {
  Reader checksum_words(host.subspan(boot_size, checksum_end - boot_size));
  std::uint32_t sum = 0;
  while (!checksum_words.empty()) { sum = (sum + checksum_words.big(2)) & checksum_mask; }
  Reader stored_checksum(host.subspan(checksum_offset));
  if (sum != stored_checksum.big(2)) { throw Fault("MCU application boot checksum mismatch"); }
}

void verify_dsp_tail(Reader &dsp) {
  for (unsigned marker = 0; marker < 4; ++marker) {
    if (dsp.big(3) != 0x100) { throw Fault("Invalid firmware trailing marker"); }
  }
  if (dsp.big(3) != validation_marker) { throw Fault("Invalid firmware validation marker"); }

  const auto tail = dsp.rest();
  if (helper_end(tail) != tail.size()) { throw Fault("Trailing data after firmware error helper"); }
}

void extract_runtime(Firmware &firmware) {
  if (firmware.update_blocks.size() != 6) { throw Fault("Expected six firmware update blocks"); }
  const auto main = std::span(firmware.update_blocks[4]);
  const auto host_begin = helper_end(main);
  const auto dsp_begin = host_begin + (((flash_size + 2) / 3) * 3) + 3;
  if (dsp_begin > main.size()) { throw Fault("MCU image exceeds firmware container"); }
  verify24(main.subspan(host_begin, dsp_begin - host_begin));
  const auto host = main.subspan(host_begin, flash_size);
  firmware.mcu.assign(host.begin(), host.end());
  verify_boot_checksum(host);
  Reader dsp(main.subspan(dsp_begin));
  std::size_t bytes = 0;
  for (std::size_t index = 0; index < 65536; ++index) {
    const auto space = dsp.big(3);
    const auto count = dsp.big(3);
    const auto address = dsp.big(3);
    bytes += 9;
    if (space == 0x100 && count == 0x100 && address == 0x100) {
      dsp.big(3);
      bytes += 3;
      verify24(main.subspan(dsp_begin, bytes));
      verify_dsp_tail(dsp);
      return;
    }
    if (space > 2 || count == 0 || address > dsp_word_mask || count > 0x1000000U - address) {
      throw Fault("Invalid DSP memory record");
    }
    DspRecord record{.space = space, .address = address, .words = {}};
    const auto contents = dsp.take(static_cast<std::size_t>(count) * 3);
    Reader words(contents);
    record.words.reserve(count);
    while (!words.empty()) { record.words.push_back(words.big(3)); }
    bytes += contents.size();
    firmware.dsp.push_back(std::move(record));
  }
  throw Fault("Missing DSP end marker");
}

} // namespace

Firmware Firmware::load(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) { throw Fault("Cannot open firmware: " + path.string()); }
  const auto size = file.tellg();
  if (size <= 0 || static_cast<std::uint64_t>(size) > input_limit) {
    throw Fault("Firmware file exceeds supported input size");
  }
  Bytes bytes(static_cast<std::size_t>(size));
  file.seekg(0);
  if (!file.read(reinterpret_cast<char *>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()))) {
    throw Fault("Cannot read complete firmware file");
  }
  return parse(bytes);
}

Firmware Firmware::parse(const Bytes &bytes) {
  if (bytes.size() > input_limit) { throw Fault("Firmware input exceeds size limit"); }
  if (bytes.size() >= 4 && bytes[0] == 'P' && bytes[1] == 'K') {
    return parse_midi(input::archive_midi(bytes));
  }
  return parse_midi(bytes);
}

Firmware Firmware::parse_midi(const Bytes &midi) {
  if (midi.size() > input_limit) { throw Fault("Firmware input exceeds size limit"); }
  Firmware result;
  const auto parts = blocks(input::sysex_messages(midi));
  constexpr std::array<unsigned, 6> modes{3, 0, 3, 0, 3, 0};
  constexpr std::array<std::uint32_t, 6> addresses{initial_address,
                                                   helper_address,
                                                   secondary_address,
                                                   helper_address,
                                                   main_address,
                                                   helper_address};
  if (parts.size() != modes.size()) { throw Fault("Expected the six-block Blofeld 1.25 layout"); }
  for (std::size_t index = 0; index < parts.size(); ++index) {
    if (parts[index].mode != modes[index] || parts[index].address != addresses[index]) {
      throw Fault("Unsupported firmware block mode or destination");
    }
  }
  for (const auto &part : parts) {
    result.sysex_packets += part.packets;
    result.update_blocks.push_back(decode(part));
  }
  const auto initial = std::span(result.update_blocks[0]);
  if (helper_end(initial) != initial.size()) { throw Fault("Trailing initial helper data"); }
  const auto secondary = std::span(result.update_blocks[2]);
  Reader page_header(secondary);
  if (page_header.big(3) != first_secondary_page || secondary.size() < pages_checksum_end) {
    throw Fault("Unsupported secondary flash page layout");
  }
  verify24(secondary.subspan(3, pages_checksum_end - 3));
  if (helper_end(secondary.subspan(pages_checksum_end)) != secondary.size() - pages_checksum_end) {
    throw Fault("Trailing secondary error helper data");
  }
  extract_runtime(result);
  return result;
}

} // namespace blofeld
