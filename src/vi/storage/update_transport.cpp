// SPDX-License-Identifier: GPL-3.0-only
#include "storage/update_transport.hpp"

#include "storage/update_reader.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace blofeld::vi::update {
namespace {
constexpr unsigned manufacturer = 0x3e;
constexpr unsigned device = 0x13;
constexpr unsigned device_broadcast = 0x7f;
constexpr unsigned header_command = 0x73;
constexpr unsigned data_command = 0x74;

struct Packed {
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

std::vector<Packed> parse_blocks(const std::vector<Bytes> &messages) {
  std::vector<Packed> result;
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

Bytes unpack(const Packed &block) {
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

}

std::vector<Block> decode_midi(const std::span<const std::uint8_t> midi) {
  if (midi.size() > input_limit) { throw std::runtime_error("Firmware input exceeds size limit"); }
  const auto parts = parse_blocks(sysex_messages(midi));
  if (parts.empty()) { throw std::runtime_error("Firmware transport has no blocks"); }
  std::vector<Block> result;
  result.reserve(parts.size());
  for (const auto &part : parts) {
    auto bytes = unpack(part);
    bytes.resize((bytes.size() / 3) * 3);
    reverse_words(bytes);
    result.push_back({.mode = part.mode,
                      .address = part.address,
                      .declared_bytes = part.length,
                      .bytes = std::move(bytes)});
  }
  return result;
}
}
