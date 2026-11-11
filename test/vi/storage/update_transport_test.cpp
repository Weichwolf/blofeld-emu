// SPDX-License-Identifier: GPL-3.0-only
#include "storage/firmware.hpp"
#include "storage/update_transport.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::vector<std::uint8_t> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  const auto size = input.tellg();
  require(size > 0, "Read the independent transport fixture");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  input.seekg(0);
  require(static_cast<bool>(input.read(reinterpret_cast<char *>(bytes.data()),
                                       static_cast<std::streamsize>(bytes.size()))),
          "Read the complete transport fixture");
  return bytes;
}

void blocks(const std::filesystem::path &path) {
  const auto input = read(path);
  const auto decoded = blofeld::vi::update::decode_midi(input);
  constexpr std::array<std::uint8_t, 6> first{0x12, 0x34, 0x56, 0xfe, 0xdc, 0xba};
  constexpr std::array<std::uint8_t, 12> second{
      0x89, 0xab, 0xcd, 0xef, 1, 0x23, 0x45, 0x67, 0x89, 0x5a, 0x5a, 0x5a};
  constexpr std::uint32_t first_address = 0x1111;
  constexpr std::uint32_t second_address = 0x2222;
  require(decoded.size() == 2 && decoded[0].mode == 3 && decoded[1].mode == 0 &&
              decoded[0].address == first_address && decoded[1].address == second_address &&
              decoded[0].declared_bytes == first.size() && decoded[1].declared_bytes == 9 &&
              std::ranges::equal(decoded[0].bytes, first) &&
              std::ranges::equal(decoded[1].bytes, second),
          "Block order, destinations, MSBs and complete supplied words survive decoding");
  bool rejected = false;
  try {
    static_cast<void>(blofeld::vi::FirmwareUpdate::from_midi(input));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Valid transport alone does not admit an unsupported instrument layout");
}

void sequenced(const std::filesystem::path &path) {
  const auto decoded = blofeld::vi::update::decode_midi(read(path));
  constexpr unsigned declared = 162;
  constexpr unsigned supplied = 168;
  require(decoded.size() == 1 && decoded[0].declared_bytes == declared &&
              decoded[0].bytes.size() == supplied,
          "Multiple ordered packets retain their declared length and extra complete words");
  for (unsigned index = 0; index < supplied; ++index) {
    const unsigned expected = index < declared ? index : 0x5a;
    require(decoded[0].bytes[index] == expected,
            "Packet boundaries neither reverse nor duplicate actual payload bytes");
  }
}

void rejected(const std::filesystem::path &path, const std::string_view diagnostic) {
  bool failed = false;
  try {
    static_cast<void>(blofeld::vi::update::decode_midi(read(path)));
  } catch (const std::runtime_error &error) {
    failed = std::string_view(error.what()).contains(diagnostic);
  }
  require(failed, "Malformed transport fails for its actual framing violation");
}
}

int main(const int argc, char **argv) try {
  require(argc == 3, "Supply artifact and fixture directories");
  const std::filesystem::path root(argv[2]);
  blocks(root / "transport.mid");
  rejected(root / "transport.syx", "MIDI header");
  sequenced(root / "sequenced.mid");
  rejected(root / "bad-length.mid", "declared length");
  rejected(root / "bad-sequence.mid", "sequence");
  rejected(root / "bad-packet.mid", "XOR");
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
