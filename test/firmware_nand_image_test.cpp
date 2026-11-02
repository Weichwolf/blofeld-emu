// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "storage/firmware_nand_image.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned flash_bytes = 32768;
constexpr unsigned runtime_begin = 32778;
constexpr unsigned runtime_source_bytes = 45;
constexpr std::uint8_t trailer = 0xe0;
constexpr std::uint8_t padding = 0xa0;
constexpr unsigned secondary_first = 896;
constexpr unsigned secondary_count = 16;
constexpr unsigned secondary_bytes = 8448;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned runtime_address = 0x124;
constexpr unsigned first_word = 0x123456;
constexpr unsigned second_word = 0xabcdef;
constexpr std::uint8_t fill = 0x42;
constexpr std::uint8_t first = 0x20;
constexpr std::uint8_t last = 0x60;
// Literal independent P/X/Y record fields, two data words, terminator and checksum.
constexpr std::array<std::uint8_t, 27> runtime{
    0,    0, 2, 0, 0, 2, 0, 1, 0x24, 0x12, 0x34, 0x56, 0xab, 0xcd,
    0xef, 0, 1, 0, 0, 1, 0, 0, 1,    0,    0x41, 0xf9, 0x93};

void require(const bool condition, const char *reason) {
  if (!condition) { throw Fault(reason); }
}

void rejects(const auto &operation, const std::string_view expected) {
  bool rejected = false;
  try {
    operation();
  } catch (const Fault &error) { rejected = std::string_view(error.what()).contains(expected); }
  require(rejected, "Invalid initial NAND source must reject without substituting erased data");
}

Firmware fixture() {
  Firmware f;
  f.mcu.resize(flash_bytes);
  f.dsp = {{.space = 2, .address = runtime_address, .words = {first_word, second_word}}};
  f.update_blocks.resize(6);
  auto &main = f.update_blocks.at(4);
  constexpr unsigned declared_trailer_bytes = 12;
  main.resize(runtime_begin + runtime.size() + declared_trailer_bytes);
  std::ranges::copy(runtime, main.begin() + runtime_begin);
  std::ranges::fill(std::span(main).last(declared_trailer_bytes), trailer);
  f.runtime_nand.assign(main.begin() + runtime_begin, main.end());
  for (unsigned i = 0; i < 6; ++i) {
    f.runtime_nand.push_back(static_cast<std::uint8_t>(padding + i));
  }
  auto &secondary = f.update_blocks.at(2);
  constexpr unsigned checksum_and_helper = 9;
  secondary.resize(3 + secondary_bytes + checksum_and_helper);
  secondary.at(1) = 3;
  secondary.at(2) = 0x80;
  auto data = std::span(secondary).subspan(3, secondary_bytes);
  std::ranges::fill(data, fill);
  for (std::size_t page = 0; page < secondary_count; ++page) {
    data[page * NandArray::page_bytes] = static_cast<std::uint8_t>(first + page);
    data[((page + 1) * NandArray::page_bytes) - 1] = static_cast<std::uint8_t>(last + page);
  }
  // Independently calculated additive checksum for this authored byte pattern.
  constexpr std::array<std::uint8_t, 3> checksum{0xcf, 0x27, 0xa8};
  std::ranges::copy(checksum, secondary.begin() + 3 + secondary_bytes);
  std::ranges::fill(std::span(secondary).last(6), byte_mask);
  return f;
}

void source_only() {
  const auto f = fixture();
  const auto before = f.update_blocks;
  const auto image = initial_firmware_nand_image(f, NandUnwrittenState::unknown);
  constexpr unsigned expected_pages = 17;
  require(image.pages.size() == expected_pages && image.blocks.empty() &&
              !image.pages.contains(1) && !image.pages.contains(secondary_first + secondary_count),
          "Both payloads are mandatory; unknown neighbors and trailing helper are not invented");
  const auto &page = image.pages.at(0);
  for (std::size_t i = 0; i < runtime.size(); ++i) {
    require(page.cells.at(i) == NandByte{.value = runtime[i], .known = byte_mask},
            "Every raw runtime byte, including headers and checksum, matches the literal oracle");
  }
  require(page.cells.at(runtime.size()) == NandByte{.value = trailer, .known = byte_mask} &&
              page.cells.at(runtime_source_bytes - 1) ==
                  NandByte{.value = padding + 5, .known = byte_mask},
          "Declared tail and actual whole transport padding are retained verbatim");
  require(page.cells.at(runtime_source_bytes) == NandByte{} && !page.programs,
          "Source-only padding and physical program history remain unknown");
  for (unsigned i = 0; i < secondary_count; ++i) {
    const auto &overlay = image.pages.at(secondary_first + i);
    require(overlay.cells.front() ==
                    NandByte{.value = static_cast<std::uint8_t>(first + i), .known = byte_mask} &&
                overlay.cells.back() ==
                    NandByte{.value = static_cast<std::uint8_t>(last + i), .known = byte_mask} &&
                overlay.cells.at(1) == NandByte{.value = fill, .known = byte_mask} &&
                !overlay.programs,
            "All sixteen whole secondary pages retain independent edge/interior byte expectations");
  }
  require(f.update_blocks == before, "Initial memory construction never changes the update source");
}

void erased_virtual() {
  const auto image =
      initial_firmware_nand_image(fixture(), NandUnwrittenState::erased_virtual_device);
  require(image.pages.size() == NandArray::pages && image.blocks.size() == NandArray::blocks,
          "An explicitly erased virtual device has complete declared geometry");
  const NandByte erased{.value = byte_mask, .known = byte_mask};
  require(image.pages.at(0).cells.at(runtime_source_bytes) == erased &&
              image.pages.at(1).cells.front() == erased &&
              image.pages.rbegin()->second.cells.back() == erased,
          "Only the explicit erased policy supplies known FF beyond source data");
  const auto &unused = image.pages.at(1);
  require(unused.programs && unused.programs->main == 0 && unused.programs->spare == 0 &&
              !image.pages.at(0).programs && !image.pages.at(secondary_first).programs,
          "Erased unused pages and unknown source program history remain distinct");
  for (const auto &[index, block] : image.blocks) {
    static_cast<void>(index);
    require(block.valid == true && block.program_protected == false &&
                block.erase_protected == false,
            "Good unprotected virtual blocks are an explicit initial-state declaration");
  }
}

void malformed() {
  auto f = fixture();
  f.update_blocks.pop_back();
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "six-block");
  f = fixture();
  f.update_blocks.at(2).resize(3);
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "secondary");
  f = fixture();
  f.update_blocks.at(4).resize(2);
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "MCU object");
  f = fixture();
  f.mcu.front() = 1;
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "MCU object");
  f = fixture();
  f.dsp.front().words.front() = 0;
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "DSP data");
  f = fixture();
  f.update_blocks.at(4).at(runtime_begin + runtime.size() - 1) ^= 1;
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "checksum");
  f = fixture();
  f.runtime_nand.pop_back();
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, NandUnwrittenState::unknown)); },
          "transport source");
  f = fixture();
  constexpr auto invalid = static_cast<NandUnwrittenState>(99);
  rejects([&] { static_cast<void>(initial_firmware_nand_image(f, invalid)); }, "policy");
}

struct ReferenceFiles {
  std::filesystem::path firmware;
  std::filesystem::path output;
};

void reference(const ReferenceFiles &files) {
  const auto image =
      initial_firmware_nand_image(Firmware::load(files.firmware), NandUnwrittenState::unknown);
  save_nand_image(files.output, image);
  std::cout << "Source-only firmware NAND pages=" << image.pages.size()
            << " blocks=" << image.blocks.size() << '\n';
}
}

int main(const int argc, char **argv) {
  try {
    if (argc == 3) {
      reference({.firmware = argv[1], .output = argv[2]});
      return 0;
    }
    if (argc != 1) {
      throw Fault("Expected no arguments or private firmware/archive output paths");
    }
    source_only();
    erased_virtual();
    malformed();
    std::cout << "Initial NAND: literal runtime and all secondary pages, explicit unknown/erased "
                 "policies, source immutability and malformed-input refusals passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
