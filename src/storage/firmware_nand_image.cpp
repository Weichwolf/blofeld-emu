// SPDX-License-Identifier: GPL-3.0-only
#include "storage/firmware_nand_image.hpp"

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace blofeld {
namespace {
constexpr std::size_t word_bytes = 3;
constexpr unsigned high_shift = 16;
constexpr unsigned middle_shift = 8;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned flash_bytes = 32768;
constexpr std::size_t secondary_pages = 16;
constexpr unsigned secondary_first_page = 0x380;
constexpr unsigned terminal_word = 0x100;
constexpr unsigned main_block = 4;
constexpr unsigned secondary_block = 2;
constexpr unsigned expected_blocks = 6;

struct Payload {
  unsigned first_page;
  std::span<const std::uint8_t> bytes;
};

std::uint32_t big24(const std::span<const std::uint8_t> bytes, const std::size_t at) {
  if (at > bytes.size() || bytes.size() - at < word_bytes) {
    throw Fault("Initial NAND source has an incomplete word");
  }
  return (static_cast<std::uint32_t>(bytes[at]) << high_shift) |
         (static_cast<std::uint32_t>(bytes[at + 1]) << middle_shift) | bytes[at + 2];
}

void checksum(const std::span<const std::uint8_t> bytes) {
  if (bytes.size() % word_bytes != 0) {
    throw Fault("Initial NAND source has partial checksum data");
  }
  std::uint32_t sum = 0;
  for (std::size_t at = 0; at < bytes.size(); at += word_bytes) {
    sum = (sum + big24(bytes, at)) & word_mask;
  }
  if (sum != 0) { throw Fault("Initial NAND source object checksum mismatch"); }
}

Payload runtime(const Firmware &firmware) {
  const auto main = std::span(firmware.update_blocks.at(main_block));
  if (main.size() < 2 || firmware.mcu.size() != flash_bytes || firmware.dsp.empty()) {
    throw Fault("Initial NAND requires parsed resident MCU and DSP firmware data");
  }
  const auto helper_bytes = (static_cast<unsigned>(main[0]) << middle_shift) | main[1];
  const auto host_begin = (((helper_bytes + 4U) / word_bytes) * word_bytes) + word_bytes;
  constexpr std::size_t host_object_bytes =
      (((flash_bytes + 2) / word_bytes) * word_bytes) + word_bytes;
  const auto dsp_begin = static_cast<std::size_t>(host_begin) + host_object_bytes;
  if (dsp_begin > main.size() ||
      !std::ranges::equal(main.subspan(host_begin, flash_bytes), firmware.mcu)) {
    throw Fault("Initial NAND source differs from the parsed MCU object");
  }
  const auto source = main.subspan(dsp_begin);
  std::size_t at = 0;
  for (const auto &record : firmware.dsp) {
    if (record.words.empty() || big24(source, at) != record.space ||
        big24(source, at + word_bytes) != record.words.size() ||
        big24(source, at + (2 * word_bytes)) != record.address) {
      throw Fault("Initial NAND source differs from the ordered parsed DSP record");
    }
    at += 3 * word_bytes;
    if (record.words.size() > (source.size() - at) / word_bytes) {
      throw Fault("Initial NAND DSP record exceeds its source");
    }
    for (const auto word : record.words) {
      if (big24(source, at) != word) {
        throw Fault("Initial NAND source differs from parsed DSP data");
      }
      at += word_bytes;
    }
  }
  for (unsigned i = 0; i < 3; ++i) {
    if (big24(source, at) != terminal_word) {
      throw Fault("Initial NAND DSP end marker is absent");
    }
    at += word_bytes;
  }
  static_cast<void>(big24(source, at));
  at += word_bytes;
  checksum(source.first(at));
  constexpr unsigned transport_data_bytes = 7;
  const auto transport_bytes =
      ((main.size() + transport_data_bytes - 1) / transport_data_bytes) * transport_data_bytes;
  const auto complete_bytes = (transport_bytes / word_bytes) * word_bytes;
  const auto result = std::span(firmware.runtime_nand);
  if (complete_bytes < dsp_begin || result.size() != complete_bytes - dsp_begin ||
      result.size() < source.size() || !std::ranges::equal(source, result.first(source.size()))) {
    throw Fault("Initial NAND runtime transport source is incomplete or inconsistent");
  }
  if (result.size() > std::size_t{secondary_first_page} * NandArray::page_bytes) {
    throw Fault("Initial NAND runtime overlaps secondary pages");
  }
  return {.first_page = 0, .bytes = result};
}

Payload secondary(const Firmware &firmware) {
  const auto source = std::span(firmware.update_blocks.at(secondary_block));
  constexpr std::size_t payload_bytes = secondary_pages * NandArray::page_bytes;
  constexpr std::size_t checked_bytes = payload_bytes + word_bytes;
  if (source.size() < word_bytes + checked_bytes || big24(source, 0) != secondary_first_page) {
    throw Fault("Initial NAND secondary pages are missing or have an unsupported address");
  }
  checksum(source.subspan(word_bytes, checked_bytes));
  return {.first_page = secondary_first_page, .bytes = source.subspan(word_bytes, payload_bytes)};
}

NandImage unwritten_image(const NandUnwrittenState state) {
  NandImage result;
  switch (state) {
    case NandUnwrittenState::unknown: return result;
    case NandUnwrittenState::erased_virtual_device: break;
    default: throw Fault("Unknown initial NAND unwritten-state policy");
  }
  NandArray::Page erased;
  erased.cells.fill({.value = byte_mask, .known = byte_mask});
  erased.programs = {.main = 0, .spare = 0};
  for (unsigned page = 0; page < NandArray::pages; ++page) { result.pages.emplace(page, erased); }
  for (unsigned block = 0; block < NandArray::blocks; ++block) {
    result.blocks.emplace(
        block, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  }
  return result;
}

void copy_payload(NandImage &image, const Payload source) {
  for (std::size_t at = 0; at < source.bytes.size(); ++at) {
    const auto page = source.first_page + static_cast<unsigned>(at / NandArray::page_bytes);
    auto &destination = image.pages[page];
    destination.cells.at(at % NandArray::page_bytes) = {.value = source.bytes[at],
                                                        .known = byte_mask};
    // Input contents are observations; an updater's program history is separate.
    destination.programs.reset();
  }
}
}

NandImage initial_firmware_nand_image(const Firmware &firmware,
                                      const NandUnwrittenState unwritten) {
  if (firmware.update_blocks.size() != expected_blocks) {
    throw Fault("Initial NAND requires the supported six-block firmware layout");
  }
  // Validate complete input extents before allocating a full erased device.
  const std::array sources{runtime(firmware), secondary(firmware)};
  auto result = unwritten_image(unwritten);
  for (const auto source : sources) { copy_payload(result, source); }
  result.validate();
  return result;
}
}
