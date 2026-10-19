// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_image.hpp"
#include "nand_persistence_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace blofeld::test {
namespace {
constexpr std::size_t literal_size = 2172;
constexpr std::size_t literal_header = 32;
constexpr std::size_t first_page = 40;
constexpr std::size_t second_page = 1104;
constexpr std::size_t first_cell = 48;
constexpr std::size_t last_cell = 1102;
constexpr unsigned high_page = 131071;
constexpr unsigned high_block = 4095;
constexpr unsigned cell_value = 0xa0;
constexpr unsigned cell_known = 0xf0;

std::uint32_t reference_crc(const std::span<const std::uint8_t> bytes) {
  constexpr std::uint32_t polynomial = 0xedb88320;
  constexpr std::uint32_t initial = 0xffffffff;
  std::uint32_t crc = initial;
  for (const auto byte : bytes) {
    crc ^= byte;
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0 ? (crc >> 1U) ^ polynomial : crc >> 1U;
    }
  }
  return ~crc;
}

void repair(std::vector<std::uint8_t> &bytes) {
  const auto end = bytes.size() - sizeof(std::uint32_t);
  const auto crc = reference_crc(std::span(bytes).first(end));
  for (unsigned index = 0; index < 4; ++index) {
    bytes.at(end + index) = static_cast<std::uint8_t>((crc >> (index * 8U)) & persisted_mask);
  }
}

NandImage literal_fixture() {
  NandImage image;
  image.blocks[0] = {.valid = true, .program_protected = false};
  image.blocks[high_block] = {.program_protected = false, .erase_protected = false};
  auto &first = image.pages[0];
  first.programs = {.main = 1, .spare = 2};
  first.cells.at(0) = {.value = persisted_cookie, .known = persisted_mask};
  first.cells.back() = {.value = cell_value, .known = cell_known};
  image.pages[high_page] = {};
  return image;
}

void test_literal_layout_and_independent_crc() {
  const auto encoded = encode_nand_image(literal_fixture());
  constexpr std::array<std::uint8_t, literal_header> header{
      'B', 'L', 'F', 'N', 'A', 'N', 'D', '\n', 1,    0, 0, 0, 1, 0, 0, 0,
      2,   0,   0,   0,   2,   0,   0,   0,    0x10, 2, 0, 0, 0, 0, 0, 0};
  persistence_require(encoded.size() == literal_size &&
                          std::ranges::equal(header, std::span(encoded).first(literal_header)),
                      "Independent literal header fixes version, geometry, counts and byte order");
  constexpr std::array<std::uint8_t, 8> block_records{0, 0, 3, 1, 0xff, 0x0f, 6, 0};
  persistence_require(
      std::ranges::equal(block_records,
                         std::span(encoded).subspan(literal_header, block_records.size())),
      "Block index and separate optional flags must retain their literal positions");
  constexpr std::array<std::uint8_t, 8> page_header{0, 0, 0, 0, 1, 2, 0, 0};
  constexpr std::array<std::uint8_t, 8> last_header{0xff, 0xff, 1, 0, 0xff, 0xff, 0, 0};
  persistence_require(
      std::ranges::equal(page_header, std::span(encoded).subspan(first_page, page_header.size())) &&
          std::ranges::equal(last_header,
                             std::span(encoded).subspan(second_page, last_header.size())) &&
          encoded.at(first_cell) == persisted_cookie &&
          encoded.at(first_cell + 1) == persisted_mask && encoded.at(last_cell) == cell_value &&
          encoded.at(last_cell + 1) == cell_known,
      "Sparse page bounds, unknown history and each value/known pair match independent byte "
      "offsets");
  const auto end = encoded.size() - sizeof(std::uint32_t);
  const auto crc = reference_crc(std::span(encoded).first(end));
  for (unsigned index = 0; index < 4; ++index) {
    persistence_require(encoded.at(end + index) == ((crc >> (index * 8U)) & persisted_mask),
                        "CRC-32 must match the independent bit-at-a-time oracle");
  }
  const auto restored = decode_nand_image(encoded);
  const auto programs = restored.pages.at(0).programs;
  persistence_require(
      programs.has_value() && restored.pages.size() == 2 && restored.blocks.size() == 2 &&
          programs->main == 1 && programs->spare == 2 && !restored.pages.at(high_page).programs &&
          !restored.blocks.at(0).erase_protected &&
          restored.pages.at(0).cells.back() == NandByte{.value = cell_value, .known = cell_known},
      "Decoding must preserve optional facts rather than invent erased or unprotected defaults");
}

void test_truncation_and_corruption() {
  const auto valid = encode_nand_image(literal_fixture());
  for (std::size_t size = 0; size < valid.size(); ++size) {
    persistence_reject([&] { static_cast<void>(decode_nand_image(std::span(valid).first(size))); },
                       "NAND image");
  }
  auto changed = valid;
  changed.push_back(0);
  persistence_reject([&] { static_cast<void>(decode_nand_image(changed)); }, "NAND image");
  for (std::size_t index = 0; index < valid.size(); ++index) {
    changed = valid;
    changed.at(index) ^= 1;
    persistence_reject([&] { static_cast<void>(decode_nand_image(changed)); }, "NAND image");
  }
}

void test_crc_valid_semantic_rejections() {
  const auto valid = encode_nand_image(literal_fixture());
  constexpr std::size_t version = 8;
  constexpr std::size_t geometry = 12;
  constexpr std::size_t reserved = 28;
  for (const auto offset : {version, geometry, reserved}) {
    auto bytes = valid;
    bytes.at(offset) ^= 2;
    repair(bytes);
    persistence_reject([&] { static_cast<void>(decode_nand_image(bytes)); }, "NAND image");
  }

  struct Change {
    std::size_t offset;
    std::uint8_t value;
  };

  constexpr std::array<Change, 9> cases{{{.offset = 34, .value = 8},
                                         {.offset = 35, .value = 4},
                                         {.offset = 44, .value = 0xff},
                                         {.offset = 45, .value = 3},
                                         {.offset = 46, .value = 1},
                                         {.offset = first_cell + 1, .value = 0},
                                         {.offset = 44, .value = 0},
                                         {.offset = 45, .value = 0},
                                         {.offset = second_page + 2, .value = 2}}};
  for (const auto change : cases) {
    auto bytes = valid;
    bytes.at(change.offset) = change.value;
    repair(bytes);
    persistence_reject([&] { static_cast<void>(decode_nand_image(bytes)); }, "NAND image");
  }
  auto duplicate = valid;
  std::fill_n(duplicate.begin() + static_cast<std::ptrdiff_t>(second_page), 4, 0);
  repair(duplicate);
  persistence_reject([&] { static_cast<void>(decode_nand_image(duplicate)); },
                     "unordered NAND image page");
  auto image = literal_fixture();
  image.blocks[high_block].program_protected = true;
  persistence_reject([&] { static_cast<void>(encode_nand_image(image)); }, "protection exceeds");
}

void test_cold_projection_and_retained_history() {
  Nand device(erased_nand_fixture());
  prepare_persistent_program(device);
  persistence_reject([&] { static_cast<void>(device.persistent_image()); },
                     "power-off projection required");
  device.write(persisted_cookie);
  constexpr unsigned confirm = 0x10;
  device.command(confirm);
  auto cut = device;
  cut.power_loss();
  const Nand interrupted(decode_nand_image(encode_nand_image(cut.persistent_image())));
  persistence_require(!interrupted.ready() && interrupted.status().known == 0 &&
                          interrupted.cell({.page = persisted_row}).known == 0,
                      "Cold import must preserve interrupted uncertainty without inventing "
                      "readiness, WP or success");
  device.observe_completion(NandOutcome::pass);
  device.power_loss();
  Nand completed(decode_nand_image(encode_nand_image(device.persistent_image())));
  prepare_persistent_program(completed);
  completed.write(0);
  persistence_reject([&] { completed.command(confirm); }, "main partial-program limit");
  persistence_require(
      completed.cell({.page = persisted_row}) ==
          NandByte{.value = persisted_cookie, .known = persisted_mask},
      "Known completed data and consumed program history must survive process reconstruction");
}

void test_known_ff_does_not_invent_history() {
  auto image = erased_nand_fixture();
  image.pages.at(persisted_row).programs.reset();
  Nand device(decode_nand_image(encode_nand_image(image)));
  prepare_persistent_program(device);
  device.write(0);
  constexpr unsigned confirm = 0x10;
  persistence_reject([&] { device.command(confirm); }, "unknown program history");
  image.pages.at(persisted_row).programs = NandProgramCount{};
  image.pages.at(persisted_row).cells.at(0) = {};
  const auto restored = decode_nand_image(encode_nand_image(image));
  const auto programs = restored.pages.at(persisted_row).programs;
  persistence_require(programs.has_value() &&
                          restored.pages.at(persisted_row).cells.at(0).known == 0 &&
                          programs->main == 0,
                      "Explicit erase-history observation does not manufacture content knowledge");
}
}
}

int main() {
  try {
    blofeld::test::test_literal_layout_and_independent_crc();
    blofeld::test::test_truncation_and_corruption();
    blofeld::test::test_crc_valid_semantic_rejections();
    blofeld::test::test_cold_projection_and_retained_history();
    blofeld::test::test_known_ff_does_not_invent_history();
    std::cout << "NAND image literal layout, CRC, bounded parser and cold history checks passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
