// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <vector>

namespace {
constexpr unsigned pattern = 0xa5;
constexpr unsigned erased = 0xff;
constexpr std::size_t header_bytes = 16;
constexpr std::size_t byte_count = 32768;
constexpr std::size_t known_offset = 32784;
constexpr std::size_t fresh_offset = 36880;
constexpr std::size_t crc_offset = 38928;
constexpr std::size_t total_bytes = 38932;
constexpr unsigned target = 0x8234;
constexpr unsigned physical_target = target - 0x8000;
constexpr unsigned other_word = target + 2;
constexpr unsigned divider = 0x100;
constexpr unsigned status = 0x105;
constexpr unsigned command = 0x106;
constexpr unsigned reference_divider = 0x48;
constexpr unsigned program = 0x20;
constexpr unsigned erase = 0x40;
constexpr unsigned mass = 0x41;
constexpr unsigned verify = 5;
constexpr unsigned sector_first = 0x200;
constexpr unsigned sector_limit = 0x400;
constexpr unsigned word_data = 0x1234;
constexpr unsigned second_data = 0x5678;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned bus_hz = 24000000;

void require(const bool value, const char *message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::uint32_t reference_crc(const std::span<const std::uint8_t> bytes) {
  // Independent reflected, bit-at-a-time CRC-32/ISO-HDLC expectation, not zlib.
  constexpr std::uint32_t polynomial = 0xedb88320;
  constexpr std::uint32_t initial_crc = 0xffffffff;
  std::uint32_t crc = initial_crc;
  for (const auto byte : bytes) {
    crc ^= byte;
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0 ? (crc >> 1U) ^ polynomial : crc >> 1U;
    }
  }
  return ~crc;
}

std::uint32_t little_u32(const std::span<const std::uint8_t> bytes) {
  constexpr unsigned top_byte_shift = 24;
  return bytes[0] | (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << top_byte_shift);
}

void repair_checksum(std::vector<std::uint8_t> &bytes) {
  const auto crc = reference_crc(std::span<const std::uint8_t>(bytes).first(crc_offset));
  for (unsigned index = 0; index < 4; ++index) {
    bytes[crc_offset + index] = static_cast<std::uint8_t>((crc >> (index * 8U)) & erased);
  }
}

blofeld::McuFlashImage fixture() {
  std::array<std::uint8_t, byte_count> bytes{};
  bytes.fill(pattern);
  constexpr unsigned protection_field = 0x7f0d;
  constexpr unsigned security_field = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  bytes[protection_field] = erased;
  bytes[security_field] = unsecured;
  return blofeld::McuFlashImage::from_known(bytes);
}

void test_literal_file_layout() {
  auto image = fixture();
  constexpr unsigned first_byte = 0x12;
  image.bytes[0] = first_byte;
  image.invalidate({.offset = 1, .size = 1});
  image.bytes[4] = erased;
  image.bytes[5] = erased;
  image.fresh.set(2);
  const auto bytes = blofeld::encode_flash_image(image);
  constexpr std::array<std::uint8_t, 16> expected_header{
      'B', 'L', 'F', 'N', 'V', 'M', '\r', '\n', 1, 0, 0, 0, 0, 0x98, 0, 0};
  require(bytes.size() == total_bytes &&
              std::ranges::equal(expected_header,
                                 std::span<const std::uint8_t>(bytes).first(header_bytes)),
          "Independent literal version/length header fixes layout and byte order");
  constexpr unsigned known_first = 0xfd;
  require(
      bytes[header_bytes] == first_byte && bytes[header_bytes + 1] == 0 &&
          bytes[header_bytes + 4] == erased && bytes[known_offset] == known_first &&
          bytes[known_offset + 1] == erased && bytes[fresh_offset] == 4 &&
          bytes[fresh_offset + 1] == 0,
      "Data and LSB-first knowledge/history masks match independent byte-position expectations");
  const std::span<const std::uint8_t> data(bytes);
  require(little_u32(data.subspan(crc_offset, 4)) == reference_crc(data.first(crc_offset)),
          "File CRC is checked against an independent bit-level oracle");
}

void test_invalid_files() {
  const auto valid = blofeld::encode_flash_image(fixture());
  for (std::size_t size = 0; size < valid.size(); ++size) {
    reject([&valid,
            size] { static_cast<void>(blofeld::decode_flash_image(std::span(valid).first(size))); },
           "Every truncated file length is rejected");
  }
  auto changed = valid;
  changed.push_back(0);
  reject([&changed] { static_cast<void>(blofeld::decode_flash_image(changed)); },
         "Trailing bytes are rejected");
  for (const auto offset :
       {std::size_t{0}, std::size_t{8}, std::size_t{12}, header_bytes, crc_offset}) {
    changed = valid;
    changed[offset] ^= 1;
    reject([&changed] { static_cast<void>(blofeld::decode_flash_image(changed)); },
           "Header, payload and checksum corruption cannot restore data");
  }
  changed = valid;
  changed[8] = 2;
  repair_checksum(changed);
  reject([&changed] { static_cast<void>(blofeld::decode_flash_image(changed)); },
         "A valid CRC cannot authorize an unsupported version");
  changed = valid;
  constexpr unsigned unknown_first_byte = 0xfe;
  changed[known_offset] &= unknown_first_byte;
  repair_checksum(changed);
  reject([&changed] { static_cast<void>(blofeld::decode_flash_image(changed)); },
         "Unknown data must be canonical zero even with a valid checksum");
  changed = valid;
  changed[fresh_offset] = 1;
  repair_checksum(changed);
  reject([&changed] { static_cast<void>(blofeld::decode_flash_image(changed)); },
         "Erase history cannot mark a non-erased word as freshly erased");
}

void prepare(blofeld::McuBus &bus) {
  bus.write(divider, reference_divider);
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
}

struct Operation {
  std::uint16_t address = target;
  std::uint8_t command = program;
  std::uint16_t data = word_data;
};

void launch(blofeld::McuBus &bus, const Operation operation) {
  bus.write_memory({.address = operation.address, .size = 2, .value = operation.data});
  bus.write(command, operation.command);
  bus.write(status, 0x80);
  bus.advance_flash(blofeld::FlashMilestone::transfer_buffer);
}

void test_cold_restart_and_snapshot_isolation() {
  blofeld::McuBus bus(fixture());
  prepare(bus);
  launch(bus, {.command = erase});
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  launch(bus, {});
  const auto interrupted = bus.power_off_flash_image();
  require(!interrupted.known.test(physical_target) && interrupted.bytes[physical_target] == 0 &&
              !interrupted.fresh.test(physical_target / 2),
          "Capturing an active program conservatively invalidates its saved word");
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  require(bus.read_memory({.address = target, .size = 2}) == word_data,
          "Capturing a cold image does not stop or mutate the live controller");
  blofeld::McuBus damaged(blofeld::decode_flash_image(blofeld::encode_flash_image(interrupted)));
  reject([&damaged] { static_cast<void>(damaged.read(target)); },
         "Unknown NVM survives reconstruction");
  constexpr unsigned alias = 0x4234;
  reject([&damaged] { static_cast<void>(damaged.read(alias)); },
         "All aliases retain the same unknownness");
  require(damaged.read(other_word) == erased && damaged.read(divider) == 0,
          "Untouched erased data survives while volatile controller configuration starts cold");
  blofeld::McuBus completed(
      blofeld::decode_flash_image(blofeld::encode_flash_image(bus.power_off_flash_image())));
  require(completed.read_memory({.address = target, .size = 2}) == word_data,
          "Completed data survives reconstruction instead of reverting to the initial image");
  prepare(completed);
  launch(completed, {.address = other_word, .data = second_data});
  completed.advance_flash(blofeld::FlashMilestone::complete_command);
  require(completed.read_memory({.address = other_word, .size = 2}) == second_data,
          "Persisted erase history admits programming an untouched erased word");
}

void test_program_history_and_unknown_reset_fields() {
  blofeld::McuBus bus(fixture());
  prepare(bus);
  launch(bus, {.command = erase});
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  launch(bus, {.data = word_mask});
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  blofeld::McuBus restored(
      blofeld::decode_flash_image(blofeld::encode_flash_image(bus.power_off_flash_image())));
  prepare(restored);
  reject([&restored] { launch(restored, {}); },
         "Persisted all-ones program history cannot become a fresh erased word");
  constexpr unsigned security_word = 0xff0e;
  launch(bus, {.address = security_word, .command = erase});
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  launch(bus, {.address = security_word});
  const auto damaged_options =
      blofeld::decode_flash_image(blofeld::encode_flash_image(bus.power_off_flash_image()));
  reject([&damaged_options] { const blofeld::McuBus rejected(damaged_options); },
         "A valid persistent file with unknown option fields cannot construct a bootable device");
}

void check_reference_image(const blofeld::McuFlashImage &image,
                           const unsigned operation,
                           const unsigned phase) {
  const auto initial = fixture();
  for (std::size_t offset = 0; offset < byte_count; ++offset) {
    const bool sector = offset >= sector_first && offset < sector_limit;
    const bool word = offset == physical_target || offset == physical_target + 1;
    const bool domain = operation == program ? word : operation == mass || sector;
    const bool unknown = phase == 4 && operation != verify && domain;
    unsigned value = operation == program && sector ? erased : initial.bytes[offset];
    bool fresh = operation == program && sector;
    if (phase == 5 && operation != verify && domain) {
      value = erased;
      if (operation == program) {
        value = offset == physical_target ? word_data >> 8U : word_data & erased;
      }
      fresh = operation != program;
    }
    if (unknown) {
      value = 0;
      fresh = false;
    }
    require(image.bytes[offset] == value && image.known.test(offset) == !unknown &&
                image.fresh.test(offset / 2) == fresh,
            "Every byte/history bit matches independent command/phase domain expectations");
  }
}

void test_all_command_export_positions() {
  for (const auto operation : {program, erase, mass, verify}) {
    for (unsigned phase = 0; phase < 6; ++phase) {
      blofeld::McuBus bus(fixture());
      prepare(bus);
      if (operation == program) {
        launch(bus, {.command = erase});
        bus.advance_flash(blofeld::FlashMilestone::complete_command);
      }
      if (phase >= 1) { bus.write_memory({.address = target, .size = 2, .value = word_data}); }
      if (phase >= 2) { bus.write(command, static_cast<std::uint8_t>(operation)); }
      if (phase >= 3) { bus.write(status, 0x80); }
      if (phase >= 4) { bus.advance_flash(blofeld::FlashMilestone::transfer_buffer); }
      if (phase >= 5) { bus.advance_flash(blofeld::FlashMilestone::complete_command); }
      const auto image =
          blofeld::decode_flash_image(blofeld::encode_flash_image(bus.power_off_flash_image()));
      check_reference_image(image, operation, phase);
    }
  }
}

void test_pending_successor_is_not_restored() {
  blofeld::McuBus bus(fixture());
  prepare(bus);
  launch(bus, {.command = erase});
  bus.advance_flash(blofeld::FlashMilestone::complete_command);
  launch(bus, {});
  bus.write_memory({.address = other_word, .size = 2, .value = second_data});
  bus.write(command, program);
  bus.write(status, 0x80);
  const auto saved =
      blofeld::decode_flash_image(blofeld::encode_flash_image(bus.power_off_flash_image()));
  require(!saved.known.test(physical_target) && saved.known.test(physical_target + 2) &&
              saved.bytes[physical_target + 2] == erased &&
              saved.fresh.test((physical_target + 2) / 2),
          "Only the executing target is damaged; queued data never reaches the cold image");
  blofeld::McuBus restored(saved);
  prepare(restored);
  launch(restored, {.address = other_word, .data = second_data});
  restored.advance_flash(blofeld::FlashMilestone::complete_command);
  require(restored.read_memory({.address = other_word, .size = 2}) == second_data,
          "Cold restore discards the queue and permits a new command for the untouched word");
  reject([&restored] { static_cast<void>(restored.read(target)); },
         "Programming an unrelated word cannot repair the interrupted target");
}
}

int main() {
  try {
    test_literal_file_layout();
    test_invalid_files();
    test_cold_restart_and_snapshot_isolation();
    test_program_history_and_unknown_reset_fields();
    test_all_command_export_positions();
    test_pending_successor_is_not_restored();
    std::cout
        << "MCU NVM layout/CRC/invariant/cold-restart checks passed; no complete board snapshot.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
