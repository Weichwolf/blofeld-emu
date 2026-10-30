// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "storage/flash_image.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>

namespace {
constexpr unsigned array_base = 0x8000, target = 0xa000, erased = 0xff;
constexpr unsigned status_register = 0x105, command_register = 0x106, divider = 0x100;
constexpr unsigned program = 0x20, mass = 0x41, verify = 5, busy = 0x80, idle = 0xc0;
constexpr unsigned base_hz = 60000000, oscillator_hz = 12000000, bus_ticks = 10;
constexpr unsigned word_data = 0x1234, byte_mask = 0xff, row_words = 32;
constexpr std::uint64_t single_ticks = 3490, burst_ticks = 1530, row_ticks = 50920;
constexpr std::uint64_t mass_ticks = 7200000, word_verify_ticks = 163940;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault("Expected explicit clock/array rejection");
}

blofeld::McuFlashImage image() {
  constexpr unsigned security_offset = 0x7f0f;
  constexpr unsigned unsecured = 0xfe;
  blofeld::McuFlashImage result;
  result.bytes.fill(erased);
  result.known.set();
  result.fresh.set();
  result.bytes[security_offset] = unsecured;
  result.fresh.reset(security_offset / 2);
  return result;
}

void advance(blofeld::McuBus &bus, const std::uint64_t ticks, const bool endpoint = true) {
  bus.advance_flash_time({.ticks = ticks,
                          .timebase_hz = base_hz,
                          .oscillator_hz = oscillator_hz,
                          .bus_period_ticks = bus_ticks,
                          .include_endpoint = endpoint});
}

void prepare(blofeld::McuBus &bus) {
  constexpr unsigned configured_divider = 0x48;
  bus.configure_flash_timing({.contract = blofeld::FlashTimingContract::uf32_formula_candidate,
                              .transfer_bus_cycles = 1,
                              .verify_unit = blofeld::FlashVerifyUnit::words});
  advance(bus, 0);
  bus.write(divider, configured_divider);
}

struct Launch {
  unsigned address = target;
  unsigned command = program;
};

void launch(blofeld::McuBus &bus, const Launch job) {
  bus.write_memory(
      {.address = static_cast<std::uint16_t>(job.address), .size = 2, .value = word_data});
  bus.write(command_register, static_cast<std::uint8_t>(job.command));
  bus.write(status_register, busy);
}

void test_row_and_crossing() {
  for (const unsigned spacing : {2U, 64U}) {
    blofeld::McuBus bus(image());
    prepare(bus);
    launch(bus, {});
    require(bus.read(status_register) == 0 && bus.next_flash_event_ticks() == bus_ticks,
            "Launch owns a real pending buffer before its explicit transfer candidate");
    advance(bus, bus_ticks);
    std::uint64_t end = single_ticks;
    const auto count = spacing == 2 ? row_words : 2U;
    for (unsigned index = 1; index < count; ++index) {
      launch(bus, {.address = target + (index * spacing)});
      require(bus.read(status_register) == 0 && bus.next_flash_event_ticks() == end,
              "The successor waits for the active deadline with CCIF still clear");
      advance(bus, end - 1);
      require(bus.read(status_register) == 0, "Neither queued job completes one tick early");
      advance(bus, end);
      require(bus.read(status_register) == busy,
              "Old completion and real successor transfer share one calendar boundary");
      end += spacing == 2 ? burst_ticks : single_ticks;
      require(bus.next_flash_event_ticks() == end,
              "Only a genuinely queued same-row successor receives the burst duration");
    }
    require(end == (spacing == 2 ? row_ticks : single_ticks * 2),
            "The independent full-row and cross-row formulas determine the endpoint");
    advance(bus, end);
    require(bus.read(status_register) == idle && !bus.next_flash_event_ticks(),
            "The final actual job alone releases CCIF");
    const auto after = bus.power_off_flash_image();
    for (unsigned index = 0; index < count; ++index) {
      const auto offset = (target - array_base) + (index * spacing);
      require(after.bytes.at(offset) == word_data >> 8U &&
                  after.bytes.at(offset + 1) == (word_data & byte_mask) &&
                  !after.fresh.test(offset / 2),
              "Every buffered command changes its own physical cells and history");
    }
  }
}

void test_verify_after_mutation() {
  blofeld::McuBus bus(image());
  prepare(bus);
  launch(bus, {.command = mass});
  advance(bus, bus_ticks);
  launch(bus, {.command = verify});
  advance(bus, mass_ticks);
  require(bus.read(status_register) == busy &&
              bus.next_flash_event_ticks() == mass_ticks + word_verify_ticks,
          "Queued verify scans the actually erased array after mass completion");
  advance(bus, mass_ticks + word_verify_ticks);
  constexpr unsigned blank = 4;
  require(bus.read(status_register) == (idle | blank),
          "Actual blank verification sets BLANK after all 16384 words");
}

void test_clone_and_power_cut() {
  blofeld::McuBus bus(image());
  prepare(bus);
  launch(bus, {});
  const auto pending = bus.power_off_flash_image();
  require(pending.known.all() && pending.fresh.test((target - array_base) / 2),
          "A cut before real transfer leaves the accepted pending word unchanged");
  advance(bus, bus_ticks);
  auto clone = bus.clone();
  const auto cut = bus.power_off_flash_image();
  require(!cut.known.test(target - array_base) && !cut.known.test(target - array_base + 1) &&
              !cut.fresh.test((target - array_base) / 2),
          "A cut during an active pulse invalidates exactly its destructive word");
  bus.reset();
  advance(bus, single_ticks);
  require(bus.read(status_register) == idle && !bus.next_flash_event_ticks(),
          "Reset cancels the whole calendar instead of replaying canceled jobs");
  rejected([&] { static_cast<void>(bus.read(target)); });
  advance(*clone, single_ticks, false);
  require(clone->read(status_register) == busy,
          "The explicit COP-priority candidate excludes completion at an equal boundary");
  advance(*clone, single_ticks);
  require(clone->read(status_register) == idle && clone->read(target) == word_data >> 8U,
          "Deep clone retains its own active cells and exact completion calendar");
  rejected([&] { advance(*clone, single_ticks - 1); });
}
}

int main() {
  try {
    test_row_and_crossing();
    test_verify_after_mutation();
    test_clone_and_power_cut();
    std::cout << "Calendar controls:32-word burst,cross-row,queued post-erase verify,clone,"
                 "pending/active cuts,reset and exclusive endpoint passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
