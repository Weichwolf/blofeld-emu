// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "storage/flash_file.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

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

struct Binding {
  std::shared_ptr<blofeld::VirtualDeviceState> owner;
  std::shared_ptr<unsigned> publications;
};

Binding prepare(blofeld::McuBus &bus, const std::filesystem::path &path) {
  const auto initial = bus.power_off_flash_image();
  blofeld::save_flash_image(path, initial);
  auto owner =
      std::make_shared<blofeld::VirtualDeviceState>(path, initial, blofeld::VirtualSerialFactory{});
  auto count = std::make_shared<unsigned>(0);
  owner->set_observer([count](const blofeld::PrivateFilePhase phase) {
    if (phase == blofeld::PrivateFilePhase::created) { ++*count; }
  });
  bus.attach_flash_store(owner);
  constexpr unsigned configured_divider = 0x48;
  bus.configure_flash_timing({.contract = blofeld::FlashTimingContract::uf32_formula_candidate,
                              .transfer_bus_cycles = 1,
                              .verify_unit = blofeld::FlashVerifyUnit::words});
  advance(bus, 0);
  bus.write(divider, configured_divider);
  return {.owner = std::move(owner), .publications = std::move(count)};
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

void test_row_and_crossing(const std::filesystem::path &directory) {
  for (const unsigned spacing : {2U, 64U}) {
    blofeld::McuBus bus(image());
    const auto path = directory / ("row-" + std::to_string(spacing) + ".bfnvm");
    const auto binding = prepare(bus, path);
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
    require(*binding.publications == count * 2,
            "Each actual queued destructive phase is durably published exactly once");
    const auto disk = blofeld::load_flash_image(path);
    const auto after = bus.power_off_flash_image();
    require(disk.bytes == after.bytes && disk.known == after.known && disk.fresh == after.fresh,
            "Native queued completion never acknowledges an unpublished array/history");
    for (unsigned index = 0; index < count; ++index) {
      const auto offset = (target - array_base) + (index * spacing);
      require(after.bytes.at(offset) == word_data >> 8U &&
                  after.bytes.at(offset + 1) == (word_data & byte_mask) &&
                  !after.fresh.test(offset / 2),
              "Every buffered command changes its own physical cells and history");
    }
  }
}

void test_verify_after_mutation(const std::filesystem::path &path) {
  blofeld::McuBus bus(image());
  const auto binding = prepare(bus, path);
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
  require(*binding.publications == 2,
          "Read-only queued verify adds no publication after the two actual mass phases");
  const auto disk = blofeld::load_flash_image(path);
  const auto after = bus.power_off_flash_image();
  require(disk.bytes == after.bytes && disk.known == after.known && disk.fresh == after.fresh,
          "Queued verify sees the actual already-published mass result");
}

}

int main(const int argc, char **argv) {
  try {
    require(argc == 2, "Expected a fresh private test directory");
    const std::filesystem::path directory(argv[1]);
    test_row_and_crossing(directory);
    test_verify_after_mutation(directory / "mass-verify.bfnvm");
    std::cout << "Durable calendar:32-word burst,cross-row and queued post-mass verify passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
