// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
constexpr unsigned prepare = 0xb938, prepare_return = 0xb954;
constexpr unsigned release = 0xb98a, poll = 0xb98d, continuation = 0xb992;
constexpr unsigned accumulator = 0x1234, index_x = 0x4567, index_y = 0x6789, stack = 0x1f00;
constexpr unsigned direction_u = hc12::ptu + 2, pin_u = hc12::ptu + 1;
constexpr unsigned release_flags = 0xd4, prepare_budget = 16;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected unknown original GPIO input to fail");
}

struct Fixture {
  McuBus memory;
  Hc12CycleBus timed;
  Hc12 cpu;
  GpioNetDriver supervisor;
  bool passive = true;
  std::vector<BusEvent> writes;

  Fixture(const std::span<const std::uint8_t> flash, const std::filesystem::path &specification)
      : memory(flash), timed(memory, [](const auto &) {}), cpu(timed, specification) {
    memory.attach_gpio_pad_source(McuPort::u, [&](const auto mask) {
      const auto outputs = memory.gpio_outputs(McuPort::u);
      const auto bias = memory.gpio_biases(McuPort::u);
      auto external = supervisor;
      external.active &= mask;
      external.known &= mask;
      external.levels &= mask;
      const std::array drivers{GpioNetDriver{.active = outputs.mask & mask,
                                             .known = outputs.known & mask,
                                             .levels = outputs.levels & mask,
                                             .reduced = outputs.reduced_drive & mask,
                                             .pull_up = bias.up & mask,
                                             .pull_down = bias.down & mask},
                               GpioNetDriver{.pull_up = passive ? mask & 1U : 0U},
                               external};
      return resolve_gpio_net(mask, drivers);
    });
    memory.set_observer([&](const auto &event) {
      if (event.write) { writes.push_back(event); }
    });
  }

  void at_release() {
    cpu.set_state({.pc = release,
                   .d = accumulator,
                   .x = index_x,
                   .y = index_y,
                   .sp = stack,
                   .ccr = hc12::reset_ccr});
    const auto before = timed.cycles();
    static_cast<void>(cpu.step());
    const auto result = cpu.state();
    require(result.pc == poll && result.d == accumulator && result.x == index_x &&
                result.y == index_y && result.sp == stack && result.ccr == release_flags &&
                timed.cycles() - before == 3 && memory.read(direction_u) == 0,
            "Original CLR DDRU preserves the literal registers and sets NZVC=0100 in three cycles");
  }

  void branch(const unsigned destination) {
    const auto before = timed.cycles();
    static_cast<void>(cpu.step());
    const auto result = cpu.state();
    require(result.pc == destination && result.d == accumulator && result.x == index_x &&
                result.y == index_y && result.sp == stack && result.ccr == release_flags &&
                timed.cycles() - before == 5,
            "Original BRCLR consumes five cycles and preserves all literal registers/flags");
  }
};

void original_preparation(Fixture &f) {
  f.cpu.set_state({.pc = prepare, .sp = stack, .ccr = hc12::reset_ccr});
  for (unsigned count = 0; f.cpu.state().pc != prepare_return && count < prepare_budget; ++count) {
    static_cast<void>(f.cpu.step());
  }

  struct Write {
    unsigned address;
    unsigned value;
  };

  const std::array expected{Write{.address = hc12::ptu, .value = 0},
                            Write{.address = hc12::ddrj, .value = 0xff},
                            Write{.address = hc12::ptj, .value = 0xff},
                            Write{.address = direction_u, .value = 1},
                            Write{.address = hc12::portb, .value = 6},
                            Write{.address = hc12::ddrb, .value = 0x16},
                            Write{.address = hc12::ptj, .value = 1}};
  require(
      f.cpu.state().pc == prepare_return && f.writes.size() == expected.size(),
      "Complete original preparation reaches RTS with exactly seven expected peripheral writes");
  for (unsigned index = 0; index < expected.size(); ++index) {
    require(f.writes.at(index).address == expected.at(index).address &&
                f.writes.at(index).value == expected.at(index).value,
            "Original preparation matches the independent ordered latch/direction ledger");
  }
  require(f.memory.read_masked_byte({.address = pin_u, .mask = 1}) == 0,
          "Original preparation actively holds the declared passive reset net low");
}

void ordinary_release(const Firmware &firmware, const std::filesystem::path &specification) {
  Fixture f(firmware.mcu, specification);
  original_preparation(f);
  f.at_release();
  f.branch(continuation);
  static_cast<void>(rejects([&] { static_cast<void>(f.memory.read(hc12::ptu)); }));
  std::cout << "Original prepare/U0-release/poll: ordered seven writes, passive high exit; "
               "other U inputs remain unknown\n";
}

void held_release(const Firmware &firmware, const std::filesystem::path &specification) {
  Fixture f(firmware.mcu, specification);
  original_preparation(f);
  f.supervisor = {.active = 1, .known = 1};
  f.at_release();
  f.branch(poll);
  f.branch(poll);
  f.supervisor = {}; // Explicit external event between completed native instructions.
  f.branch(continuation);
  std::cout << "Original poll remains low while external reset is asserted; exits after release\n";
}

void unknown_release(const Firmware &firmware,
                     const std::filesystem::path &specification,
                     const bool passive) {
  Fixture f(firmware.mcu, specification);
  original_preparation(f);
  f.passive = passive;
  if (passive) { f.supervisor = {.active = 1}; }
  f.at_release();
  const auto reason = rejects([&] { static_cast<void>(f.cpu.step()); });
  require(reason.contains("0x0278") && reason.contains("0xb98d") && reason.contains("mask=0x01"),
          "Original poll refuses unknown selected U0 with actual register address and native PC");
  std::cout << "Original unknown U0 refused: " << reason << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-gpio-reset firmware.zip pinned-HC12.sla");
    const auto firmware = Firmware::load(arguments[1]);
    const std::filesystem::path specification(arguments[2]);
    ordinary_release(firmware, specification);
    held_release(firmware, specification);
    unknown_release(firmware, specification, false);
    unknown_release(firmware, specification, true);
    std::cout << "Conditional original GPIO routines pass; no physical wiring, DSP execution, "
                 "synchronizer timing or complete boot claim\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
