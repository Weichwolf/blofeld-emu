// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/st7565.hpp"
#include "lcd_fixture.hpp"
#include "parallel_update_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

namespace {
constexpr unsigned byte_mask = 255;
constexpr unsigned caller = 0x1700;
constexpr unsigned stack = 0x2600;
constexpr unsigned first_service = 0x8d70;
constexpr unsigned callback_return = 0x1209;
constexpr unsigned helper_return = 0xbb56;
constexpr unsigned helper_bytes = 761;
constexpr unsigned normal_helper_instructions = 210871;
constexpr unsigned failed_helper_instructions = 43909;
constexpr unsigned lcd_data_read = 0x8b5a;
constexpr unsigned request_flags = 0x1815;
constexpr unsigned update_prepared = 0x40;
constexpr unsigned setup_limit = 2000000;
constexpr unsigned helper_limit = 250000;
constexpr unsigned caller_bytes = 9;
constexpr unsigned masked = 0xd0;
constexpr unsigned e_fixture_mask = 0xeb;
constexpr unsigned enable_pad = 0x10;
constexpr unsigned external_u_mask = 0x3f;
constexpr unsigned other_serial_inputs = 0xe8;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void setup(blofeld::McuBus &bus, blofeld::Hc12 &cpu, std::ostream &trace) {
  // Own caller invokes three complete original subroutines; no GPIO-direction stub.
  constexpr unsigned jsr = 0x16;
  constexpr std::array<unsigned, 3> entries{0xb938, 0xb955, 0x8b94};
  for (unsigned index = 0; index < entries.size(); ++index) {
    const auto address = static_cast<std::uint16_t>(caller + (3 * index));
    bus.write(address, jsr);
    bus.write(static_cast<std::uint16_t>(address + 1U),
              static_cast<std::uint8_t>(entries[index] >> 8U));
    bus.write(static_cast<std::uint16_t>(address + 2U),
              static_cast<std::uint8_t>(entries[index] & byte_mask));
  }
  cpu.set_state({.pc = caller, .sp = stack, .ccr = masked});
  unsigned instructions = 0;
  constexpr unsigned ready_poll = 0xb98d;
  constexpr unsigned display_initialization = 0x8b94;
  while (cpu.state().pc != caller + caller_bytes) {
    require(instructions < setup_limit,
            "Original peripheral setup must return through its own RTS");
    if (cpu.state().pc == ready_poll) {
      bus.set_pin_drive(blofeld::McuPort::u, {.mask = external_u_mask, .levels = 1});
    }
    if (cpu.state().pc == display_initialization) {
      bus.set_pin_drive(blofeld::McuPort::b, {.mask = 8});
    }
    const auto result = cpu.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Isolated original peripheral setup executes without injected interrupt entries");
    ++instructions;
  }
  constexpr unsigned original_direction = 0xf7;
  require(cpu.state().sp == stack && bus.read(blofeld::hc12::ddrb) == original_direction &&
              (bus.read(blofeld::hc12::ddre) & enable_pad) != 0,
          "Original setup establishes its own stack return and LCD GPIO directions");
  trace << "native-setup instructions=" << instructions << " ddrb=f7\n";
}

void check_clear(const blofeld::test::LcdFixture &display, const blofeld::test::LcdCounts before) {
  constexpr unsigned visible_columns = 128;
  constexpr unsigned visible_pages = 8;
  const auto counts = display.counts();
  require(counts.writes - before.writes == visible_columns * visible_pages &&
              counts.reads == before.reads,
          "Original first callback performs 1024 data writes and no display reads");
  for (unsigned page = 0; page < visible_pages; ++page) {
    for (unsigned column = 0; column < blofeld::St7565::columns; ++column) {
      const auto value = display.controller().ram(page, column);
      const unsigned expected_known = column < visible_columns ? byte_mask : 0;
      if (value.known != expected_known || value.value != 0) {
        throw blofeld::Fault("Original zero-clear mismatch page=" + blofeld::hex(page) +
                             " column=" + blofeld::hex(column) +
                             " value=" + blofeld::hex(value.value, 2) +
                             " known=" + blofeld::hex(value.known, 2));
      }
    }
  }
  for (unsigned column = 0; column < blofeld::St7565::columns; ++column) {
    require(display.controller().ram(8, column).known == 0,
            "Original callback leaves the icon page unknown");
  }
}

void execute_helper(blofeld::McuBus &bus,
                    blofeld::Hc12 &cpu,
                    blofeld::test::LcdFixture &display,
                    std::ostream &trace,
                    const bool withdraw) {
  require(cpu.state().pc == first_service, "Helper continuation starts at its original callback");
  const auto before = display.counts();
  if (withdraw) { display.withdraw_data(); }
  unsigned instructions = 0;
  bool cleared = false;
  while (cpu.state().pc != helper_return) {
    require(instructions < helper_limit, "Original helper must reach a bounded return or fault");
    const auto pc = cpu.state().pc;
    try {
      const auto result = cpu.step();
      require(result.instruction_executed && !result.interrupt_vector,
              "Original helper executes its own display and decompression instructions");
    } catch (const blofeld::Fault &fault) {
      const std::string_view message(fault.what());
      trace << "observed-fault instructions=" << instructions << ' ' << message << '\n';
      require(withdraw && cleared && pc == lcd_data_read &&
                  instructions == failed_helper_instructions &&
                  message.contains("MCU read at 0x0000") && message.contains("pc=0x8b5a") &&
                  message.contains("ccr=") && message.contains("mask=0xff") &&
                  display.counts().reads == before.reads,
              "Only the deliberately withdrawn LCD read may stop the original helper");
      trace << "withdrawn-display instructions=" << instructions << ' ' << message << '\n';
      return;
    }
    ++instructions;
    if (cpu.state().pc == callback_return) {
      require(!cleared && cpu.state().sp == stack - 14,
              "Original clear callback returns once with its real nested frame");
      check_clear(display, before);
      cleared = true;
    }
  }
  const auto counts = display.counts();
  // Pinned original drawing with the independently verified N=V BGE predicate.
  constexpr unsigned expected_commands = 416;
  constexpr unsigned expected_writes = 3078;
  constexpr unsigned expected_reads = 2060;
  require(!withdraw && cleared && instructions == normal_helper_instructions &&
              cpu.state().sp == stack - 10 && (bus.read(request_flags) & update_prepared) != 0 &&
              counts.commands == expected_commands && counts.writes == expected_writes &&
              counts.reads == expected_reads,
          "Complete original first helper returns through its own RTS after real display reads");
  trace << "native-helper instructions=" << instructions << " commands=" << counts.commands
        << " writes=" << counts.writes << " reads=" << counts.reads << " return=bb56\n";
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuBus bus(firmware.mcu);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original LCD helper trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  blofeld::test::boot(bus, cpu, trace);
  blofeld::test::LcdFixture display(bus);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = e_fixture_mask, .levels = 3});
  bus.set_pin_drive(blofeld::McuPort::b, {.mask = other_serial_inputs});
  bus.set_observer([&display](const blofeld::BusEvent &event) { display.observe(event); });
  setup(bus, cpu, trace);
  for (const bool withdraw : {false, true}) {
    blofeld::test::transfer(
        bus,
        cpu,
        {.payload = blofeld::test::original_helper(firmware.update_blocks.at(0), helper_bytes),
         .dispatch = !withdraw},
        trace);
    bus.set_pin_drive(blofeld::McuPort::e, {.mask = e_fixture_mask, .levels = 3});
    blofeld::test::check_first_callback(bus, cpu, trace);
    execute_helper(bus, cpu, display, trace, withdraw);
  }
  const auto image = bus.power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Original display-helper completion and withdrawn read preserve all MCU NVM/history");
  std::cout << "Original first helper: 1024-byte clear, LCD reads/drawing, native return and "
               "withdrawn-data fault verified. Explicit GPIO/entry/ready experiment; no full "
               "application/DSP/update boot or measured board timing.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-lcd-helper-test firmware.zip private.trace");
    }
    return run({.firmware = arguments[1], .trace = arguments[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
