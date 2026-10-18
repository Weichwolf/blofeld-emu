// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {
constexpr unsigned helper_base = 0x1200;
constexpr unsigned helper_bytes = 632;
constexpr unsigned initialization = 0x13b1;
constexpr unsigned status_access = 0x13bb;
constexpr unsigned stack = 0x2000;
constexpr unsigned divider = 0x100;
constexpr unsigned protection = 0x104;
constexpr unsigned security = 0x101;
constexpr unsigned options_field = 0xff0d;
constexpr unsigned security_field = 0xff0f;
constexpr unsigned masked_interrupts = 0xd0;
constexpr unsigned loaded_helper_divider = 0xc8;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Inputs {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.reference);
  const auto &main = firmware.update_blocks.at(4);
  const auto size = (static_cast<unsigned>(main.at(0)) << 8U) | main.at(1);
  require(size == helper_bytes,
          "The checked original update contains the identified HC12 RAM helper");
  blofeld::McuBus bus(firmware.mcu);
  for (std::size_t offset = 0; offset < size; ++offset) {
    bus.write(static_cast<std::uint16_t>(helper_base + offset), main.at(offset + 2));
  }
  const auto original_protection = bus.read(options_field);
  const auto original_security = bus.read(security_field);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state({.pc = initialization, .sp = stack, .ccr = masked_interrupts});
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original flash initialization trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  unsigned completed = 0;
  while (cpu.state().pc != status_access) {
    require(completed < 4,
            "Original flash configuration executes exactly its own four instructions");
    trace << completed << " pc=" << blofeld::hex(cpu.state().pc) << ' ' << cpu.disassemble()
          << '\n';
    cpu.step();
    ++completed;
  }
  require(completed == 4 && bus.read(divider) == loaded_helper_divider &&
              bus.read(protection) == original_protection,
          "Unmodified original helper latches FCLKDIV=48 and its permitted FPROT=ff request");
  require(
      bus.read(security) == original_security && bus.read(security_field) == original_security &&
          bus.read(options_field) == original_protection && cpu.state().sp == stack,
      "Isolated original initialization preserves security bytes, persistent options and stack");
  bus.reset();
  require(bus.read(divider) == 0 && bus.read(protection) == original_protection &&
              bus.read(security) == original_security,
          "Reset after original initialization reloads the supplied nonvolatile option fields");
  std::cout << "Original RAM flash-helper initialization executed four unchanged instructions; "
            << "stopped before FSTAT command-state access. This isolated entry fixture does not "
            << "prove update transport, program/erase, recovery or installed device security.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-flash-config-test firmware.zip private.trace");
    }
    return run({.reference = arguments[1], .trace = arguments[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
