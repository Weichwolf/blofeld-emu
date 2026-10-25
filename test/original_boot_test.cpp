// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {
constexpr unsigned application_entry = 0x8200;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned clear_start = 0x8078;
constexpr unsigned clear_end = 0x807d;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned ram_end = 0x2600;
constexpr unsigned instruction_limit = 200000;
constexpr unsigned fixture_inputs = 0xfb;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void verify_ram_clear(blofeld::McuBus &bus, const blofeld::CpuState state) {
  require(state.x == ram_end && state.y == 0, "Original RAM loop completes with X=2600 and Y=0");
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    require(bus.read(static_cast<std::uint16_t>(address)) == 0,
            "Original reset loop initializes every SMRAM/QRAM byte");
  }
}

void verify_cold_ram(blofeld::McuBus &bus) {
  bool unknown = false;
  try {
    static_cast<void>(bus.read(ram_begin));
  } catch (const blofeld::Fault &) { unknown = true; }
  require(unknown, "RAM is unknown before the original clearing loop");
}

void print_state(std::ostream &stream, const blofeld::CpuState state) {
  stream << " pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
         << " x=" << blofeld::hex(state.x) << " y=" << blofeld::hex(state.y)
         << " sp=" << blofeld::hex(state.sp) << " ccr=" << blofeld::hex(state.ccr, 2);
}

struct BootInputs {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

int run_reference(const BootInputs &inputs) {
  const auto &reference = inputs.reference;
  const auto &trace_path = inputs.trace;
  if (!std::filesystem::is_regular_file(reference)) {
    std::cout << "SKIP original bootloader execution: private firmware is absent\n";
    constexpr int skipped_reference = 77;
    return skipped_reference;
  }
  const auto firmware = blofeld::Firmware::load(reference);
  blofeld::McuBus bus(firmware.mcu);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = fixture_inputs});
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  std::ofstream trace;
  if (!trace_path.empty()) {
    const auto parent = trace_path.parent_path();
    if (!parent.empty()) { std::filesystem::create_directories(parent); }
    trace.open(trace_path);
    if (!trace) { throw blofeld::Fault("Cannot create private boot trace"); }
    trace.exceptions(std::ios::failbit | std::ios::badbit);
  }
  bool ram_checked = false;
  bool ram_verified = false;
  bool phy_sampled = false;
  unsigned completed = 0;
  while (completed < instruction_limit && cpu.state().pc != application_entry) {
    const auto before = cpu.state();
    if (before.pc == phy_poll && !phy_sampled) {
      bus.observe_phy({.edges = 15}); // Explicit completed detector window, not analog startup.
      phy_sampled = true;
    }
    if (before.pc == clear_start && !ram_checked) {
      verify_cold_ram(bus);
      ram_checked = true;
    }
    if (before.pc == clear_end) {
      verify_ram_clear(bus, before);
      ram_verified = true;
    }
    if (trace.is_open()) {
      trace << completed;
      print_state(trace, before);
      trace << ' ' << cpu.disassemble() << '\n';
    }
    try {
      cpu.step();
    } catch (const blofeld::Fault &fault) {
      std::cerr << "Original boot fixture after " << completed << " instructions:";
      print_state(std::cerr, before);
      std::cerr << ' ' << fault.what() << '\n';
      throw;
    }
    ++completed;
  }
  require(ram_checked && ram_verified && cpu.state().pc == application_entry,
          "Original bootloader must reach the application, not merely exhaust its budget");
  require(cpu.state().sp == ram_end && cpu.state().d == 0,
          "Original flash checksum and return stack complete without host substitution");
  constexpr unsigned timer_prescaler = 0x4d;
  require(bus.read(timer_prescaler) == 6, "Original firmware selects a divide-by-64 timer clock");
  std::cout << "Untimed MCU boot fixture reached the original application after " << completed
            << " instructions; RAM clear/checksum verified. No scheduled board boot is proven.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count < 2 || count > 3) {
      throw blofeld::Fault("Usage: boot-test firmware.zip [private-trace]");
    }
    const std::filesystem::path trace_path = count == 3 ? arguments[2] : "";
    return run_reference({.reference = arguments[1], .trace = trace_path});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
