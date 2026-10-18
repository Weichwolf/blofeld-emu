// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"

#include <charconv>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>

namespace {

struct Options {
  std::filesystem::path firmware;
  std::filesystem::path specification;
  std::filesystem::path trace;
  std::uint64_t instructions = 1000000;
  bool inspect = false;
};

Options options(const int count, char **values) {
  Options result;
  for (int index = 1; index < count; ++index) {
    const std::string_view argument(values[index]);
    if (argument == "--inspect") {
      result.inspect = true;
      continue;
    }
    if (index + 1 >= count) { throw blofeld::Fault("Missing value for " + std::string(argument)); }
    const std::string_view value(values[++index]);
    if (argument == "--firmware") {
      result.firmware = value;
    } else if (argument == "--sla") {
      result.specification = value;
    } else if (argument == "--trace") {
      result.trace = value;
    } else if (argument == "--instructions") {
      const auto conversion =
          std::from_chars(value.data(), value.data() + value.size(), result.instructions);
      if (conversion.ec != std::errc{} || conversion.ptr != value.data() + value.size()) {
        throw blofeld::Fault("Invalid instruction limit");
      }
    } else {
      throw blofeld::Fault("Unknown argument: " + std::string(argument));
    }
  }
  if (result.firmware.empty()) {
    throw blofeld::Fault("Usage: blofeld-headless --firmware update.zip [--inspect] "
                         "[--instructions N] [--trace file]");
  }
  return result;
}

std::filesystem::path default_specification() {
  std::error_code error;
  const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error) {
    const auto installed = executable.parent_path().parent_path() / "share/blofeld-emu/HC12.sla";
    if (std::filesystem::is_regular_file(installed)) { return installed; }
  }
  return BLOFELD_DEFAULT_SLA;
}

void print_state(std::ostream &output, const blofeld::CpuState &state) {
  output << "pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
         << " x=" << blofeld::hex(state.x) << " y=" << blofeld::hex(state.y)
         << " sp=" << blofeld::hex(state.sp) << " ccr=" << blofeld::hex(state.ccr, 2);
}

int run(const Options &config) {
  const auto firmware = blofeld::Firmware::load(config.firmware);
  std::cout << "Firmware: " << firmware.mcu.size() << " MCU bytes, " << firmware.dsp.size()
            << " DSP records, " << firmware.sysex_packets << " checked data packets\n";
  if (config.inspect) { return 0; }
  std::ofstream trace;
  if (!config.trace.empty()) {
    trace.open(config.trace);
    if (!trace) { throw blofeld::Fault("Cannot create trace file"); }
    trace.exceptions(std::ios::failbit | std::ios::badbit);
  }
  blofeld::McuBus bus(firmware.mcu);
  bus.set_observer([&trace](const blofeld::BusEvent &event) {
    if (trace.is_open()) {
      trace << "  io-write " << blofeld::hex(event.address) << '=' << blofeld::hex(event.value, 2)
            << ' ' << event.name << '\n';
    }
  });
  const auto specification =
      config.specification.empty() ? default_specification() : config.specification;
  blofeld::Hc12 cpu(bus, specification);
  cpu.reset();
  std::uint64_t executed = 0;
  try {
    for (; executed < config.instructions; ++executed) {
      if (trace.is_open()) {
        trace << executed << ' ';
        print_state(trace, cpu.state());
        trace << ' ' << cpu.disassemble() << '\n';
      }
      cpu.step();
    }
  } catch (const blofeld::Fault &fault) {
    if (trace.is_open()) { trace.flush(); }
    std::cerr << "Stopped after " << executed << " completed MCU instructions: " << fault.what()
              << "\n";
    print_state(std::cerr, cpu.state());
    std::cerr << '\n';
    return 2;
  }
  if (trace.is_open()) { trace.flush(); }
  std::cout << "Instruction budget reached; boot completion is not established.\n";
  print_state(std::cout, cpu.state());
  std::cout << '\n';
  return 0;
}

} // namespace

int main(const int argc, char **argv) {
  try {
    return run(options(argc, argv));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
