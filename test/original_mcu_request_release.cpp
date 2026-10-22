// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <string_view>

namespace {
constexpr unsigned oscillator = 12000000;
constexpr unsigned boot_inputs = 0xfb;
constexpr unsigned first_loopback = 0x8029, second_loopback = 0x8036, phy_poll = 0x8049;
constexpr unsigned ram_begin = 0x1200, ram_end = 0x2600, ram_canary = 0xa5;
constexpr unsigned caller = 0x1400, stack = 0x2200, request = 0xb6dc;
constexpr unsigned bus_flags = 0x1815, setup_flags = 0x21;
constexpr unsigned instruction_limit = 1024, byte_mask = 255;
constexpr unsigned x_seed = 0x5678, y_seed = 0x9abc;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw blofeld::Fault(std::string(message)); }
}

struct Input {
  unsigned kind = 0;
  unsigned location = 0;
  unsigned parameter = 0;
};

constexpr std::array<Input, 8> inputs{{{},
                                       {.kind = 1, .location = 0x40},
                                       {.kind = 1},
                                       {.kind = 2},
                                       {.location = 15, .parameter = 511},
                                       {.kind = 1, .location = 0x4f, .parameter = 63},
                                       {.kind = 1, .location = 7, .parameter = 24},
                                       {.kind = 2, .parameter = 0x4c}}};

void clock_prefix(blofeld::McuMachine &machine, std::ostream &trace) {
  auto &bus = machine.bus();
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = boot_inputs, .levels = 3});
  bool observed = false;
  unsigned steps = 0;
  while (bus.run_clock_route().source != blofeld::McuClockSource::usb_phy) {
    require(steps++ < instruction_limit, "Original MCU clock-prefix bound");
    const auto pc = machine.cpu().state().pc;
    if (pc == first_loopback || pc == second_loopback) {
      const auto sample = bus.observe_pads(blofeld::McuPort::e, 4);
      require(sample.known == 4, "Declared E2/E3 loopback must have a known source");
      bus.set_pin_drive(
          blofeld::McuPort::e,
          {.mask = boot_inputs,
           .levels = static_cast<std::uint8_t>(3U | ((sample.levels & 4U) != 0 ? 8U : 0U))});
    }
    if (pc == phy_poll && !observed) {
      bus.observe_phy({.edges = 15});
      observed = true;
    }
    machine.cpu().step();
  }
  require(observed, "Original clock selection must consume the declared PHY observation");
  trace << "PREFIX pc=" << machine.cpu().state().pc << " instructions=" << steps
        << " cycles=" << machine.run_clock().bus_cycles()
        << " ticks=" << machine.run_clock().elapsed_ticks()
        << " timebase=" << machine.run_clock().timebase_hz()
        << " divider=" << bus.run_clock_route().bus_divider << '\n';
}

void prepare(blofeld::McuMachine &machine, const Input input) {
  auto &bus = machine.bus();
  bus.write(blofeld::hc12::initee, setup_flags);
  bus.retire_instruction();
  for (unsigned address = ram_begin; address < ram_end; ++address) {
    bus.write(static_cast<std::uint16_t>(address), ram_canary);
  }
  bus.write(blofeld::hc12::ptj, 1);
  bus.write(blofeld::hc12::ddrj, 7);
  bus.write(blofeld::hc12::ptt, 1);
  bus.write(blofeld::hc12::ddrt, 15);
  bus.write(blofeld::hc12::ptq, 0);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  bus.set_pin_drive(blofeld::McuPort::q, {.floating = byte_mask});
  bus.write(bus_flags, setup_flags);
  bus.write(stack, static_cast<std::uint8_t>(input.location));
  bus.write(stack + 1, static_cast<std::uint8_t>(input.kind));
  // Own native JSR supplies the real caller frame; original instructions are untouched.
  constexpr std::array<std::uint8_t, 3> code{0x16, request >> 8, request & byte_mask};
  for (unsigned index = 0; index < code.size(); ++index) {
    bus.write(static_cast<std::uint16_t>(caller + index), code[index]);
  }
  machine.cpu().set_state({.pc = caller,
                           .d = static_cast<std::uint16_t>(input.parameter),
                           .x = x_seed,
                           .y = y_seed,
                           .sp = stack,
                           .ccr = 0x40});
}

void run(const blofeld::Firmware &firmware, const Input input, std::ostream &trace) {
  blofeld::McuMachine machine(
      blofeld::McuFlashImage::from_known(firmware.mcu),
      BLOFELD_DEFAULT_SLA,
      blofeld::McuRunContract{
          .clock = {.oscillator_hz = oscillator,
                    .switch_timing = blofeld::McuRunClockSwitch::reference_cycle_boundary},
          .memory_timing = blofeld::McuRunMemory::internal_flash_reads});
  clock_prefix(machine, trace);
  prepare(machine, input);
  const auto nvm = machine.bus().power_off_flash_image();
  bool output = false;
  bool released = false;
  unsigned data_reads = 0;
  unsigned steps = 0;
  machine.bus().set_observer([&](const blofeld::BusEvent &event) {
    if (!event.write && event.address == blofeld::hc12::ptq) { ++data_reads; }
    if (!event.write ||
        (event.address != blofeld::hc12::ptq && event.address != blofeld::hc12::ddrq &&
         event.address != blofeld::hc12::ptj)) {
      return;
    }
    if (event.address == blofeld::hc12::ddrq) {
      output = output || event.value != 0;
      released = output && event.value == 0;
    }
    const auto pins = machine.bus().gpio_outputs(blofeld::McuPort::q);
    trace << "WRITE pc=" << machine.cpu().state().pc << " address=" << event.address
          << " value=" << static_cast<unsigned>(event.value)
          << " cycles=" << machine.run_clock().bus_cycles()
          << " ticks=" << machine.run_clock().elapsed_ticks()
          << " mask=" << static_cast<unsigned>(pins.mask)
          << " known=" << static_cast<unsigned>(pins.known)
          << " levels=" << static_cast<unsigned>(pins.levels) << '\n';
  });
  while (!released) {
    require(steps++ < instruction_limit, "Original MCU release-prefix bound");
    trace << "STEP pc=" << machine.cpu().state().pc
          << " cycles=" << machine.run_clock().bus_cycles()
          << " ticks=" << machine.run_clock().elapsed_ticks() << '\n';
    machine.cpu().step();
  }
  machine.bus().set_observer({});
  const auto final = machine.bus().power_off_flash_image();
  require(nvm.bytes == final.bytes && nvm.known == final.known && nvm.fresh == final.fresh &&
              !machine.timing_faulted(),
          "Original request prefix must retain NVM and healthy execution");
  trace << "RELEASE pc=" << machine.cpu().state().pc
        << " cycles=" << machine.run_clock().bus_cycles()
        << " ticks=" << machine.run_clock().elapsed_ticks() << " data_reads=" << data_reads
        << " nvm_unchanged=1\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware and private MCU release trace");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    unsigned index = 0;
    for (const auto input : inputs) {
      trace << "CASE index=" << index++ << " kind=" << input.kind << " location=" << input.location
            << " parameter=" << input.parameter << '\n';
      run(firmware, input, trace);
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
