// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "parallel_update_fixture.hpp"
#include "storage/device_identity.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned after_serial = 0x84f9, status_return = 0x85b7, draw_text = 0xa6ee;
constexpr unsigned bad_dsp_text = 0xfc0b, bad_serial_text = 0xfc3c;
constexpr unsigned serial_slot = 6, dsp_slot = 8, serial_bad = 0x80, word_shift = 8;
constexpr unsigned ready_poll = 0xb98d, display_initialization = 0x8b94;
constexpr unsigned release_inputs = 0x820e, startup_poll = 0x869d;
constexpr unsigned instruction_limit = 12046302;
constexpr unsigned byte_mask = 255, text_mode = 2, screen_bytes = 1024;
constexpr unsigned ready_inputs = 0x3f, released_inputs = 0x1e;
constexpr unsigned application_e_inputs = 0xeb, other_b_inputs = 0xe8, rti_vector = 0xfff0;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

unsigned word(McuBus &bus, const unsigned address) {
  return (static_cast<unsigned>(bus.read(static_cast<std::uint16_t>(address))) << word_shift) |
         bus.read(static_cast<std::uint16_t>(address + 1));
}

void inputs(McuBus &bus, const unsigned pc) {
  // Existing explicit untimed application GPIO fixture; no peer response source.
  if (pc == ready_poll) { bus.set_pin_drive(McuPort::u, {.mask = ready_inputs, .levels = 1}); }
  if (pc == display_initialization) { bus.set_pin_drive(McuPort::b, {.mask = 8}); }
  if (pc == release_inputs) { bus.set_pin_drive(McuPort::b, {.mask = released_inputs}); }
}

struct Diagnostics {
  unsigned dsp_calls = 0;
  unsigned serial_calls = 0;
  unsigned instructions = 0;
  unsigned interrupts = 0;
  bool serial_observed = false;

  void observe(McuBus &bus, const Hc12 &cpu, const bool unavailable) {
    const auto state = cpu.state();
    if (state.pc == after_serial) {
      require(!serial_observed, "The real startup serial decision executes once per cold start");
      serial_observed = true;
      require(bus.read(static_cast<std::uint16_t>(state.sp + serial_slot)) ==
                      (unavailable ? serial_bad : 0) &&
                  bus.read(static_cast<std::uint16_t>(state.sp + dsp_slot)) != 0,
              "Original startup chooses its serial marker and retains an actual DSP failure");
    }
    if (state.pc != draw_text) { return; }
    const auto pointer = word(bus, state.sp + 2U);
    if (pointer == bad_dsp_text || pointer == bad_serial_text) {
      require((state.d & byte_mask) == text_mode,
              "Original diagnostic rendering uses its actual stack-passed text and mode");
    }
    if (pointer == bad_dsp_text) { ++dsp_calls; }
    if (pointer == bad_serial_text) { ++serial_calls; }
  }
};

void cold_start(VirtualDeviceState &device,
                const std::filesystem::path &artifact,
                const bool unavailable,
                std::ostream &trace) {
  const auto initial = device.image();
  McuBus bus(initial);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  test::LcdFixture display(bus);
  test::boot(bus, cpu, trace); // Actual original reset vector, loopback and checksum.
  bus.set_pin_drive(McuPort::q, {.floating = byte_mask});
  bus.observe_usb_decoder({});
  bus.set_pin_drive(McuPort::e, {.mask = application_e_inputs, .levels = 3, .floating = 0x10});
  bus.set_pin_drive(McuPort::b, {.mask = other_b_inputs});
  bus.set_observer([&display](const BusEvent &event) { display.observe(event); });
  Diagnostics observed;
  bool deliver_rti = true;
  for (unsigned steps = 0; cpu.state().pc != status_return; ++steps) {
    require(steps < instruction_limit, "Cold startup must reach its own diagnostic return");
    const auto pc = cpu.state().pc;
    inputs(bus, pc);
    observed.observe(bus, cpu, unavailable);
    if (pc == startup_poll && deliver_rti && !cpu.pending_interrupt()) {
      const auto periods = bus.oscillator_cycles_until_rti();
      require(periods != 0, "Original startup has configured a real RTI source");
      bus.advance_oscillator(periods);
      deliver_rti = false;
    }
    const auto result = cpu.step();
    if (result.instruction_executed) {
      ++observed.instructions;
      if (pc == startup_poll) { deliver_rti = true; }
    }
    if (result.interrupt_vector) {
      require(*result.interrupt_vector == rti_vector,
              "Only the declared native RTI source is admitted");
      ++observed.interrupts;
    }
  }
  require(observed.serial_observed && observed.dsp_calls == 1 &&
              observed.serial_calls == (unavailable ? 1U : 0U),
          "Actual reset-to-render execution draws bad DSP and only draws an unavailable serial");
  const auto image = capture_lcd(display.controller(), {.width = 128, .reverse_commons = true});
  const auto bitmap = encode_lcd_pbm(image);
  require(display.counts().writes > screen_bytes && encode_lcd_text(image).contains('#'),
          "The original warning renderer produces actual known, nonblank LCD pixels");
  write_private_file(artifact, bitmap);
  const auto retained = bus.power_off_flash_image();
  require(retained.bytes == initial.bytes && retained.known == initial.known &&
              retained.fresh == initial.fresh,
          "Native startup preserves all factory/program cells and NVM history");
  device.save(retained);
  trace << "cold unavailable=" << unavailable << " instructions=" << observed.instructions
        << " interrupts=" << observed.interrupts << " dsp-draws=" << observed.dsp_calls
        << " serial-draws=" << observed.serial_calls << '\n';
  std::cout << "Cold native startup unavailable=" << unavailable << ": DSP warning retained, "
            << "serial warning=" << observed.serial_calls << ", NVM preserved.\n"
            << std::flush;
}

struct Arguments {
  std::filesystem::path reference;
  std::filesystem::path trace;
  std::filesystem::path state;
};

void run(const Arguments &arguments) {
  const auto firmware = Firmware::load(arguments.reference);
  const auto original = McuFlashImage::from_known(firmware.mcu);
  std::ofstream trace(arguments.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  unsigned generations = 0;
  const auto factory = [&] {
    ++generations;
    return std::string("EMU-COLD0001");
  };
  for (unsigned restart = 0; restart < 3; ++restart) {
    VirtualDeviceState device(arguments.state, original, factory);
    require(generations == 1, "Only the first cold unit creation allocates its serial");
    if (restart == 2) {
      auto damaged = device.image();
      constexpr auto offset = device_identity::serial_address - 0x8000;
      damaged.bytes[offset] |= serial_bad; // Explicit own factory-data fault; no RAM/PC patch.
      device.save(damaged);
    }
    const auto artifact = arguments.trace.string() + "." + std::to_string(restart) + ".pbm";
    cold_start(device, artifact, restart == 2, trace);
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original archive, private trace and new private unit path");
    run({.reference = arguments[1], .trace = arguments[2], .state = arguments[3]});
    std::cout << "Conditional original reset/diagnostic rendering passed; full DSP boot, normal "
                 "runtime and physical board equivalence remain unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
