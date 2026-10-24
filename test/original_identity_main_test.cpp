// SPDX-License-Identifier: GPL-3.0-only
#include "application_fixture.hpp"
#include "application_midi_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/lcd_export.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/private_file.hpp"
#include "blofeld/virtual_device_state.hpp"
#include "lcd_fixture.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned after_serial = 0x84f9, draw_text = 0xa6ee;
constexpr unsigned bad_dsp_text = 0xfc0b, bad_serial_text = 0xfc3c;
constexpr unsigned serial_slot = 6, dsp_slot = 8, word_shift = 8;
constexpr unsigned expected_rti = 6384, maximum_steps = 14000000;
constexpr unsigned main_resume = 0x8315, waiting_stack = 0x25ea;
constexpr unsigned maximum_frame_bytes = 16384;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

unsigned word(const McuBus &bus, const unsigned address) {
  return (static_cast<unsigned>(bus.peek(static_cast<std::uint16_t>(address))) << word_shift) |
         bus.peek(static_cast<std::uint16_t>(address + 1));
}

struct Diagnostics {
  unsigned serial_checks = 0;
  unsigned dsp_draws = 0;
  unsigned serial_draws = 0;

  void observe(const McuMachine &machine, const test::LcdFixture & /*display*/) {
    const auto &bus = machine.bus();
    const auto state = machine.cpu().state();
    if (state.pc == after_serial) {
      ++serial_checks;
      require(bus.peek(static_cast<std::uint16_t>(state.sp + serial_slot)) == 0 &&
                  bus.peek(static_cast<std::uint16_t>(state.sp + dsp_slot)) != 0,
              "Real startup reads available factory identity while the missing DSP stays failed");
    }
    if (state.pc != draw_text) { return; }
    const auto text = word(bus, state.sp + 2U);
    if (text == bad_dsp_text) { ++dsp_draws; }
    if (text == bad_serial_text) { ++serial_draws; }
  }
};

struct FrameOutput {
  std::filesystem::path directory;
  std::filesystem::path artifact;
};

void publish_frame(const McuMachine &machine,
                   const test::LcdFixture &display,
                   const FrameOutput &output) {
  const auto before = machine.cpu().state();
  const auto revision = machine.bus().revision();
  const auto image = capture_lcd(display.controller(), {.width = 128, .reverse_commons = true});
  const auto pixels = encode_lcd_pbm(image);
  LcdPublisher publisher(output.directory);
  require(publisher.publish(image) && !publisher.publish(image),
          "The actual complete main-WAI LCD frame is published once without duplicate I/O");
  const auto after = machine.cpu().state();
  require(read_private_file(output.directory / "display.pbm", maximum_frame_bytes) == pixels &&
              after.pc == before.pc && after.sp == before.sp && after.d == before.d &&
              after.x == before.x && after.y == before.y && after.ccr == before.ccr &&
              machine.bus().revision() == revision && machine.cpu().waiting(),
          "Host LCD publication leaves the inactive native MCU and bus unchanged");
  write_private_file(output.artifact, pixels);
}

struct Arguments {
  std::filesystem::path reference;
  std::filesystem::path trace;
  std::filesystem::path directory;
  bool factory = false;
};

void run(const Arguments &arguments) {
  const auto firmware = Firmware::load(arguments.reference);
  const auto original = McuFlashImage::from_known(firmware.mcu);
  std::unique_ptr<VirtualDeviceState> device;
  if (arguments.factory) {
    device = std::make_unique<VirtualDeviceState>(arguments.directory / "unit.bfnvm", original);
  }
  const auto initial = device ? device->image() : original;
  McuMachine machine(initial, BLOFELD_DEFAULT_SLA);
  auto display = std::make_unique<test::LcdFixture>(machine.bus());
  std::ofstream trace(arguments.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Diagnostics diagnostic;
  const auto result =
      arguments.factory
          ? test::start_application_reference(
                machine,
                display,
                trace,
                false,
                maximum_steps,
                [&diagnostic](const auto &mcu, const auto &lcd) { diagnostic.observe(mcu, lcd); })
          : test::start_application(machine, display, trace, false);
  require(
      !machine.clocked() && machine.cpu().waiting() && machine.cpu().state().pc == main_resume &&
          machine.cpu().state().sp == waiting_stack && result.interrupts == expected_rti &&
          result.oscillator_periods == expected_rti &&
          result.before_input_interrupts == expected_rti && result.before_input_instructions != 0,
      "Original main WAI and explicit reference RTI counts remain independently checked");
  if (arguments.factory) {
    require(
        diagnostic.serial_checks == 1 && diagnostic.dsp_draws == 1 && diagnostic.serial_draws == 0,
        "A complete cold main-loop start retains the genuine DSP warning and no serial warning");
  }
  publish_frame(machine,
                *display,
                {.directory = arguments.directory, .artifact = arguments.trace.string() + ".pbm"});
  if (arguments.factory) { test::verify_application_midi(machine.bus(), machine.cpu(), trace); }
  const auto retained = machine.bus().power_off_flash_image();
  require(retained.bytes == initial.bytes && retained.known == initial.known &&
              retained.fresh == initial.fresh,
          "Main WAI, LCD publication and native SCI foreground/ISRs retain complete unit NVM");
  if (device) { device->save(retained); }
  trace << "identity-main factory=" << arguments.factory << " instructions=" << result.instructions
        << " before-input=" << result.before_input_instructions
        << " interrupts=" << result.interrupts << " pc=" << hex(machine.cpu().state().pc) << '\n';
  std::cout << "Original identity main factory=" << arguments.factory
            << ": native WAI, strict input recovery, LCD publication and retained NVM passed.\n";
  if (arguments.factory) {
    std::cout << "Original SCI:264 fixed waveform bytes use actual WAI/ISR/foreground paths.\n";
  }
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 5, "Expected original archive, private trace/directory and baseline|factory");
    const std::string_view mode(arguments[4]);
    require(mode == "baseline" || mode == "factory", "Unknown identity startup test mode");
    run({.reference = arguments[1],
         .trace = arguments[2],
         .directory = arguments[3],
         .factory = mode == "factory"});
    std::cout << "Conditional MCU/LCD/SCI experiment only; complete DSP/profile/audio/update/"
                 "recovery and physical hardware equivalence remain unqualified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
