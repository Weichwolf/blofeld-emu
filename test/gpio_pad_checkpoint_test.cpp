// SPDX-License-Identifier: GPL-3.0-only
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>

namespace {
using blofeld::test::require;
constexpr unsigned entry = 0x8000, stack = 0x2000, seeded_data = 0x1234;
constexpr unsigned first_mcu = 0xa5, next_mcu = 0x5a, first_dsp = 0x5a5, next_dsp = 0xa5a;
constexpr unsigned initial_flags = 0xd0, negative_flags = 0xd8;
constexpr unsigned extal = 12000000, timebase = 540000000;
constexpr unsigned byte_mask = 255, byte_bits = 8, word_mask = 0xffffff;
constexpr unsigned reset_vector_offset = 0x7ffe, protection_offset = 0x7f0d;
constexpr unsigned security_offset = 0x7f0f, unsecured = 0xfe;
constexpr unsigned dsp_input_mask = 0xfff;

void rejects(const auto &operation, const std::string_view label) {
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    require(std::string_view(error.what()).contains(label),
            "Unexpected native pad-checkpoint failure: " + std::string(error.what()));
    return;
  }
  throw blofeld::Fault("Missing native pad-checkpoint rejection");
}

blofeld::GpioPadReader reader(const std::shared_ptr<unsigned> &source) {
  return [source](const std::uint32_t bits) {
    return blofeld::GpioPadSample{.known = bits, .levels = *source & bits};
  };
}

blofeld::McuFlashImage mcu_image() {
  std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> image{};
  // Own HCS12 LDAA EXT instructions read the actual pin-input register twice.
  constexpr auto pins = blofeld::hc12::ptq + 1;
  constexpr std::array<std::uint8_t, 6> code{
      0xb6, pins >> byte_bits, pins & byte_mask, 0xb6, pins >> byte_bits, pins & byte_mask};
  for (unsigned index = 0; index < code.size(); ++index) { image.at(index) = code[index]; }
  image.at(reset_vector_offset) = entry >> byte_bits;
  image.at(protection_offset) = byte_mask;
  image.at(security_offset) = unsecured;
  return blofeld::McuFlashImage::from_known(image);
}

void check_mcu(blofeld::McuMachine &machine, const unsigned value, const unsigned offset) {
  machine.cpu().step();
  const auto actual = machine.cpu().state();
  const auto expected = (value << byte_bits) | (seeded_data & byte_mask);
  require(actual.pc == entry + offset && actual.d == expected && actual.sp == stack &&
              actual.ccr == (value == first_mcu ? negative_flags : initial_flags),
          "Native LDAA uses the freshly connected pin value and independent load flags");
}

void mcu_checkpoint() {
  blofeld::McuMachine machine(mcu_image(), BLOFELD_DEFAULT_SLA);
  machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  machine.cpu().set_state({.pc = entry, .d = seeded_data, .sp = stack, .ccr = initial_flags});
  auto source = std::make_shared<unsigned>(first_mcu);
  const std::weak_ptr<unsigned> lifetime(source);
  machine.bus().attach_gpio_pad_source(blofeld::McuPort::q, reader(source));
  source.reset();
  const auto saved = machine.checkpoint();
  const auto nvm = machine.bus().power_off_flash_image();
  require(lifetime.use_count() == 1, "MCU checkpoint must not own the live board callback");
  check_mcu(machine, first_mcu, 3);
  machine.restore(saved);
  require(lifetime.expired(), "Replacing the MCU owner releases its old board callback");
  rejects([&] { machine.cpu().step(); }, "Detached resolved GPIO pad source");
  rejects([&] { static_cast<void>(machine.checkpoint()); }, "requires a completed instruction");
  machine.restore(saved);
  const auto fresh = std::make_shared<unsigned>(next_mcu);
  machine.bus().attach_gpio_pad_source(blofeld::McuPort::q, reader(fresh));
  check_mcu(machine, next_mcu, 3);
  check_mcu(machine, next_mcu, 6);
  const auto after = machine.bus().power_off_flash_image();
  require(fresh.use_count() == 2 && nvm.bytes == after.bytes && nvm.known == after.known &&
              nvm.fresh == after.fresh,
          "Restored MCU uses only its fresh connection and preserves every NVM field");
}

blofeld::DspRunContract dsp_contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates,
          .resets = blofeld::DspResetTiming::reference_instruction_boundary};
}

void dsp_checkpoint() {
  blofeld::test::DspProgram program;
  program.emit("movep #>" + blofeld::test::number(dsp_input_mask) + ",x:<<$ffffbf");
  program.emit("movep x:<<$ffffbd,x0");
  program.emit("movep x:<<$ffffbd,x0");
  blofeld::DspMachine machine(dsp_contract());
  program.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  auto source = std::make_shared<unsigned>(first_dsp);
  const std::weak_ptr<unsigned> lifetime(source);
  machine.bus().gpio(blofeld::DspPort::c).attach_pad_source(reader(source));
  source.reset();
  machine.cpu().step();
  require(machine.bus().gpio(blofeld::DspPort::c).read(blofeld::DspGpioRegister::control) ==
              dsp_input_mask,
          "Own native setup selects actual GPIO input mode before the checkpoint");
  const auto saved = machine.checkpoint();
  require(lifetime.use_count() == 1, "DSP checkpoint must not own the live board callback");
  machine.cpu().step();
  require((machine.cpu().state().x & word_mask) == first_dsp,
          "The original owner consumes the independent first pad value");
  const auto expected = machine.cpu().debug_state();
  const auto ticks = machine.run_clock().elapsed_ticks();
  machine.restore(saved);
  require(lifetime.expired(), "Replacing the DSP owner releases its old board callback");
  rejects([&] { machine.cpu().step(); }, "Detached resolved GPIO pad source");
  require(machine.timing_faulted(), "A detached native read quarantines the clocked DSP");
  const auto failed = machine.cpu().debug_state();
  rejects([&] { machine.cpu().step(); }, "requires reset");
  require(machine.cpu().debug_state() == failed, "Quarantined retry changes no native state");
  rejects([&] { static_cast<void>(machine.checkpoint()); }, "requires a completed reference step");
  machine.restore(saved);
  const auto fresh = std::make_shared<unsigned>(first_dsp);
  machine.bus().gpio(blofeld::DspPort::c).attach_pad_source(reader(fresh));
  machine.cpu().step();
  require(machine.cpu().debug_state() == expected && !machine.timing_faulted() &&
              machine.run_clock().elapsed_ticks() == ticks,
          "A fresh source replays the native instruction and reference clock exactly");
  *fresh = next_dsp;
  machine.cpu().step();
  require((machine.cpu().state().x & word_mask) == next_dsp,
          "The following actual MOVEP consumes changed fresh-source input");
  const auto before_reset = machine.run_clock().elapsed_ticks();
  const auto epoch = machine.run_clock().native_epoch();
  machine.cpu().reset_spi_boot();
  require(machine.run_clock().elapsed_ticks() == before_reset &&
              machine.run_clock().native_epoch() == epoch + 1 && fresh.use_count() == 2,
          "Reference reset retains the live board source across a new zero-time native epoch");
  machine.cpu().step();
  machine.cpu().step();
  require((machine.cpu().state().x & word_mask) == next_dsp && !machine.timing_faulted(),
          "The actual post-reset native read still consumes the retained fresh connection");
}
}

int main() {
  try {
    mcu_checkpoint();
    dsp_checkpoint();
    std::cout << "Native MCU/DSP snapshots require fresh pad sources; live reset retains them.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
