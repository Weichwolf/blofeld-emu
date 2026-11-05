// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
using Register = DspGpioRegister;
constexpr auto functional = DspGpioContract::logical_pins;
constexpr unsigned pattern = 0xa55;

struct Port {
  DspPort kind;
  unsigned bits, movep_data, bset_data;
};

// Authored FM MOVEP immediate and BSET peripheral templates, with UM pin geometry.
constexpr std::array ports{
    Port{.kind = DspPort::c, .bits = 12, .movep_data = 0x07f43d, .bset_data = 0x013d20},
    Port{.kind = DspPort::e, .bits = 12, .movep_data = 0x07b49d, .bset_data = 0x011d60},
    Port{.kind = DspPort::f, .bits = 11, .movep_data = 0x09f4b5, .bset_data = 0x0ab560}};

void retained_pin(const Port port, const unsigned index, const bool high) {
  const auto bit = 1U << index;
  DspGpio gpio(port.kind);
  gpio.set_contract(functional);
  gpio.set_drive({.mask = bit, .levels = bit});
  require(rejects([&] { static_cast<void>(gpio.read(Register::data, bit)); }).contains("Unknown"),
          "An observed pad cannot invent unwritten disconnected PDR storage");
  gpio.set_drive({});
  gpio.write({.value = high ? bit : 0U, .mask = bit});
  require(gpio.read(Register::data, bit) == (high ? bit : 0U) && gpio.outputs().mask == 0 &&
              gpio.observe_pads(bit).known == 0,
          "Explicit disconnected storage reads back without creating a pin driver or pad level");
  auto copy = gpio;
  copy.reset();
  require(copy.read(Register::data, bit) == (high ? bit : 0U),
          "Nonreset PDR storage survives a trusted copy and peripheral reset");
  copy.set_contract(DspGpioContract::strict);
  require(rejects([&] { static_cast<void>(copy.read(Register::data, bit)); }).contains("Unknown"),
          "Strict C/E/F disconnected diagnostics retain their original refusal");
  gpio.set_drive({.mask = bit, .levels = high ? 0U : bit});
  require(gpio.read(Register::data, bit) == (high ? bit : 0U),
          "A disconnected register is distinct from the independently observed pad voltage");
  gpio.write({.reg = Register::control, .value = bit});
  require(gpio.read(Register::data, bit) == (high ? 0U : bit),
          "Input selection reads the actual opposite pad instead of retained output storage");
  gpio.set_drive({});
  require(rejects([&] { static_cast<void>(gpio.read(Register::data, bit)); }).contains("Unknown"),
          "Missing input data never falls back to a known output preload");
  gpio.write({.reg = Register::direction, .value = bit});
  require(rejects([&] { static_cast<void>(gpio.read(Register::data, bit)); }).contains("Unknown"),
          "A peripheral mux does not gain an invented GPIO readback");
}

class Run {
public:
  Run(const native::NativeProgram &program, const bool clocked) : clocked_(clocked) {
    for (const auto port : ports) { machine_.bus().gpio(port.kind).set_contract(functional); }
    native::reset_native(machine_, program);
    if (clocked) {
      timeline_ = std::make_unique<OwnDspTimeline>(
          machine_,
          OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts,
          0);
    }
  }

  void retire() {
    if (!clocked_) {
      native::step_native(machine_);
      return;
    }
    const auto target = machine_.cpu().progress().instructions + 1;
    constexpr unsigned maximum_cycles = 128;
    for (unsigned cycles = 0; cycles < maximum_cycles; ++cycles) {
      static_cast<void>(timeline_->advance_one(timeline_->next_deadline(),
                                               OwnDspBoundary::through,
                                               {.cycle = cycle_++,
                                                .timing = DspDmaTiming::reference_sampled_cycles,
                                                .grant = DspDmaGrant::available}));
      if (const auto &failure = machine_.io().failure()) {
        if (failure->cause) { std::rethrow_exception(failure->cause); }
        throw Fault("Native GPIO I/O refused without a captured exception");
      }
      require(!timeline_->faulted(), "The native clock owner remains healthy");
      if (machine_.cpu().progress().instructions == target) { return; }
    }
    throw Fault("Native disconnected BSET did not retire within its authored cycle bound");
  }

  [[nodiscard]] OwnDspMachine &machine() noexcept { return machine_; }

private:
  OwnDspMachine machine_{native::native_clocks};
  bool clocked_;
  unsigned cycle_ = 0;
  std::unique_ptr<OwnDspTimeline> timeline_;
};

void native_pin(const Port port, const unsigned index, const bool high, const bool clocked) {
  const auto pins = (1U << port.bits) - 1;
  const auto bit = 1U << index;
  const auto initial = ((pattern & ~bit) | (high ? bit : 0U)) & pins;
  native::NativeProgram program;
  for (const auto value : {port.movep_data,
                           initial,
                           port.movep_data + 1,
                           pins,
                           port.movep_data + 1,
                           0U,
                           port.bset_data | index,
                           port.movep_data + 1,
                           pins,
                           0U}) {
    program.word(value);
  }
  Run run(program, clocked);
  const auto entry = run.machine().cpu().registers();
  for (unsigned instruction = 0; instruction < 3; ++instruction) { run.retire(); }
  auto expected = entry;
  expected.pc += 6;
  const auto &gpio = run.machine().bus().gpio(port.kind);
  require(run.machine().cpu().registers() == expected && gpio.outputs().mask == 0 &&
              gpio.observe_pads(pins).known == 0,
          "Real MOVEP stores and disconnect preserve all CPU registers without inventing pads");
  expected.pc += 1;
  expected.sr = (expected.sr & ~1U) | static_cast<unsigned>(high);
  run.retire();
  require(run.machine().cpu().registers() == expected &&
              gpio.read(Register::data) == (initial | bit) && gpio.outputs().mask == 0 &&
              gpio.observe_pads(pins).known == 0,
          "Native BSET reports the literal old Carry and changes only retained selected storage");

  expected.pc += 2;
  run.retire();
  require(run.machine().cpu().registers() == expected && gpio.outputs().mask == pins &&
              gpio.observe_pads(pins).known == pins &&
              gpio.observe_pads(pins).levels == (initial | bit),
          "Only later native output selection drives the independently expected stored value");
}
}

int main() {
  try {
    constexpr unsigned notification = 9;
    native_pin(ports[1], notification, true, true);
    unsigned pins = 0;
    unsigned native_cases = 1;
    for (const auto port : ports) {
      for (unsigned bit = 0; bit < port.bits; ++bit) {
        for (const bool high : {false, true}) {
          retained_pin(port, bit, high);
          ++pins;
          for (const bool clocked : {false, true}) {
            if (port.kind == DspPort::e && bit == notification && high && clocked) { continue; }
            native_pin(port, bit, high, clocked);
            ++native_cases;
          }
        }
      }
    }
    std::cout << "Functional disconnected GPIO storage guards=" << pins
              << " native-BSET=" << native_cases << " passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
