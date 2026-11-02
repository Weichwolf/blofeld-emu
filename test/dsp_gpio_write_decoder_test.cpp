// SPDX-License-Identifier: GPL-3.0-only
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
using Register = DspGpioRegister;
constexpr auto functional = DspGpioContract::logical_pins;
constexpr unsigned word_mask = 0xffffff, pattern = 0xa5a561;
constexpr unsigned bcr = 0xfffffb, f_bcr = 0x01ffff, f_direction = 0xfffff6;

struct Port {
  DspPort kind;
  unsigned bits, opcode;
};

// UM register geometry and FM13-135 immediate MOVEP templates; authored code.
constexpr std::array ports{Port{.kind = DspPort::c, .bits = 12, .opcode = 0x07f43d},
                           Port{.kind = DspPort::e, .bits = 12, .opcode = 0x07b49d},
                           Port{.kind = DspPort::f, .bits = 11, .opcode = 0x09f4b5},
                           Port{.kind = DspPort::d, .bits = 2, .opcode = 0x08f495}};

void every_bit() {
  constexpr unsigned word_bits = 24;
  for (const auto port : ports) {
    const auto pins = (1U << port.bits) - 1;
    for (const auto reg : {Register::data, Register::direction, Register::control}) {
      DspGpio gpio(port.kind);
      gpio.set_contract(functional);
      if (reg == Register::data) { gpio.write({.reg = Register::direction, .value = pins}); }
      for (unsigned bit = 0; bit < word_bits; ++bit) {
        const auto value = 1U << bit;
        gpio.write({.reg = reg, .value = value});
        require(gpio.read(reg) == (value & pins) &&
                    ((gpio.outputs().mask | gpio.outputs().peripheral) & ~pins) == 0,
                "Every register bit selects exactly its independently declared pin storage");
      }
      gpio.write({.reg = reg, .value = pattern});
      constexpr unsigned upper = 0x800000;
      gpio.write({.reg = reg, .value = upper, .mask = upper});
      require(gpio.read(reg) == (pattern & pins),
              "A reserved-only masked store cannot clear neighboring existing pins");
      gpio.write({.reg = reg, .value = upper, .mask = upper | 1U});
      require(gpio.read(reg) == (pattern & pins & ~1U),
              "A mixed mask changes only the selected existing pin");
    }
  }
}

void reset_and_guards() {
  constexpr unsigned invalid_word = 0x1000000;
  for (const auto port : ports) {
    const auto pins = (1U << port.bits) - 1;
    DspGpio gpio(port.kind);
    require(gpio.contract() == DspGpioContract::strict &&
                rejects([&] {
                  gpio.write({.reg = Register::direction, .value = word_mask});
                }).contains("reserved-bit write"),
            "Strict diagnostics remain explicit and reject nonzero upper bits");
    gpio.set_contract(functional);
    const auto invalid = std::bit_cast<DspGpioContract>(std::uint8_t{2});
    static_cast<void>(rejects([&] { gpio.set_contract(invalid); }));
    require(gpio.contract() == functional, "Invalid contracts do not replace the decoder");
    require(rejects([&] { gpio.write({.value = invalid_word}); }).contains("masked write") &&
                rejects([&] { gpio.write({.mask = invalid_word}); }).contains("masked write") &&
                rejects([&] { gpio.write({.value = 2, .mask = 1}); }).contains("masked write"),
            "Functional pin decoding cannot hide invalid host word/mask combinations");
    gpio.write({.reg = Register::direction, .value = word_mask});
    gpio.write({.value = pattern});
    require(rejects([&] {
              gpio.set_drive({.mask = pins, .levels = (pattern ^ pins) & pins});
            }).contains("contention") &&
                gpio.read(Register::data) == (pattern & pins),
            "Actual conflicting pin drivers reject without replacing the known output");
    const auto copy = gpio;
    gpio.reset();
    require(gpio.contract() == functional && gpio.read(Register::direction) == 0 &&
                gpio.read(Register::control) == 0,
            "Reset retains functional geometry while clearing actual pin configuration");
    gpio.write({.reg = Register::direction, .value = word_mask});
    require(gpio.read(Register::data) == (port.kind == DspPort::d ? 0U : pattern & pins) &&
                copy.contract() == functional && copy.read(Register::data) == (pattern & pins),
            "Port-specific reset latches and independent copied storage are preserved");
    gpio.set_contract(DspGpioContract::strict);
    require(rejects([&] { gpio.write({.value = word_mask}); }).contains("reserved-bit write"),
            "Switching back restores strict diagnostics");
  }
  DspGpio input(DspPort::c);
  input.set_contract(functional);
  input.write({.reg = Register::control, .value = word_mask});
  require(rejects([&] { static_cast<void>(input.read(Register::data)); }).contains("Unknown"),
          "Pin masking never fabricates external input data");
  DspGpio dedicated(DspPort::f);
  dedicated.write({.reg = Register::control, .value = 1});
  require(rejects([&] {
            dedicated.write({.reg = Register::direction, .value = 1});
          }).contains("Reserved DSP Port F pin function") &&
              dedicated.read(Register::direction) == 0,
          "Strict reserved Port-F functions still reject atomically");
}

void transient_mux() {
  constexpr unsigned pins = 0x7ff;
  DspGpio gpio(DspPort::f);
  gpio.set_contract(functional);
  gpio.write({.reg = Register::direction, .value = pins});
  gpio.write({.value = pattern & pins});
  gpio.write({.reg = Register::control, .value = pins});
  gpio.set_drive({.mask = pins, .levels = pins});
  require(gpio.read(Register::control) == pins && gpio.read(Register::direction) == pins &&
              gpio.outputs().mask == 0 && gpio.outputs().peripheral == pins &&
              gpio.observe_pads(pins).known == 0,
          "A transient reserved mux retains register state without inventing an active function");
  require(rejects([&] { static_cast<void>(gpio.read(Register::data)); }).contains("Unknown") &&
              rejects([&] {
                gpio.validate_peripheral_drive({.mask = 1, .levels = 1});
              }).contains("Unmodeled DSP Port F"),
          "Reserved mux data and internal drivers remain visibly unsupported");
  gpio.write({.value = pins});
  gpio.write({.reg = Register::direction});
  require(gpio.read(Register::data) == pins,
          "The next defined input mode reads actual external data");
  gpio.set_drive({});
  require(rejects([&] { static_cast<void>(gpio.read(Register::data)); }).contains("Unknown"),
          "Withdrawing the input cannot turn the transient mode into known data");
  gpio.write({.reg = Register::direction, .value = 1});
  gpio.write({.reg = Register::control});
  require(gpio.outputs().mask == 1 && gpio.outputs().known == 1 &&
              gpio.read(Register::data, 1) == 1,
          "Functional latch storage retains a preload without driving the reserved mux");
  gpio.write({.value = 1});
  require(gpio.read(Register::data, 1) == 1,
          "An explicit output write restores normal defined GPIO behavior");
}

void logical_bus() {
  DspBus bus;
  require(rejects([&] {
            bus.write({.space = DspSpace::y, .address = f_direction, .value = word_mask});
          }).contains("BCR"),
          "Strict Port-F diagnostics retain the explicit BIU prerequisite");
  bus.gpio(DspPort::f).set_contract(functional);
  constexpr unsigned f_data = f_direction - 1;
  constexpr unsigned f_function = f_direction + 1;
  constexpr unsigned pins = 0x7ff;
  constexpr unsigned bcr_reset = 0x1fffff;
  bus.write({.space = DspSpace::y, .address = f_direction, .value = word_mask});
  bus.write({.space = DspSpace::y, .address = f_data, .value = pattern});
  require(bus.read({.space = DspSpace::x, .address = bcr}) == bcr_reset &&
              bus.read({.space = DspSpace::y, .address = f_direction}) == pins &&
              bus.read({.space = DspSpace::y, .address = f_data}) == (pattern & pins),
          "Logical pin access does not invent a BCR write or simulated BIU completion");
  const auto old = bus.access_peripheral_bit(
      {.space = DspSpace::y, .address = f_data, .operation = DspBitOperation::clear});
  const auto prior = bus.access_peripheral_bit(
      {.space = DspSpace::y, .address = f_data, .bit = 1, .operation = DspBitOperation::set});
  require(old && *old && prior && !*prior &&
              bus.read({.space = DspSpace::y, .address = f_data}) == ((pattern & pins & ~1U) | 2U),
          "Direct pin RMW preserves actual old bits and neighboring pins");
  bus.reset();
  require(bus.gpio(DspPort::f).contract() == functional &&
              bus.read({.space = DspSpace::y, .address = f_direction}) == 0,
          "Reset retains the logical bus contract without configuring its real pins");
  bus.write({.space = DspSpace::y, .address = f_function, .value = pins});
  require(rejects([&] {
            static_cast<void>(bus.read({.space = DspSpace::y, .address = f_data}));
          }).contains("Unknown DSP GPIO data read"),
          "Logical Port-F access never supplies missing NAND/input levels");
  bus.write({.space = DspSpace::x, .address = bcr, .value = f_bcr});
  require(rejects([&] {
            static_cast<void>(bus.read({.space = DspSpace::x, .address = bcr}));
          }).contains("BIU bus-master state"),
          "Actual BCR commands still cannot invent bus-master status");
}

enum class PreloadMux : std::uint8_t { disconnected, input, peripheral };

void preloaded_pin(const Port port, const unsigned index, const PreloadMux mode, const bool high) {
  const auto bit = 1U << index;
  DspGpio gpio(port.kind);
  gpio.set_contract(functional);
  if (mode != PreloadMux::disconnected) { gpio.write({.reg = Register::control, .value = bit}); }
  if (mode == PreloadMux::peripheral) { gpio.write({.reg = Register::direction, .value = bit}); }
  gpio.write({.value = high ? bit : 0U, .mask = bit});
  require(gpio.outputs().mask == 0 && gpio.observe_pads(bit).known == 0,
          "A preload does not drive an input, disconnected pin or peripheral");
  if (mode == PreloadMux::input) {
    gpio.set_drive({.mask = bit, .levels = high ? 0U : bit});
    require(gpio.read(Register::data, bit) == (high ? 0U : bit),
            "An input reads its actual pad, independently of a pending output preload");
    gpio.set_drive({});
  }
  const auto saved = gpio;
  gpio.write({.reg = Register::control});
  gpio.write({.reg = Register::direction, .value = bit});
  require(gpio.outputs().mask == bit && gpio.outputs().known == bit &&
              gpio.read(Register::data, bit) == (high ? bit : 0U),
          "Selecting an output publishes exactly its explicitly preloaded bit");
  require(rejects([&] {
            gpio.set_drive({.mask = bit, .levels = high ? 0U : bit});
          }).contains("contention"),
          "A preload never permits opposed strong output drivers");
  auto copy = saved;
  copy.write({.reg = Register::control});
  copy.write({.reg = Register::direction, .value = bit});
  require(copy.read(Register::data, bit) == gpio.read(Register::data, bit),
          "Independent copies preserve a preload before output selection");
}

void preloaded_pins() {
  for (const auto port : ports) {
    for (unsigned index = 0; index < port.bits; ++index) {
      for (const auto mode :
           {PreloadMux::disconnected, PreloadMux::input, PreloadMux::peripheral}) {
        for (const bool high : {false, true}) { preloaded_pin(port, index, mode, high); }
      }
    }
  }
}

void native_bit(const unsigned bit, const bool preload) {
  constexpr unsigned f_direction_opcode = 0x09f4b6;
  constexpr unsigned f_data_opcode = 0x09f4b5;
  constexpr unsigned f_set_opcode = 0x0ab560;
  constexpr unsigned pins = 0x7ff;
  native::NativeProgram program;
  if (preload) {
    for (const auto word : {f_data_opcode, pattern, f_direction_opcode, word_mask}) {
      program.word(word);
    }
  } else {
    for (const auto word : {f_direction_opcode, word_mask, f_data_opcode, pattern}) {
      program.word(word);
    }
  }
  program.word(f_set_opcode | bit);
  program.word(0);
  OwnDspMachine machine(native::native_clocks);
  machine.bus().gpio(DspPort::f).set_contract(functional);
  native::reset_native(machine, program);
  native::step_native(machine);
  native::step_native(machine);
  auto expected = machine.cpu().registers();
  ++expected.pc;
  const unsigned old = (pattern & pins) >> bit & 1U;
  expected.sr = (expected.sr & ~1U) | old;
  native::step_native(machine);
  require(machine.cpu().registers() == expected &&
              machine.bus().gpio(DspPort::f).read(Register::data) ==
                  ((pattern | ((1U << bit) & pins)) & pins),
          "Native BSET reads real Carry and changes only an existing selected pin");
}

void execute(const Port port, const bool clocked, const bool preload) {
  const auto pins = (1U << port.bits) - 1;
  native::NativeProgram program;
  if (preload) {
    program.word(port.opcode);
    program.word(pattern);
    program.word(port.opcode + 1);
    program.word(word_mask);
  } else {
    program.word(port.opcode + 1);
    program.word(word_mask);
    program.word(port.opcode);
    program.word(pattern);
  }
  OwnDspMachine machine(native::native_clocks);
  machine.bus().gpio(port.kind).set_contract(functional);
  native::reset_native(machine, program);
  const auto checkpoint = machine.checkpoint();
  auto expected = machine.cpu().registers();
  expected.pc += 4;
  const auto retired = machine.cpu().progress().instructions;
  if (clocked) {
    OwnDspTimeline timeline(
        machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
    for (unsigned cycle = 0; cycle < 64 && machine.cpu().progress().instructions < retired + 2;
         ++cycle) {
      static_cast<void>(timeline.advance_one(timeline.next_deadline(),
                                             OwnDspBoundary::through,
                                             {.cycle = cycle,
                                              .timing = DspDmaTiming::reference_sampled_cycles,
                                              .grant = DspDmaGrant::available}));
    }
    require(!timeline.faulted(), "Real native GPIO instructions complete through the clock owner");
  } else {
    native::step_native(machine);
    native::step_native(machine);
  }
  require(machine.cpu().registers() == expected &&
              machine.cpu().progress().instructions == retired + 2 &&
              machine.bus().gpio(port.kind).read(Register::direction) == pins &&
              machine.bus().gpio(port.kind).read(Register::data) == (pattern & pins),
          "Original native MOVEP semantics change PC and existing pins, preserving all CPU flags");
  OwnDspMachine fresh(native::native_clocks);
  fresh.restore(checkpoint);
  native::step_native(fresh);
  native::step_native(fresh);
  require(fresh.cpu().registers() == expected &&
              fresh.bus().gpio(port.kind).contract() == functional &&
              fresh.bus().gpio(port.kind).read(Register::data) == (pattern & pins),
          "Fresh native restore retains geometry and executes the real continuation");
}
}

int main() {
  try {
    every_bit();
    reset_and_guards();
    transient_mux();
    preloaded_pins();
    logical_bus();
    for (const unsigned bit : {0U, 1U, 23U}) {
      for (const bool preload : {false, true}) { native_bit(bit, preload); }
    }
    for (const auto port : ports) {
      for (const bool preload : {false, true}) {
        execute(port, false, preload);
        execute(port, true, preload);
      }
    }
    std::cout << "Functional GPIO pin decoding and native execution passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
