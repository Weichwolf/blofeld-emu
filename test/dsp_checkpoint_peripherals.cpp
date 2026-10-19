// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"

namespace blofeld::test::checkpoint {
namespace {
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned htx = 0xffff93;
constexpr unsigned tx_value = 0xa15c37;
constexpr unsigned spi_bits_before_snapshot = 13;
constexpr unsigned timer_compare = 5;
constexpr unsigned compare_flag = 0x200000;
constexpr unsigned pctl = 0x04601d;
constexpr unsigned switched = 0x600085;

unsigned bits(DspMachine &machine, const unsigned first, const unsigned end) {
  unsigned sampled = 0;
  for (unsigned bit = first; bit < end; ++bit) {
    const bool value = ((cookie >> (word_bits - 1 - bit)) & 1U) != 0;
    machine.bus().shi().observe_spi({.selected = true, .clock = false, .mosi = value});
    machine.bus().shi().observe_spi({.selected = true, .clock = true, .mosi = value});
    const auto output = machine.bus().shi().outputs().miso;
    if (!output.driven || !output.level) {
      throw Fault("Explicit SHI transmit data must remain known at capture");
    }
    sampled = (sampled << 1U) | (output.level.value() ? 1U : 0U);
    machine.bus().shi().observe_spi({.selected = true, .clock = false, .mosi = value});
  }
  return sampled;
}
}

void test_checkpoint_peripherals() {
  DspMachine machine;
  auto &bus = machine.bus();
  bus.shi().observe_spi({.mosi = false});
  bus.shi().write({.address = hcsr - 1, .value = 0});
  bus.shi().write({.address = hcsr, .value = 9});
  bus.shi().write({.address = htx, .value = tx_value});
  bus.shi().observe_spi({.selected = true, .mosi = false});
  const auto first = bits(machine, 0, spi_bits_before_snapshot);
  bus.timers().write({.address = csr0 - 1, .value = 0});
  bus.timers().write({.address = csr0 - 2, .value = timer_compare});
  bus.timers().write({.address = csr0, .value = 7});
  bus.timers().advance_clk2(3);
  bus.pll().write(pctl);
  bus.gpio(DspPort::c).write({.reg = DspGpioRegister::direction, .value = 1});
  bus.gpio(DspPort::c).write({.reg = DspGpioRegister::data, .value = 1, .mask = 1});
  bus.gpio(DspPort::c).set_drive({.mask = 2, .levels = 2});
  bus.write({.space = DspSpace::x, .address = 0x10000, .value = cookie});
  bus.observe_omr(switched);
  bus.advance_instruction_cycles(1);
  const auto saved = machine.checkpoint();
  const auto finish = [&machine, first] {
    const auto tail = bits(machine, spi_bits_before_snapshot, word_bits);
    constexpr unsigned tail_bits = word_bits - spi_bits_before_snapshot;
    require(((first << tail_bits) | tail) == tx_value,
            "Snapshot must preserve SHI transmit bit position and partial word");
    require(machine.bus().shi().read(hrx) == cookie,
            "Remaining pin events must finish exactly the original incoming 24-bit word");
    machine.bus().timers().advance_clk2(3);
    require(machine.bus().timers().read(csr0, compare_flag) == compare_flag &&
                machine.bus().timers().read(csr0 - 3) == timer_compare,
            "Timer count and first-clock phase must resume at the independent compare edge");
    machine.bus().advance_instruction_cycles(2);
    require(machine.bus().read({.address = 0x1000}) == cookie,
            "Remaining explicit map-switch cycles must retain physical X/P alias contents");
    const auto pads = machine.bus().gpio(DspPort::c).observe_pads(3);
    require(pads.known == 3 && pads.levels == 3 && machine.bus().pll().read() == pctl,
            "Restored GPIO latch knowledge, external drive and PLL command must remain distinct");
  };
  finish();
  machine.bus().reset();
  machine.bus().write({.space = DspSpace::x, .address = 0x10000, .value = other_cookie});
  machine.restore(saved);
  static_cast<void>(
      rejects([&machine] { static_cast<void>(machine.bus().read({.address = 0x1000})); },
              "A checkpoint must not complete the remaining map-switch delay early"));
  finish();
  machine.restore(saved);
  static_cast<void>(rejects(
      [&machine] { static_cast<void>(machine.bus().gpio(DspPort::c).read(DspGpioRegister::data)); },
      "Whole-word GPIO reads must retain unknown neighboring inputs after restore"));
  finish();
}
}
