// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/gpio_pads.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_clock.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_reset_sync {
using namespace blofeld;
namespace hw = blofeld::test::hcs08_wire;
using blofeld::test::hcs08::require;
constexpr unsigned oscillator_hz = 12000000, helper_base = 0x1200, helper_bytes = 1018;
constexpr unsigned caller = 0x2000, returned = 0x2003, stack = 0x2200, entry = 0x138c;
constexpr unsigned pin = 2, reset_pin = 0x80, tscr2 = 0x4d, tcnt = 0x44;
constexpr unsigned low_stamp = 0x13ef, high_stamp = 0x13f7, difference_pc = 0x13f9;
constexpr unsigned limit = 2000000;
constexpr unsigned scale = 2;
constexpr unsigned timer_period_ticks = 16; // 120 MHz reference, 30 MHz bus, TIM bus/4.
constexpr unsigned reset_cycles = 5;
constexpr unsigned timer_start_ticks = 100; // Five reset cycles at the initial OSC/2 bus rate.
constexpr unsigned memory_map = 0x21, usb_high = 0x300, phy_start = 0x40;
constexpr unsigned pll_select = 0x80, timer_enable = 0x90, initial_ccr = 0xd0;
constexpr unsigned byte_mask = 255, word_mask = 65535, other_e_low = 0x7c;

struct Reference {
  unsigned bdc_hz = 0;
  unsigned phase_ticks = 0;
  bool reset_pullup = false;
  bool connected = false;
};

inline std::uint64_t reference_period(const McuRunClock &clock, const Reference ref) {
  constexpr unsigned slow_hz = 3000000;
  constexpr unsigned nominal_hz = 4000000;
  constexpr unsigned fast_hz = 6000000;
  require(ref.bdc_hz == slow_hz || ref.bdc_hz == nominal_hz || ref.bdc_hz == fast_hz,
          "Explicit reference target clock profile");
  const auto hz = clock.timebase_hz() * scale;
  require(hz % ref.bdc_hz == 0, "Exact reference common clock");
  const auto ticks = hz / ref.bdc_hz;
  require(ref.phase_ticks > 0 && ref.phase_ticks <= ticks && ref.phase_ticks % 2 == 1,
          "Explicit reference phase avoids coincident host edges");
  return ticks;
}

// Reference RESET/BKGD circuit only. Native target reset/ICG startup, board
// continuity and prior target power-on history remain unqualified. CPU entry
// registers and memory are declared own inputs, never hardware reset claims.
struct Fixture {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  McuMachine host;
  Reference reference;
  std::unique_ptr<hw::Target> target;
  std::unique_ptr<Hcs08Bdc> bdc;
  std::unique_ptr<Hcs08BdcWire> wire;
  ExecutionContext context;
  std::optional<std::uint64_t> asserted, released;
  std::optional<std::uint64_t> timer_read;
  std::uint64_t period, steps = 0;
  unsigned low_read = 0, high_read = 0, difference = 0;
  std::ostream &trace;

  Fixture(const Firmware &image,
          const std::vector<std::uint8_t> &code,
          const Reference ref,
          std::ostream &output)
      : firmware(image),
        helper(code),
        host(McuFlashImage::from_known(image.mcu),
             BLOFELD_DEFAULT_SLA,
             McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                      .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                            .memory_timing = McuRunMemory::internal_flash_reads}),
        reference(ref),
        period(reference_period(host.run_clock(), ref)),
        trace(output) {
    auto &bus = host.bus();
    bus.write(hc12::initee, memory_map);
    bus.retire_instruction();
    for (unsigned n = 0; n < helper.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(helper_base + n), helper[n]);
    }
    constexpr std::array<std::uint8_t, 5> own{0x16,
                                              static_cast<std::uint8_t>(entry >> 8U),
                                              static_cast<std::uint8_t>(entry & byte_mask),
                                              0x20,
                                              0xfe};
    for (unsigned n = 0; n < own.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(caller + n), own[n]);
    }
    bus.write(hc12::clock_select, 0);
    bus.write(usb_high, phy_start);
    bus.observe_phy({.edges = 15});
    bus.write(hc12::clock_select, pll_select);
    bus.write(tscr2, 2);
    bus.write(hc12::tscr1, timer_enable);
    const auto route = bus.run_clock_route();
    require(route.source == McuClockSource::usb_phy && route.bus_divider == 2 &&
                bus.read(hc12::refdv) == 0 && bus.read(hc12::clock_select) == pll_select &&
                bus.read(hc12::tscr1) == timer_enable && bus.read(tscr2) == 2,
            "Declared source-derived entry clock/timer state, not inferred from the entry PC");
    require(host.run_clock().bus_cycles() == reset_cycles && time() == timer_start_ticks &&
                bus.read(tcnt) == 0 && bus.read(tcnt + 1) == 0,
            "Five actual reset cycles precede the declared TIM enable epoch at count zero");
    trace << "CLOCK common=" << host.run_clock().timebase_hz() * scale
          << " bus=" << McuRunClock::phy_hz / route.bus_divider << " timer-divisor=4"
          << " timer-start=" << timer_start_ticks << " reset-cycles=" << reset_cycles << '\n';
    bus.write(hc12::ddrb, byte_mask);
    bus.write(hc12::portb, pin);
    bus.write(hc12::ddre, reset_pin);
    bus.write(hc12::porte, reset_pin);
    bus.attach_gpio_pad_source(McuPort::e, [this](const auto mask) { return reset_net(mask); });
    bus.attach_gpio_pad_source(McuPort::b, [this](const auto mask) { return data_net(mask); });
    bus.set_observer([this](const BusEvent &event) { observe(event); });
    host.set_cycle_observer([this](const Hc12BusCycle &cycle) { advance(cycle); });
    host.cpu().set_state({.pc = caller, .sp = stack, .ccr = initial_ccr});
  }

  [[nodiscard]] std::uint64_t time() const { return host.run_clock().elapsed_ticks() * scale; }

  [[nodiscard]] GpioPadSample reset_net(const std::uint32_t mask) const {
    const auto own = host.bus().gpio_outputs(McuPort::e);
    return resolve_gpio_net(mask,
                            {.active = own.mask & mask,
                             .known = own.known & own.mask & mask,
                             .levels = own.levels & own.mask & mask},
                            {.pull_up = (3U | (reference.reset_pullup ? reset_pin : 0U)) & mask,
                             .pull_down = other_e_low & mask});
  }

  [[nodiscard]] GpioPadSample data_net(const std::uint32_t mask) const {
    const auto own = host.bus().gpio_outputs(McuPort::b);
    const auto drive = wire ? wire->target_drive() : Hcs08BdcDrive::released;
    const auto selected = mask & pin;
    GpioNetDriver peer{.pull_up = selected};
    if (drive != Hcs08BdcDrive::released) {
      peer.active = selected;
      peer.known = selected;
      peer.levels = drive == Hcs08BdcDrive::high ? selected : 0;
      peer.pull_up = 0;
    }
    return resolve_gpio_net(mask,
                            {.active = own.mask & mask,
                             .known = own.known & own.mask & mask,
                             .levels = own.levels & own.mask & mask},
                            peer);
  }

  void reset_event() {
    const auto sample = reset_net(reset_pin);
    if (sample.known != reset_pin) { return; }
    if ((sample.levels & reset_pin) == 0 && !asserted) {
      asserted = time();
      if (reference.connected) { target = std::make_unique<hw::Target>(); }
      trace << "ASSERT time=" << time() << " pc=" << context.pc << '\n';
    }
    if ((sample.levels & reset_pin) != 0 && asserted && !released) {
      released = time();
      const auto mode = data_net(pin);
      require(mode.known == pin && (mode.levels & pin) == 0,
              "Actual BKGD host low selects the declared active-reset mode");
      if (reference.connected) {
        if (!target) { throw Fault("Target lifetime requires an observed reset assertion"); }
        bdc = std::make_unique<Hcs08Bdc>(
            target->cpu, target->mapping, hw::chip, Hcs08BdcReset::bkgd_held_low);
        wire = std::make_unique<Hcs08BdcWire>(
            *bdc,
            hw::wire_contract,
            hw::low,
            Hcs08BdcWireClock{.period_ticks = period, .first_edge_ticks = reference.phase_ticks});
      }
      trace << "RELEASE time=" << time() << " pc=" << context.pc << " period=" << period << '\n';
    }
  }

  void observe(const BusEvent &event) {
    if (!event.write) { return; }
    if (event.address == hc12::porte || event.address == hc12::ddre) { reset_event(); }
    if (event.address != hc12::portb && event.address != hc12::ddrb) { return; }
    const auto own = host.bus().gpio_outputs(McuPort::b);
    auto drive = Hcs08BdcDrive::released;
    if ((own.mask & pin) != 0) {
      require((own.known & pin) != 0, "Host BKGD drive must be known");
      drive = (own.levels & pin) != 0 ? hw::high : hw::low;
    }
    if (wire) {
      if (!released) { throw Fault("Wire owner lost its reset-release epoch"); }
      wire->host_drive(drive, context, time() - released.value());
    }
    trace << "HOST time=" << time() << " pc=" << context.pc << " address=" << event.address
          << " value=" << static_cast<unsigned>(event.value)
          << " drive=" << static_cast<unsigned>(drive) << '\n';
  }

  void advance(const Hc12BusCycle &cycle) {
    context = cycle.context;
    const auto boundary = time() + (host.run_clock().next_cycle_ticks(host.bus()) * scale);
    if (wire) {
      if (!released) { throw Fault("Wire clock requires a reset-release epoch"); }
      const auto epoch = released.value();
      while (wire->state().next_clock_ticks + epoch <= boundary) {
        const auto before = wire->target_drive();
        const auto syncs = wire->state().syncs;
        wire->clock();
        if (before != wire->target_drive() || syncs != wire->state().syncs) {
          trace << "TARGET time=" << wire->state().time_ticks + epoch
                << " drive=" << static_cast<unsigned>(wire->target_drive())
                << " syncs=" << wire->state().syncs << '\n';
        }
      }
    }
    if (cycle.address == tcnt && cycle.size == 2) {
      require(cycle.kind == 'R' && cycle.word_part == Hc12WordPart::whole && !timer_read &&
                  (context.pc == low_stamp || context.pc == high_stamp),
              "Only actual original aligned TCNT word reads supply timestamp observations");
      timer_read = boundary;
      trace << "TIMER time=" << boundary << " pc=" << context.pc << " address=" << tcnt
            << " size=" << cycle.size << '\n';
    }
  }

  void run() {
    while (host.cpu().state().pc != returned) {
      require(steps < limit, "Native original reset/SYNC reference bound");
      const auto before = host.cpu().state().pc;
      host.cpu().step();
      ++steps;
      if (before == low_stamp || before == high_stamp || before == difference_pc) {
        const auto value = host.cpu().state().d;
        if (before != difference_pc) {
          require(timer_read &&
                      value == (((timer_read.value() - timer_start_ticks) / timer_period_ticks) &
                                word_mask),
                  "Actual native LDD value follows continuing bus/4 TCNT at its operand cycle");
          timer_read.reset();
        }
        if (before == low_stamp) { low_read = value; }
        if (before == high_stamp) { high_read = value; }
        if (before == difference_pc) { difference = value; }
        trace << "STAMP time=" << time() << " pc=" << before << " value=" << value << '\n';
      }
    }
    unsigned tail = 0;
    while (wire && (wire->state().perceived || wire->state().service_due)) {
      require(++tail < 128, "Own idle caller settles the last real BDC bit/service boundary");
      host.cpu().step();
      ++steps;
    }
    trace << "END rate=" << reference.bdc_hz << " steps=" << steps << " time=" << time()
          << " result=" << (host.cpu().state().d & byte_mask) << " low=" << low_read
          << " high=" << high_read << " difference=" << difference
          << " syncs=" << (wire ? wire->state().syncs : 0)
          << " commands=" << (bdc ? bdc->state().serviced : 0) << '\n';
  }
};
} // namespace blofeld::test::keyboard_reset_sync
