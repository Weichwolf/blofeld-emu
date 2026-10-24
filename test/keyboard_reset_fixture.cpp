// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_reset_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/hcs08_reset.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_wire_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_reset {
using hcs08::require;
constexpr unsigned initial_hx = 0x3322, initial_sp = 0x4567, initial_a = 0xa5;
constexpr unsigned lower_canary = 0xa5, upper_canary = 0x5a, low_delay = 0x1600,
                   high_delay = 0x1601;
constexpr unsigned memory_mapping = 0x21, phy_enable = 0x40, pll_select = 0x80,
                   previous_pc = 0x1234;
constexpr unsigned own_reset_vector = 0xa5a5, reset_x = 0x22, reset_ccr_known = 0x68,
                   active_status = 0xc8;
constexpr unsigned sample_low = 0x13f1, sample_high = 0x13f9, sample_difference = 0x13fb;
constexpr unsigned host_flags = 0xd0;
constexpr auto released = Hcs08BdcDrive::released;

Fixture::Fixture(const Firmware &image,
                 const std::vector<std::uint8_t> &code,
                 const unsigned first_edge,
                 const unsigned timer_prescale,
                 std::ostream &output,
                 const hcs08_programmer::Memory *memory)
    : firmware(image),
      helper(code),
      host(McuFlashImage::from_known(image.mcu),
           BLOFELD_DEFAULT_SLA,
           McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                    .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                          .memory_timing = McuRunMemory::internal_flash_reads}),
      target(memory),
      next_edge(first_edge),
      trace(output) {
  require(first_edge > 0 && first_edge <= target_period && helper.size() == helper_bytes &&
              (timer_prescale == 2 || timer_prescale == 6),
          "Declared nominal clock/phase/TIM fixture inputs");
  auto &bus = host.bus();
  bus.write(hc12::initee, memory_mapping);
  bus.retire_instruction();
  for (unsigned n = 0; n < helper.size(); ++n) {
    bus.write(static_cast<std::uint16_t>(helper_base + n), helper[n]);
  }
  constexpr std::array<std::uint8_t, 5> own_caller{0x16, 0x13, 0x8c, 0x20, 0xfe};
  for (unsigned n = 0; n < own_caller.size(); ++n) {
    bus.write(static_cast<std::uint16_t>(caller + n), own_caller[n]);
  }
  bus.write(low_delay, lower_canary);
  bus.write(high_delay, upper_canary);
  bus.write(hc12::usb_control, phy_enable);
  bus.observe_phy({.edges = 15});
  bus.write(hc12::clock_select, pll_select);
  constexpr unsigned tscr2 = 0x4d;
  constexpr unsigned timer_enable = 0x90;
  bus.write(tscr2, static_cast<std::uint8_t>(timer_prescale));
  bus.write(hc12::tscr1, timer_enable);
  host.cpu().set_state({.pc = caller, .sp = stack, .ccr = host_flags});
  target.cpu.set_state(
      {.pc = previous_pc, .hx = initial_hx, .sp = initial_sp, .a = initial_a, .ccr = byte_mask});
  bind();
}

Hcs08Bdc &Fixture::controller() {
  if (!bdc) { throw Fault("Missing qualified bdc owner"); }
  return *bdc;
}

const Hcs08Bdc &Fixture::controller() const {
  if (!bdc) { throw Fault("Missing qualified bdc owner"); }
  return *bdc;
}

Hcs08BdcWire &Fixture::transport() {
  if (!wire) { throw Fault("Missing qualified wire owner"); }
  return *wire;
}

const Hcs08BdcWire &Fixture::transport() const {
  if (!wire) { throw Fault("Missing qualified wire owner"); }
  return *wire;
}

Hcs08ResetSequence &Fixture::reset_sequence() {
  if (!reset) { throw Fault("Missing qualified reset owner"); }
  return *reset;
}

const Hcs08ResetSequence &Fixture::reset_sequence() const {
  if (!reset) { throw Fault("Missing qualified reset owner"); }
  return *reset;
}

void Fixture::bind() {
  host.bus().attach_gpio_pad_source(
      McuPort::e, [this](const std::uint32_t mask) { return pads(McuPort::e, mask); });
  host.bus().attach_gpio_pad_source(
      McuPort::b, [this](const std::uint32_t mask) { return pads(McuPort::b, mask); });
  host.bus().set_observer([this](const BusEvent &event) { observe(event); });
  host.set_cycle_observer([this](const Hc12BusCycle &cycle) { advance(cycle); });
}

Hcs08ResetDrive Fixture::reset_drive() const {
  const auto pins = host.bus().gpio_outputs(McuPort::e);
  if ((pins.mask & reset_pin) == 0) { return Hcs08ResetDrive::released; }
  require((pins.known & reset_pin) != 0 && (pins.levels & reset_pin) == 0,
          "RESET supports only known low/released host drive");
  return Hcs08ResetDrive::low;
}

Hcs08BdcDrive Fixture::bkgd_drive() const {
  const auto pins = host.bus().gpio_outputs(McuPort::b);
  if ((pins.mask & bkgd_pin) == 0) { return released; }
  require((pins.known & bkgd_pin) != 0, "BKGD host latch is known");
  return (pins.levels & bkgd_pin) != 0 ? Hcs08BdcDrive::high : Hcs08BdcDrive::low;
}

GpioPadSample Fixture::pads(const McuPort port, const std::uint32_t mask) const {
  const auto host_pins = host.bus().gpio_outputs(port);
  const auto pin = port == McuPort::e ? reset_pin : bkgd_pin;
  const auto selected = mask & pin;
  GpioNetDriver peer{.pull_up = selected};
  // Independent fixture stimuli for other PE/PB inputs read by native RMW.
  // These are not a measured board profile or RESET acknowledgements.
  constexpr unsigned other_e_levels = 3;
  peer.active = mask & ~pin;
  peer.known = peer.active;
  peer.levels = port == McuPort::e ? other_e_levels & peer.active : 0;
  auto drive = released;
  if (port == McuPort::e) {
    if (reset && reset_sequence().drives_low()) { drive = Hcs08BdcDrive::low; }
  } else if (wire) {
    drive = transport().target_drive();
  }
  if (drive != released) {
    peer.active |= selected;
    peer.known |= selected;
    if (drive == Hcs08BdcDrive::high) { peer.levels |= selected; }
    peer.pull_up = 0;
  }
  return resolve_gpio_net(mask,
                          {.active = host_pins.mask & mask,
                           .known = host_pins.known & host_pins.mask & mask,
                           .levels = host_pins.levels & host_pins.mask & mask},
                          peer);
}

void Fixture::observe(const BusEvent &event) {
  if (!event.write || (event.address != hc12::portb && event.address != hc12::ddrb &&
                       event.address != hc12::porte && event.address != hc12::ddre)) {
    return;
  }
  const auto ticks = host.run_clock().elapsed_ticks();
  if (reset_drive() == Hcs08ResetDrive::low && !reset) {
    reset.emplace(Hcs08ResetContract::reference_bus_edges);
    target.memory.flash.reset();
    trace << "ASSERT ticks=" << ticks << " pc=" << context.pc << '\n';
  }
  if (wire && (event.address == hc12::portb || event.address == hc12::ddrb)) {
    transport().host_drive(bkgd_drive(), context, ticks - origin);
  }
  trace << "PIN ticks=" << ticks << " pc=" << context.pc << " address=" << event.address
        << " value=" << static_cast<unsigned>(event.value)
        << " reset=" << static_cast<unsigned>(reset_drive())
        << " bkgd=" << static_cast<unsigned>(bkgd_drive()) << '\n';
}

void Fixture::advance(const Hc12BusCycle cycle) {
  context = cycle.context;
  const auto boundary =
      host.run_clock().elapsed_ticks() + host.run_clock().next_cycle_ticks(host.bus());
  while (next_edge <= boundary) {
    edge(next_edge);
    next_edge += target_period;
  }
}

void Fixture::edge(const std::uint64_t ticks) {
  if (reset && reset_sequence().state().phase != Hcs08ResetPhase::ready) {
    const auto before = reset_sequence().state();
    reset_sequence().edge(reset_drive(),
                          bkgd_drive() == Hcs08BdcDrive::low ? Hcs08ModePad::low
                                                             : Hcs08ModePad::high,
                          context);
    const auto after = reset_sequence().state();
    if (before.phase != after.phase || before.mode != after.mode) {
      trace << "RESET ticks=" << ticks << " edges=" << after.edges
            << " phase=" << static_cast<unsigned>(after.phase)
            << " mode=" << (after.mode ? static_cast<unsigned>(after.mode.value()) : 2U) << '\n';
    }
    if (after.phase == Hcs08ResetPhase::ready) {
      target.cpu.reset();
      require(after.mode && after.mode.value() == Hcs08BdcReset::bkgd_held_low,
              "Actual resolved BKGD-low reset selects active BDC");
      const auto cpu = target.cpu.state();
      require(cpu.pc == own_reset_vector && cpu.hx == reset_x && cpu.sp == byte_mask &&
                  cpu.a == initial_a && cpu.ccr_known == reset_ccr_known &&
                  cpu.ccr == reset_ccr_known,
              "Actual vector/reset effects preserve only documented knowledge");
      bdc.emplace(target.cpu, target.mapping, hcs08_wire::chip, after.mode.value());
      wire.emplace(
          *bdc,
          hcs08_wire::wire_contract,
          bkgd_drive(),
          Hcs08BdcWireClock{.period_ticks = target_period, .first_edge_ticks = target_period});
      origin = ticks;
      trace << "TARGET pc=" << cpu.pc << " hx=" << cpu.hx << " sp=" << cpu.sp
            << " a=" << static_cast<unsigned>(cpu.a) << " ccr=" << static_cast<unsigned>(cpu.ccr)
            << " known=" << static_cast<unsigned>(cpu.ccr_known)
            << " status=" << static_cast<unsigned>(controller().status()) << '\n';
    }
    return;
  }
  if (!wire) { return; }
  const auto before = transport().state();
  transport().clock();
  const auto after = transport().state();
  if (before.sync != after.sync || before.syncs != after.syncs) {
    trace << "SYNC ticks=" << ticks << " cycle=" << after.cycles
          << " phase=" << static_cast<unsigned>(after.sync) << " count=" << after.syncs << '\n';
  }
  if (after.received_bytes != received) {
    require(after.received_bytes == received + 1 &&
                controller().state().frame[0] == blofeld::hcs08_bdc::background,
            "Actual original BACKGROUND is the only decoded command");
    received = after.received_bytes;
    trace << "RX value=" << static_cast<unsigned>(controller().state().frame[0])
          << " ticks=" << ticks << '\n';
  }
}

void Fixture::step() {
  const auto cpu = host.cpu().state();
  trace << "HOST pc=" << cpu.pc << " d=" << cpu.d << " x=" << cpu.x << " y=" << cpu.y
        << " sp=" << cpu.sp << " ccr=" << static_cast<unsigned>(cpu.ccr)
        << " ticks=" << host.run_clock().elapsed_ticks() << '\n';
  if (cpu.pc == sample_low || cpu.pc == sample_high || cpu.pc == sample_difference) {
    trace << "COUNTER pc=" << cpu.pc << " value=" << cpu.d
          << " ticks=" << host.run_clock().elapsed_ticks() << '\n';
  }
  host.cpu().step();
  ++steps;
}

void Fixture::finish(const unsigned timer_prescale) {
  constexpr unsigned limit = step_limit;
  constexpr unsigned wire_tail_bound = 1024;
  while (host.cpu().state().pc != returned) {
    require(steps < limit, "Original reset/SYNC helper has a bounded observed completion");
    step();
  }
  for (unsigned n = 0; wire && (transport().state().perceived || transport().state().service_due ||
                                transport().state().sync != Hcs08BdcSync::idle);
       ++n) {
    require(n < wire_tail_bound && host.cpu().state().pc == returned,
            "Own native idle BRA completes remaining reference wire tail");
    step();
  }
  const bool accepted = timer_prescale == 2;
  const auto reset_state = reset_sequence().state();
  require(reset_state.phase == Hcs08ResetPhase::ready && reset_state.external_at_sample &&
              *reset_state.external_at_sample && bdc && wire && !controller().faulted() &&
              !transport().faulted() && !host.timing_faulted(),
          "Actual external reset completes before original SYNC with healthy native owners");
  require(host.cpu().state().sp == stack &&
              (host.cpu().state().d & byte_mask) == (accepted ? 9U : 0U) &&
              host.bus().peek(low_delay) == (accepted ? 9U : lower_canary) &&
              host.bus().peek(high_delay) == (accepted ? 12U : upper_canary),
          "Actual raw-TCNT acceptance or rejection controls native B and delay stores");
  require(transport().state().syncs == 2 && controller().state().serviced == (accepted ? 1U : 0U) &&
              received == (accepted ? 1U : 0U) && controller().status() == active_status &&
              target.memory.retired == 0,
          "Reset-held BKGD release and explicit request each SYNC; target remains genuinely held "
          "without instruction injection");
  const auto image = host.bus().power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Original native helper preserves all MCU NVM/knowledge/history");
  for (unsigned n = 0; n < helper.size(); ++n) {
    require(host.bus().peek(static_cast<std::uint16_t>(helper_base + n)) == helper[n],
            "Complete original helper remains byte-identical");
  }
  require(!target.memory.flash.state().divider_loaded && !target.memory.flash.state().active &&
              target.memory.transfers == 0,
          "Reset/SYNC cannot invent an NVM clock, command or completion");
  trace << "RETURN accepted=" << accepted << " steps=" << steps
        << " commands=" << controller().state().serviced << " syncs=" << transport().state().syncs
        << " target-retired=" << target.memory.retired << '\n';
}

Saved Fixture::checkpoint() const {
  require(reset && bdc && wire && reset_sequence().state().phase == Hcs08ResetPhase::ready,
          "Joint snapshot follows an actual completed reset");
  return {.host = host.checkpoint(),
          .memory = target.memory,
          .cpu = target.cpu.checkpoint(),
          .reset = reset_sequence(),
          .bdc = controller().checkpoint(),
          .wire = transport().checkpoint(),
          .next_edge = next_edge,
          .origin = origin,
          .steps = steps,
          .received = received};
}

void Fixture::restore(const Saved &saved) {
  target.cpu.restore_checkpoint(saved.cpu);
  reset = saved.reset;
  const auto mode = saved.reset.state().mode;
  if (!mode) { throw Fault("Saved reset mode is absent"); }
  bdc.emplace(target.cpu, target.mapping, hcs08_wire::chip, *mode);
  controller().restore_checkpoint(saved.bdc);
  wire.emplace(controller(),
               hcs08_wire::wire_contract,
               released,
               Hcs08BdcWireClock{.period_ticks = target_period, .first_edge_ticks = target_period});
  transport().restore_checkpoint(saved.wire);
  host.restore(saved.host);
  next_edge = saved.next_edge;
  origin = saved.origin;
  steps = saved.steps;
  received = saved.received;
  bind();
}

std::string Fixture::endpoint() const {
  const auto h = host.cpu().state();
  const auto t = target.cpu.state();
  std::ostringstream out;
  out << h.pc << ' ' << h.d << ' ' << h.x << ' ' << h.y << ' ' << h.sp << ' '
      << static_cast<unsigned>(h.ccr) << ' ' << host.run_clock().elapsed_ticks() << ' '
      << host.run_clock().bus_cycles() << ' ' << t.pc << ' ' << t.hx << ' ' << t.sp << ' '
      << static_cast<unsigned>(t.a) << ' ' << static_cast<unsigned>(t.ccr) << ' '
      << static_cast<unsigned>(t.ccr_known) << ' ' << next_edge << ' ' << origin << ' ' << steps
      << ' ' << received;
  return out.str();
}
}
