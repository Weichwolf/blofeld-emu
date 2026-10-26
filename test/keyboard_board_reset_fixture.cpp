// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard_board_reset_fixture.hpp"

#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "board/hcs08_board_wire.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "hcs08_board_wire_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_reset.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_board_reset {
using hcs08::require;
constexpr unsigned lower_canary = 0xa5, upper_canary = 0x5a, low_delay = 0x1600,
                   high_delay = 0x1601;
constexpr unsigned memory_mapping = 0x21, phy_enable = 0x40, pll_select = 0x80;
constexpr unsigned reset_ccr_known = 0x68, active_status = 0xc8;
constexpr unsigned sample_low = 0x13f1, sample_high = 0x13f9, sample_difference = 0x13fb;
constexpr unsigned host_flags = 0xd0;
constexpr auto released = Hcs08BdcDrive::released;

Fixture::Fixture(const Firmware &image,
                 const std::vector<std::uint8_t> &code,
                 const unsigned first_edge,
                 const unsigned timer_prescale,
                 std::ostream &output)
    : firmware(image),
      helper(code),
      owned_host(std::make_unique<McuMachine>(
          McuFlashImage::from_known(image.mcu),
          BLOFELD_DEFAULT_SLA,
          McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                   .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                         .memory_timing = McuRunMemory::internal_flash_reads})),
      host(*owned_host),
      pre_reset(Hcs08Board::power_on_checkpoint(
                    board_wire::blank(), board_wire::contract, Hcs08ResetEntry::active_background),
                BLOFELD_HCS08_SLA,
                {}),
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
  bind();
}

Fixture::Fixture(McuMachine &machine,
                 const Firmware &image,
                 const std::vector<std::uint8_t> &code,
                 const unsigned first_edge,
                 std::ostream &output)
    : firmware(image),
      helper(code),
      host(machine),
      pre_reset(Hcs08Board::power_on_checkpoint(
                    board_wire::blank(), board_wire::contract, Hcs08ResetEntry::active_background),
                BLOFELD_HCS08_SLA,
                {}),
      next_edge(first_edge),
      trace(output) {
  require(first_edge > 0 && first_edge <= target_period && code.size() == helper_bytes,
          "Declared borrowed controller clock/phase/reference extent");
  const auto now = host.run_clock().elapsed_ticks();
  if (now >= next_edge) { next_edge += ((now - next_edge) / target_period + 1) * target_period; }
}

std::bitset<Hcs08BoardMemory::ram_bytes> Fixture::ram_known() const {
  std::bitset<Hcs08BoardMemory::ram_bytes> result;
  for (unsigned n = 0; n < Hcs08BoardMemory::ram_bytes; ++n) {
    result[n] = target_owner()
                    .memory()
                    .inspect_ram(static_cast<std::uint16_t>(Hcs08BoardMemory::ram_first + n))
                    .known != 0;
  }
  return result;
}

Hcs08BoardWire &Fixture::target_owner() {
  if (!target) { throw Fault("Missing production target wire owner"); }
  return *target;
}

const Hcs08BoardWire &Fixture::target_owner() const {
  if (!target) { throw Fault("Missing production target wire owner"); }
  return *target;
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
  peer.active = mask & ~pin & ~static_cast<std::uint32_t>(host_pins.mask);
  peer.known = peer.active;
  peer.levels = port == McuPort::e ? other_e_levels & peer.active : 0;
  auto drive = released;
  if (port == McuPort::e) {
    if (reset && reset_sequence().drives_low()) { drive = Hcs08BdcDrive::low; }
  } else if (target) {
    drive = target_owner().target_drive();
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
    trace << "ASSERT ticks=" << ticks << " pc=" << context.pc << '\n';
  }
  if (target && (event.address == hc12::portb || event.address == hc12::ddrb)) {
    target_owner().host_drive(bkgd_drive(), context, ticks - origin);
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
      require(after.mode && after.mode.value() == Hcs08BdcReset::bkgd_held_low,
              "Actual resolved BKGD-low reset selects active BDC");
      target.emplace(pre_reset.reset_checkpoint({.cause = Hcs08ResetCause::warm,
                                                 .entry = Hcs08ResetEntry::active_background}),
                     BLOFELD_HCS08_SLA,
                     Hcs08BoardInputs{},
                     Hcs08BoardWireInputs{.contract = Hcs08BdcWireContract::reference_command_delay,
                                          .initial_host = bkgd_drive(),
                                          .clock = {.period_ticks = target_period,
                                                    .first_edge_ticks = target_period}});
      const auto cpu = target_owner().cpu_state();
      constexpr unsigned high_known = 0xff00;
      require(!cpu.pc_known && cpu.pc == 0 && cpu.hx == 0 && cpu.hx_known == high_known &&
                  cpu.sp == byte_mask && cpu.a == 0 && cpu.a_known == 0 &&
                  cpu.ccr_known == reset_ccr_known && cpu.ccr == reset_ccr_known,
              "Actual production warm entry retains unknown A/X and reads no reset vector");
      origin = ticks;
      trace << "TARGET pc=" << cpu.pc << " pc-known=" << cpu.pc_known << " hx=" << cpu.hx
            << " sp=" << cpu.sp << " a=" << static_cast<unsigned>(cpu.a)
            << " ccr=" << static_cast<unsigned>(cpu.ccr)
            << " a-known=" << static_cast<unsigned>(cpu.a_known) << " hx-known=" << cpu.hx_known
            << " known=" << static_cast<unsigned>(cpu.ccr_known)
            << " status=" << static_cast<unsigned>(target_owner().bdc_state().control | 0x40U)
            << '\n';
    }
    return;
  }
  if (!target) { return; }
  require(
      target_owner().bdc_state().active && target_owner().run_state().offered_periods == 0 &&
          target_owner().memory().periods().attempted == 0,
      "Only explicit BDC cadence is supplied;active-background BUSCLK gating remains unqualified");
  const auto before = target_owner().wire_state();
  target_owner().clock_bdc();
  const auto after = target_owner().wire_state();
  if (before.sync != after.sync || before.syncs != after.syncs) {
    trace << "SYNC ticks=" << ticks << " cycle=" << after.cycles
          << " phase=" << static_cast<unsigned>(after.sync) << " count=" << after.syncs << '\n';
  }
  if (after.received_bytes != received) {
    const auto control = target_owner().bdc_state();
    require(after.received_bytes == received + 1 && control.received > 0,
            "Every actual BKGD byte retains its serial frame position");
    const auto byte = control.frame.at(control.received - 1);
    require(byte == blofeld::hcs08_bdc::background,
            "Actual original BACKGROUND is the only decoded command");
    if (received_byte) { received_byte(byte); }
    received = after.received_bytes;
    trace << "RX value=" << static_cast<unsigned>(byte) << " ticks=" << ticks << '\n';
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
  for (unsigned n = 0; target && (target_owner().wire_state().perceived ||
                                  target_owner().wire_state().service_due ||
                                  target_owner().wire_state().sync != Hcs08BdcSync::idle);
       ++n) {
    require(n < wire_tail_bound && host.cpu().state().pc == returned,
            "Own native idle BRA completes remaining reference wire tail");
    step();
  }
  const bool accepted = timer_prescale == 2;
  const auto reset_state = reset_sequence().state();
  require(reset_state.phase == Hcs08ResetPhase::ready && reset_state.external_at_sample &&
              *reset_state.external_at_sample && target && !target_owner().faulted() &&
              !host.timing_faulted(),
          "Actual external reset completes before original SYNC with healthy native owners");
  require(host.cpu().state().sp == stack &&
              (host.cpu().state().d & byte_mask) == (accepted ? 9U : 0U) &&
              host.bus().peek(low_delay) == (accepted ? 9U : lower_canary) &&
              host.bus().peek(high_delay) == (accepted ? 12U : upper_canary),
          "Actual raw-TCNT acceptance or rejection controls native B and delay stores");
  require(target_owner().wire_state().syncs == 2 &&
              target_owner().bdc_state().serviced == (accepted ? 1U : 0U) &&
              received == (accepted ? 1U : 0U) &&
              (target_owner().bdc_state().control | 0x40U) == active_status &&
              target_owner().run_state().retired_instructions == 0,
          "Reset-held BKGD release and explicit request each SYNC; target remains genuinely held "
          "without instruction injection");
  const auto image = host.bus().power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Original native helper preserves all MCU NVM/knowledge/history");
  for (unsigned n = 0; n < helper.size(); ++n) {
    require(host.bus().peek(static_cast<std::uint16_t>(helper_base + n)) == helper[n],
            "Complete original helper remains byte-identical");
  }
  constexpr unsigned vector = 0xfffe;
  const auto target_image = target_owner().memory().flash_image();
  require(!target_owner().cpu_state().pc_known &&
              !target_image.known[vector - hcs08_flash::first] &&
              !target_image.known[vector + 1 - hcs08_flash::first],
          "Held original reset/SYNC preserves unknown target PC and both unread vector bytes");
  require(!target_owner().memory().flash_state().divider_loaded &&
              !target_owner().memory().flash_state().active &&
              target_owner().memory().periods().attempted == 0,
          "Reset/SYNC cannot invent an NVM clock, command or completion");
  require(target_image == board_wire::blank(),
          "The entire declared target NVM data,knownness and erase history remain unchanged");
  for (unsigned address = Hcs08BoardMemory::ram_first; address < Hcs08BoardMemory::ram_end;
       ++address) {
    require(target_owner().memory().inspect_ram(static_cast<std::uint16_t>(address)).known == 0,
            "No target CPU or memory command supplies any of the1024SRAM byte masks");
  }
  trace << "RETURN accepted=" << accepted << " steps=" << steps
        << " commands=" << target_owner().bdc_state().serviced
        << " syncs=" << target_owner().wire_state().syncs
        << " target-retired=" << target_owner().run_state().retired_instructions << '\n';
}

Saved Fixture::checkpoint() const {
  require(reset && target && reset_sequence().state().phase == Hcs08ResetPhase::ready,
          "Joint snapshot follows actual completed reset into production target ownership");
  return {.host = host.checkpoint(),
          .target = target_owner().checkpoint(),
          .reset = reset_sequence(),
          .next_edge = next_edge,
          .origin = origin,
          .steps = steps,
          .received = received};
}

void Fixture::restore(const Saved &saved) {
  target.emplace(saved.target, BLOFELD_HCS08_SLA);
  reset = saved.reset;
  host.restore(saved.host);
  next_edge = saved.next_edge;
  origin = saved.origin;
  steps = saved.steps;
  received = saved.received;
  bind();
}

std::string Fixture::endpoint() const {
  const auto h = host.cpu().state();
  const auto t = target_owner().cpu_state();
  std::ostringstream out;
  out << h.pc << ' ' << h.d << ' ' << h.x << ' ' << h.y << ' ' << h.sp << ' '
      << static_cast<unsigned>(h.ccr) << ' ' << host.run_clock().elapsed_ticks() << ' '
      << host.run_clock().bus_cycles() << ' ' << t.pc << ' ' << t.pc_known << ' ' << t.hx << ' '
      << t.sp << ' ' << static_cast<unsigned>(t.a) << ' ' << static_cast<unsigned>(t.ccr) << ' '
      << static_cast<unsigned>(t.ccr_known) << ' ' << next_edge << ' ' << origin << ' ' << steps
      << ' ' << received;
  return out.str();
}
}
