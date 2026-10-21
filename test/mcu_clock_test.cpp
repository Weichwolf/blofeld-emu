// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_clock.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {
// Independent expectations: 9S12UF32DGV1 A.4.3, S12CRGUV1 section 3,
// S12USB20D6E2FV1 section 3.1.1. All test addresses are literal reference values.
constexpr unsigned reference_divider = 0x35;
constexpr unsigned flags = 0x37;
constexpr unsigned clock_select = 0x39;
constexpr unsigned test_flags = 0x36;
constexpr unsigned interrupt_enable = 0x38;
constexpr unsigned rti_control = 0x3b;
constexpr unsigned usb_high = 0x300;
constexpr unsigned usb_low = 0x301;
constexpr unsigned clock_enabled = 0x40;
constexpr unsigned clock_suspended = 0x48;
constexpr unsigned pll_selected = 0x80;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned por_and_lock = 0x58;
constexpr unsigned por_flag = 0x40;
constexpr unsigned usb_disconnect = 0x20;
constexpr unsigned usb_configuration_mask = 0x3f;
constexpr unsigned all_clock_options_except_cop = 0xb6;
constexpr unsigned lock_change_flag = 0x10;
constexpr unsigned realtime_flag = 0x80;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void test_reset_and_usb_gates() {
  blofeld::McuClock clock;
  require(clock.read(flags) == por_flag && clock.read(usb_high) == 8 &&
              clock.read(usb_low) == usb_disconnect && !clock.phy_running() &&
              !clock.pll_selected(),
          "POR status and UMCR reset 0820 with PHY suspended");
  clock.write({.address = usb_high, .value = 0});
  clock.write({.address = usb_low, .value = byte_mask});
  require(clock.read(usb_high) == 8 && clock.read(usb_low) == usb_disconnect,
          "MCE=0 blocks PHY suspend and low configuration writes");
  reject([&clock] { clock.observe_phy({.edges = 15}); },
         "Suspended PHY cannot supply detector edges");
  clock.write({.address = usb_high, .value = clock_enabled});
  require(clock.phy_running() && clock.read(usb_high) == clock_enabled,
          "MCE and SPHY in the same high-byte write start PHY operation");
  reject([&clock] { static_cast<void>(clock.read(flags)); },
         "PHY startup cannot invent a measured LOCK state");
  clock.write({.address = clock_select, .value = pll_selected});
  reject([&clock] { static_cast<void>(clock.pll_selected()); },
         "Unknown PHY qualification cannot become a known system clock source");
  clock.write({.address = clock_select, .value = 0});
  clock.write({.address = usb_low, .value = byte_mask});
  require(clock.read(usb_low) == usb_configuration_mask, "Reserved USB low bits do not latch");
  clock.write({.address = usb_high, .value = 0});
  clock.write({.address = usb_low, .value = 0});
  require(clock.read(usb_low) == usb_configuration_mask && clock.phy_running(),
          "MCE gates controller writes; SPHY still controls the PHY clock");
  clock.observe_phy({.edges = 15});
  require(clock.read(flags) == por_and_lock, "Controller clock gate does not suspend the PHY");
}

void test_suspend_and_restart() {
  blofeld::McuClock clock;
  clock.write({.address = usb_high, .value = clock_enabled});
  clock.observe_phy({.edges = 15});
  clock.write({.address = flags, .value = lock_change_flag});
  clock.write({.address = usb_high, .value = clock_suspended});
  constexpr unsigned por_and_lock_loss = 0x50;
  require(clock.read(flags) == por_and_lock_loss && !clock.phy_running(),
          "Suspending a locked PHY drops LOCK and sets LOCKIF");
  clock.write({.address = flags, .value = lock_change_flag});
  clock.write({.address = usb_high, .value = clock_enabled});
  reject([&clock] { static_cast<void>(clock.read(flags)); },
         "PHY restart requires fresh detector observations");
  clock.observe_phy({.edges = 0});
  require(clock.read(flags) == por_flag, "An unlocked restart does not invent a lock transition");
}

void test_detector_and_sticky_flags() {
  blofeld::McuClock clock;
  clock.write({.address = usb_high, .value = clock_enabled});
  constexpr std::array<unsigned, 4> unqualified_edges{0, 14, 16, 100};
  for (const auto edges : unqualified_edges) {
    clock.observe_phy({.edges = edges});
    require(clock.read(flags) == por_flag, "Unqualified PHY cannot set LOCK or LOCKIF");
  }
  clock.observe_phy({.edges = 15});
  require(clock.read(flags) == por_and_lock, "15 edges in three OSC periods qualify the PHY");
  clock.write({.address = flags, .value = 0});
  require(clock.read(flags) == por_and_lock, "Writing zero preserves all status flags");
  clock.write({.address = flags, .value = byte_mask});
  require(clock.read(flags) == 8, "W1C clears PORF/LOCKIF while LOCK is read-only");
  clock.observe_phy({.edges = 15});
  require(clock.read(flags) == 8, "Repeated locked windows do not reassert LOCKIF");
  clock.observe_phy({.edges = 14});
  require(clock.read(flags) == lock_change_flag, "Loss of lock raises sticky LOCKIF");
  clock.observe_phy({.edges = 0});
  require(clock.read(flags) == lock_change_flag,
          "LOCKIF remains set across repeated unlocked windows");
  clock.reset();
  require(clock.read(flags) == 0, "External reset does not recreate a software-cleared PORF");
  blofeld::McuClock fresh;
  fresh.reset();
  require(fresh.read(flags) == por_flag, "External reset retains uncleared PORF");
}

void test_pll_selection_and_write_protection() {
  blofeld::McuClock clock;
  clock.write({.address = reference_divider, .value = byte_mask});
  require(clock.read(reference_divider) == 15, "REFDV implements four divider bits");
  clock.write({.address = clock_select, .value = pll_selected});
  require(!clock.pll_selected(), "PLLSEL falls back to OSC while PHY is unlocked");
  clock.write({.address = reference_divider, .value = 0});
  clock.write({.address = usb_high, .value = clock_enabled});
  require(clock.read(reference_divider) == 15 && !clock.phy_running(),
          "PLLSEL prevents REFDV and SPHY writes even without lock");
  clock.write({.address = clock_select, .value = 1});
  require(clock.read(clock_select) == 0, "First CLKSEL write freezes COPWAI even when zero");
  clock.write({.address = usb_high, .value = clock_enabled});
  clock.observe_phy({.edges = 15});
  clock.write({.address = clock_select, .value = byte_mask});
  require(clock.read(clock_select) == all_clock_options_except_cop && clock.pll_selected(),
          "CLKSEL masks reserved bits and selects a qualified PHY clock");
  clock.write({.address = usb_high, .value = clock_suspended});
  require(clock.phy_running(), "PLLSEL protects PHY suspension");
  clock.observe_phy({.edges = 14});
  require(!clock.pll_selected() && clock.read(clock_select) == all_clock_options_except_cop,
          "Loss of lock falls back without inventing a new PLLSEL register value");
  clock.write({.address = clock_select, .value = 0});
  clock.write({.address = usb_high, .value = clock_suspended});
  require(!clock.phy_running(), "Clearing PLLSEL permits PHY suspension");
  clock.reset();
  clock.write({.address = clock_select, .value = 1});
  clock.write({.address = clock_select, .value = 0});
  require(clock.read(clock_select) == 1, "External reset rearms COPWAI write-once protection");
}

void test_unimplemented_usb_commands_and_interrupts() {
  blofeld::McuClock clock;
  constexpr unsigned commands_without_clock = 0x36;
  clock.write({.address = usb_high, .value = commands_without_clock});
  require(clock.read(usb_high) == 8, "MCE=0 blocks reset and resume commands");
  clock.write({.address = usb_high, .value = clock_enabled});
  constexpr std::array<unsigned, 3> commands{0x60, 0x50, 0x44};
  for (const auto command : commands) {
    reject(
        [&clock, command] {
          clock.write({.address = usb_high, .value = static_cast<std::uint8_t>(command)});
        },
        "Unimplemented USB commands must fail");
    require(clock.read(usb_high) == clock_enabled, "Rejected commands preserve control state");
  }
  constexpr unsigned endpoint_reset = 0x42;
  require(clock.write({.address = usb_high, .value = endpoint_reset}) ==
                  blofeld::UsbControlCommand::endpoint_reset &&
              clock.read(usb_high) == clock_enabled,
          "Legal EPRST emits one bus action and always reads back zero");
  clock.write({.address = clock_select, .value = pll_selected});
  constexpr unsigned phy_reset = 0x44;
  clock.write({.address = usb_high, .value = phy_reset});
  require(clock.read(usb_high) == clock_enabled, "PLLSEL blocks PHY reset commands");
  clock.write({.address = test_flags, .value = byte_mask});
  require(clock.read(test_flags) == 0, "Factory flags are inert in normal mode");
  constexpr unsigned reserved_interrupt_bits = 0x6f;
  clock.write({.address = interrupt_enable, .value = reserved_interrupt_bits});
  require(clock.read(interrupt_enable) == 0, "Reserved interrupt enable bits do not latch");
  require(!clock.lock_interrupt_pending(), "Disabled lock IRQ does not sample unknown clock state");
  clock.write({.address = interrupt_enable, .value = lock_change_flag});
  reject([&clock] { static_cast<void>(clock.lock_interrupt_pending()); },
         "Enabled lock IRQ cannot assume unknown PHY qualification");
  clock.observe_phy({.edges = 15});
  require(clock.lock_interrupt_pending(), "Qualified PHY lock transition asserts enabled IRQ");
  clock.write({.address = flags, .value = lock_change_flag});
  require(!clock.lock_interrupt_pending(),
          "Lock IRQ polling does not fabricate or retain a cleared flag");
  clock.observe_phy({.edges = 14});
  require(clock.lock_interrupt_pending(), "Lock loss also asserts the lock-change IRQ");
  clock.reset();
  require(clock.read(interrupt_enable) == 0 && !clock.lock_interrupt_pending(),
          "Reset clears interrupt enable and pending lock requests");
  clock.write({.address = rti_control, .value = lock_change_flag});
  constexpr unsigned minimum_rti_period = 1024;
  clock.advance_oscillator(minimum_rti_period);
  require((clock.read(flags) & realtime_flag) != 0,
          "RTI configuration starts an oscillator-clocked counter");
  constexpr unsigned reserved_clock = 0x34;
  reject([&clock] { static_cast<void>(clock.read(reserved_clock)); },
         "Unknown clock registers fail");
}

void test_bus_mapping_and_clock_reset() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  bus.write(usb_high, clock_enabled);
  bus.observe_phy({.edges = 15});
  require(bus.read(flags) == por_and_lock, "Bus forwards actual clock-window observations");
  constexpr unsigned initrg = 0x11;
  constexpr unsigned relocated_base = 0x1000;
  bus.write(initrg, 0x10);
  bus.retire_instruction();
  require(bus.read(relocated_base + flags) == por_and_lock &&
              bus.read(relocated_base + usb_high) == clock_enabled,
          "Register relocation also relocates USB and CRG access");
  bus.write(relocated_base + flags, byte_mask);
  bus.reset();
  require(bus.read(usb_high) == 8 && bus.read(flags) == 0,
          "Board reset suspends PHY, restores registers and preserves cleared PORF");
}

void test_original_boot_clock_boundary(const std::filesystem::path &reference) {
  if (!std::filesystem::is_regular_file(reference)) {
    std::cout << "SKIP original clock boundary: private firmware is absent\n";
    return;
  }
  const auto firmware = blofeld::Firmware::load(reference);
  blofeld::McuBus bus(firmware.mcu);
  constexpr unsigned driven_inputs = 0xfb;
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = driven_inputs});
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  constexpr unsigned before_usb = 34;
  for (unsigned index = 0; index < before_usb; ++index) { cpu.step(); }
  constexpr unsigned usb_setup_pc = 0x8046;
  constexpr unsigned polling_pc = 0x8049;
  constexpr unsigned pll_select_pc = 0x804d;
  constexpr unsigned sci_setup_pc = 0x8058;
  require(cpu.state().pc == usb_setup_pc, "Original reset reaches USB PHY setup");
  cpu.step();
  require(bus.read(usb_high) == clock_enabled && bus.read(usb_low) == 0,
          "Original STD enables controller clock and releases PHY suspension");
  reject([&bus] { static_cast<void>(bus.read(flags)); }, "No startup lock is fabricated");
  bus.observe_phy({.edges = 14});
  cpu.step();
  require(cpu.state().pc == polling_pc, "Original polling branch waits for a qualified PHY");
  bus.observe_phy({.edges = 15});
  cpu.step();
  require(cpu.state().pc == pll_select_pc,
          "Original polling branch exits on detector qualification");
  cpu.step();
  require(bus.read(clock_select) == pll_selected, "Original firmware selects the PHY clock");
  bus.observe_phy({.edges = 0});
  constexpr unsigned por_and_loss = 0x50;
  require(bus.read(flags) == por_and_loss,
          "Clock loss remains visible after original PLL selection");
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().pc == sci_setup_pc, "Original reset reaches SCI setup after clock selection");
  constexpr unsigned sci_setup_steps = 6;
  for (unsigned index = 0; index < sci_setup_steps; ++index) { cpu.step(); }
  constexpr unsigned timer_setup_pc = 0x8063;
  constexpr unsigned sci_baud_low = 0xc9;
  constexpr unsigned sci_control2 = 0xcb;
  constexpr unsigned midi_divider = 60;
  require(bus.read(sci_baud_low) == midi_divider && bus.read(sci_control2) == 12,
          "Original firmware sets the MIDI baud divisor and enables RX/TX");
  require(cpu.state().pc == timer_setup_pc, "Original reset reaches timer configuration");
  cpu.step();
  constexpr unsigned timer_prescaler = 0x4d;
  require(bus.read(timer_prescaler) == 6, "Original reset selects a divide-by-64 timer prescaler");
}
}

int main(const int count, char **arguments) {
  try {
    test_reset_and_usb_gates();
    test_suspend_and_restart();
    test_detector_and_sticky_flags();
    test_pll_selection_and_write_protection();
    test_unimplemented_usb_commands_and_interrupts();
    test_bus_mapping_and_clock_reset();
    if (count == 2) { test_original_boot_clock_boundary(arguments[1]); }
    std::cout << "USB PHY control and CRG clock qualification checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
