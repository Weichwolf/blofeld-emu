// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
namespace reg = blofeld::hc12;
using blofeld::McuPort;
constexpr unsigned tios = 0x40;
constexpr unsigned force = 0x41;
constexpr unsigned output_high = 0x48;
constexpr unsigned output_low = 0x49;
constexpr unsigned counter_low = 0x45;
constexpr unsigned timer_on = 0x80;
constexpr unsigned context_pc = 0x8123;
constexpr unsigned context_ccr = 0x90;
constexpr unsigned frame_cycles = 160;
constexpr unsigned frame_byte = 0x55;
constexpr std::uint16_t r_input = 0x269;
constexpr std::uint16_t s_input = 0x271;
constexpr std::array<std::uint8_t, 32768> flash{};

struct Pad {
  McuPort port;
  std::uint8_t mask;
  std::uint16_t input;
};

// Independent UF32 integration-table expectations, not a production mapper call.
constexpr std::array<Pad, 8> large_pads{{{.port = McuPort::t, .mask = 1, .input = 0x241},
                                         {.port = McuPort::t, .mask = 2, .input = 0x241},
                                         {.port = McuPort::t, .mask = 4, .input = 0x241},
                                         {.port = McuPort::t, .mask = 8, .input = 0x241},
                                         {.port = McuPort::r, .mask = 0x10, .input = r_input},
                                         {.port = McuPort::r, .mask = 0x20, .input = r_input},
                                         {.port = McuPort::r, .mask = 0x40, .input = r_input},
                                         {.port = McuPort::r, .mask = 0x80, .input = r_input}}};
constexpr std::array<Pad, 8> small_pads{{{.port = McuPort::t, .mask = 1, .input = 0x241},
                                         {.port = McuPort::t, .mask = 2, .input = 0x241},
                                         {.port = McuPort::m, .mask = 8, .input = 0x249},
                                         {.port = McuPort::m, .mask = 0x10, .input = 0x249},
                                         {.port = McuPort::q, .mask = 0x10, .input = 0x261},
                                         {.port = McuPort::q, .mask = 0x20, .input = 0x261},
                                         {.port = McuPort::q, .mask = 0x40, .input = 0x261},
                                         {.port = McuPort::q, .mask = 0x80, .input = 0x261}}};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void check_pad(const blofeld::McuBus &bus, const Pad pad, const bool high) {
  const auto sample = bus.observe_pads(pad.port, pad.mask);
  require(sample.known == pad.mask && sample.levels == (high ? pad.mask : 0),
          "Peripheral pad must match the independent known-level expectation");
}

void reject(const auto &operation, const std::string &diagnostic) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.contains(diagnostic) && message.contains("pc=0x8123") &&
                message.contains("ccr=0x90"),
            "Peripheral fault must retain its operation, PC and CCR");
    return;
  }
  throw blofeld::Fault("Expected peripheral fault did not occur");
}

void timer_mode(blofeld::McuBus &bus, const unsigned channel, const unsigned mode) {
  bus.write(channel < 4 ? output_low : output_high,
            static_cast<std::uint8_t>(mode << ((channel % 4) * 2)));
}

void timer_case(const unsigned channel, const Pad pad, const std::uint8_t routing) {
  const auto bit = static_cast<std::uint8_t>(1U << channel);
  blofeld::McuBus bus(flash);
  bus.begin_instruction({.pc = context_pc, .ccr = context_ccr});
  bus.write(reg::modrr, routing);
  bus.write(tios, bit);
  // Keep the compare target at 0x0100, away from the unqualified TCNT=0 activation.
  bus.write(static_cast<std::uint16_t>(reg::timer_channel0 + (channel * 2)), 1);
  timer_mode(bus, channel, 1);
  bus.write(reg::tscr1, timer_on);
  bus.advance_timer(1);
  bus.set_pin_drive(pad.port, {.mask = pad.mask, .levels = pad.mask});
  require(bus.observe_pads(pad.port, pad.mask).known == 0,
          "A matching external drive cannot resolve an unknown timer latch");
  reject([&] { static_cast<void>(bus.read(pad.input)); }, "Unknown MCU peripheral output latch");
  bus.set_pin_drive(pad.port, {});
  timer_mode(bus, channel, 2);
  bus.write(force, bit);
  check_pad(bus, pad, false);
  bus.advance_timer(3);
  require(bus.read(counter_low) == 4, "Unchanged routing must not skip timer cycles");
  timer_mode(bus, channel, 3);
  bus.write(force, bit);
  check_pad(bus, pad, true);
  if (routing == 1) { check_pad(bus, {.port = McuPort::b, .mask = bit, .input = 1}, true); }
  auto clone = bus.clone();
  clone->advance_timer(1);
  check_pad(*clone, pad, true);
  timer_mode(*clone, channel, 2);
  clone->write(force, bit);
  check_pad(*clone, pad, false);
  check_pad(bus, pad, true);
  bus.set_pin_drive(pad.port, {.mask = pad.mask});
  reject([&] { bus.advance_timer(1); }, "MCU timer advance at 0x0040");
  require(bus.read(counter_low) == 5, "Contention is checked after the actual next timer cycle");
  bus.set_pin_drive(pad.port, {});
  bus.advance_timer(1);
  check_pad(bus, pad, true);
  require(bus.read(counter_low) == 6, "Stable routing retains checks after removing contention");
  bus.reset();
  require(bus.observe_pads(pad.port, pad.mask).known == 0, "Reset releases peripheral ownership");
  bus.write(reg::modrr, routing);
  bus.write(tios, bit);
  bus.write(static_cast<std::uint16_t>(reg::timer_channel0 + (channel * 2)), 1);
  timer_mode(bus, channel, 3);
  bus.write(force, bit);
  bus.write(reg::tscr1, timer_on);
  bus.advance_timer(1);
  check_pad(bus, pad, true);
}

void timer_routes() {
  for (unsigned channel = 0; channel < 8; ++channel) {
    timer_case(channel, large_pads.at(channel), 0);
    timer_case(channel, small_pads.at(channel), 1);
    timer_case(channel,
               {.port = McuPort::b, .mask = static_cast<std::uint8_t>(1U << channel), .input = 1},
               5);
  }
}

void sci_case(const std::uint8_t routing, const Pad tx, const Pad rx) {
  blofeld::McuBus bus(flash);
  bus.write(reg::modrr, routing);
  const auto data = static_cast<std::uint16_t>(tx.input - 1);
  const auto direction = static_cast<std::uint16_t>(tx.input + 1);
  bus.write(data, tx.mask);
  bus.write(direction, static_cast<std::uint8_t>(tx.mask | rx.mask));
  bus.write(reg::sci_baud_low, 1);
  bus.write(reg::sci_control2, 8);
  static_cast<void>(bus.read(reg::sci_status1));
  bus.write(reg::sci_data_low, frame_byte);
  check_pad(bus, tx, true);
  bus.advance_sci(frame_cycles);
  constexpr std::array<bool, 10> expected{
      false, true, false, true, false, true, false, true, false, true};
  for (const bool level : expected) {
    bus.advance_timer(1);
    check_pad(bus, tx, level);
    auto clone = bus.clone();
    clone->advance_timer(1);
    check_pad(*clone, tx, level);
    bus.advance_sci(16);
  }
  bus.write(reg::sci_control2, 4);
  bus.set_pin_drive(rx.port, {.mask = rx.mask, .levels = rx.mask});
  check_pad(bus, rx, true);
  bus.advance_timer(1);
  bus.set_pin_drive(rx.port, {.mask = rx.mask});
  check_pad(bus, rx, false);
  bus.set_pin_drive(rx.port, {});
  bus.write(reg::sci_control2, 0);
  bus.advance_timer(1);
  check_pad(bus, rx, false);
  check_pad(bus, tx, true);
  bus.write(data, 0);
  check_pad(bus, tx, false);
  bus.power_cycle();
  require(bus.observe_pads(tx.port, tx.mask).known == 0,
          "Power cycle releases serial ownership and GPIO directions");
}

void gpio_changes() {
  blofeld::McuBus bus(flash);
  constexpr Pad pad{.port = McuPort::q, .mask = 1, .input = 0x261};
  bus.write(reg::ptq, 1);
  bus.write(reg::ddrq, 1);
  bus.advance_timer(1);
  check_pad(bus, pad, true);
  bus.write(reg::ptq, 0);
  check_pad(bus, pad, false);
  bus.write(reg::ddrq, 0);
  bus.set_pin_drive(McuPort::q, {.floating = 1});
  require(bus.observe_pads(McuPort::q, 1).known == 0,
          "Floating input remains unknown without pull");
  bus.write(reg::perq, 1);
  check_pad(bus, pad, true);
  bus.write(reg::ppsq, 1);
  check_pad(bus, pad, false);
  bus.set_pin_drive(McuPort::q, {.mask = 1, .levels = 1});
  check_pad(bus, pad, true);
  bus.reset();
  check_pad(bus, pad, true);
  bus.set_pin_drive(McuPort::q, {});
  require(bus.observe_pads(McuPort::q, 1).known == 0, "Reset retains external drive, clears pull");
}
}

int main() {
  try {
    timer_routes();
    sci_case(0,
             {.port = McuPort::r, .mask = 8, .input = r_input},
             {.port = McuPort::r, .mask = 4, .input = r_input});
    sci_case(1,
             {.port = McuPort::s, .mask = 0x20, .input = s_input},
             {.port = McuPort::s, .mask = 0x10, .input = s_input});
    gpio_changes();
    std::cout << "24 timer routes, 2 serial waveforms, clone/reset and live GPIO checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
