// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_timer.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
// Independent register/edge/clock expectations: TIM16B8C guide sections
// 1.3.2 and 1.4; UF32 integration and PIM module-routing tables.
constexpr unsigned tios = 0x40;
constexpr unsigned force = 0x41;
constexpr unsigned mask7 = 0x42;
constexpr unsigned data7 = 0x43;
constexpr unsigned counter_address = 0x44;
constexpr unsigned control1 = 0x46;
constexpr unsigned toggle_overflow = 0x47;
constexpr unsigned outputs_high = 0x48;
constexpr unsigned outputs_low = 0x49;
constexpr unsigned edges_high = 0x4a;
constexpr unsigned edges_low = 0x4b;
constexpr unsigned interrupt_enable = 0x4c;
constexpr unsigned control2 = 0x4d;
constexpr unsigned flags1 = 0x4e;
constexpr unsigned flags2 = 0x4f;
constexpr unsigned channel0 = 0x50;
constexpr unsigned channel7 = 0x5e;
constexpr unsigned timer_on = 0x80;
constexpr unsigned timer_fast_clear = 0x90;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_wrap = 65536;
constexpr unsigned word_max = 65535;
constexpr unsigned idle_cycles = 100;
constexpr unsigned channels_end = 0x60;
constexpr unsigned module_routing = 0x257;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

unsigned read_word(auto &device, const std::uint16_t address) {
  const auto low_address =
      static_cast<std::uint16_t>((static_cast<unsigned>(address) + 1) & word_max);
  return (static_cast<unsigned>(device.read(address)) << 8U) | device.read(low_address);
}

struct WordWrite {
  std::uint16_t address = 0;
  unsigned value = 0;
};

void write_word(blofeld::McuTimer &timer, const WordWrite word) {
  timer.write({.address = word.address,
               .value = static_cast<std::uint8_t>((word.value >> 8U) & byte_mask)});
  timer.write({.address = static_cast<std::uint16_t>(word.address + 1),
               .value = static_cast<std::uint8_t>(word.value & byte_mask)});
}

void test_reset_and_prescalers() {
  for (unsigned selection = 0; selection < 8; ++selection) {
    blofeld::McuTimer timer;
    for (unsigned address = tios; address < channels_end; ++address) {
      require(timer.read(static_cast<std::uint16_t>(address)) == 0,
              "Timer registers reset to zero");
    }
    timer.advance(idle_cycles);
    require(read_word(timer, counter_address) == 0, "Disabled TEN stops the timer");
    timer.write({.address = control2, .value = static_cast<std::uint8_t>(selection)});
    timer.write({.address = control1, .value = timer_on});
    const auto divisor = 1U << selection;
    timer.advance(divisor - 1);
    require(read_word(timer, counter_address) == 0, "No count before a complete prescaler period");
    timer.advance(1);
    require(read_word(timer, counter_address) == 1, "All eight documented prescaler divisors");
    constexpr unsigned further_counts = 17;
    timer.advance(divisor * further_counts);
    require(read_word(timer, counter_address) == further_counts + 1,
            "Prescaler phase persists between slices");
    timer.reset();
    require(read_word(timer, counter_address) == 0 && timer.read(control1) == 0,
            "External reset clears timer state");
  }
}

void test_delayed_prescaler_update_and_stop() {
  blofeld::McuTimer timer;
  timer.write({.address = control1, .value = timer_on});
  timer.advance(3);
  timer.write({.address = control2, .value = 2});
  constexpr unsigned before_sync = 124;
  timer.advance(before_sync);
  constexpr unsigned before_boundary = 127;
  require(read_word(timer, counter_address) == before_boundary && timer.read(control2) == 2,
          "Requested divisor does not replace active divisor before prescaler synchronization");
  timer.advance(1);
  timer.advance(3);
  require(read_word(timer, counter_address) == 128, "New divide-by-four waits a whole period");
  timer.advance(1);
  constexpr unsigned after_update = 129;
  require(read_word(timer, counter_address) == after_update, "Prescaler update at all-zero stages");
  timer.write({.address = control1, .value = 0});
  timer.advance(idle_cycles);
  require(read_word(timer, counter_address) == after_update, "TEN clear retains the count");
}

void test_all_capture_channels_and_edges() {
  for (unsigned channel = 0; channel < 8; ++channel) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      blofeld::McuTimer timer;
      const auto bit = static_cast<std::uint8_t>(1U << channel);
      timer.observe_inputs({.known = byte_mask});
      const auto edge_address = static_cast<std::uint16_t>(channel < 4 ? edges_low : edges_high);
      timer.write({.address = edge_address,
                   .value = static_cast<std::uint8_t>(mode << ((channel % 4) * 2))});
      timer.write({.address = control1, .value = timer_on});
      constexpr unsigned rise_time = 17;
      constexpr unsigned fall_time = 30;
      timer.advance(rise_time);
      timer.observe_inputs({.known = byte_mask, .levels = bit});
      const auto channel_address = static_cast<std::uint16_t>(channel0 + (channel * 2));
      const bool rising = mode == 1 || mode == 3;
      require(read_word(timer, channel_address) == (rising ? rise_time : 0),
              "Rising-edge selection on each channel");
      timer.advance(fall_time - rise_time);
      timer.observe_inputs({.known = byte_mask});
      auto expected = rising ? rise_time : 0;
      if (mode >= 2) { expected = fall_time; }
      require(read_word(timer, channel_address) == expected,
              "Falling/any/disabled capture edge selection");
      require(timer.read(flags1) == (mode == 0 ? 0 : bit),
              "Capture flags match only enabled edges");
      write_word(timer, {.address = channel_address, .value = word_max});
      require(read_word(timer, channel_address) == expected,
              "Writes to input-capture registers have no effect");
    }
  }
}

void test_capture_flags_and_unknown_inputs() {
  blofeld::McuTimer timer;
  timer.write({.address = edges_low, .value = 3});
  timer.write({.address = control1, .value = timer_on});
  reject([&timer] { timer.advance(1); }, "Unknown capture input cannot invent absent edges");
  reject([&timer] { timer.observe_inputs({.known = 1}); },
         "Unknown input history cannot invent a first edge");
  timer.write({.address = control1, .value = 0});
  timer.observe_inputs({.known = 1});
  timer.write({.address = control1, .value = timer_on});
  timer.advance(5);
  timer.observe_inputs({.known = 1, .levels = 1});
  reject([&timer] { timer.observe_inputs({.known = 1}); },
         "Synchronized input transitions require an intervening bus clock");
  timer.advance(7);
  timer.observe_inputs({.known = 1});
  require(read_word(timer, channel0) == 12 && timer.read(flags1) == 1,
          "New captures overwrite the previous value");
  timer.write({.address = flags1, .value = 0});
  require(timer.read(flags1) == 1, "Writing zero does not clear CnF");
  timer.write({.address = control1, .value = 0});
  timer.write({.address = flags1, .value = 1});
  require(timer.read(flags1) == 1, "TEN must remain enabled for W1C");
  timer.write({.address = control1, .value = timer_fast_clear});
  static_cast<void>(timer.read(channel0));
  require(timer.read(flags1) == 0, "TFFCA capture access acknowledges its own channel");
  reject([&timer] { timer.observe_inputs({.known = 0, .levels = 1}); },
         "Invalid input-known masks fail");
}

void test_overflow_and_fast_clear() {
  blofeld::McuTimer timer;
  timer.write({.address = control1, .value = timer_on});
  timer.advance(word_wrap);
  require(read_word(timer, counter_address) == 0 && timer.read(flags2) == timer_on,
          "FFFF to zero sets TOF");
  timer.write({.address = flags2, .value = 0});
  timer.write({.address = control1, .value = 0});
  timer.write({.address = flags2, .value = timer_on});
  require(timer.read(flags2) == timer_on, "Overflow W1C is disabled with TEN clear");
  timer.write({.address = control1, .value = timer_fast_clear});
  static_cast<void>(timer.read(counter_address + 1));
  require(timer.read(flags2) == 0, "Any TCNT access clears TOF with TFFCA");
  timer.advance(word_wrap);
  constexpr unsigned ignored_count = 0xabcd;
  write_word(timer, {.address = counter_address, .value = ignored_count});
  require(read_word(timer, counter_address) == 0 && timer.read(flags2) == 0,
          "Normal-mode TCNT writes are ignored while still performing fast flag clearing");
}

void test_compare_force_and_fast_clear() {
  blofeld::McuTimer timer;
  timer.write({.address = tios, .value = 1});
  write_word(timer, {.address = channel0, .value = 3});
  timer.write({.address = outputs_low, .value = 3});
  require(timer.pins().outputs == 0, "Disabled timer leaves GPIO ownership");
  timer.write({.address = force, .value = 1});
  require(timer.read(force) == 0 && timer.read(flags1) == 0,
          "Forced compare does not set its flag");
  timer.write({.address = control1, .value = timer_fast_clear});
  require(timer.pins().levels == 1 && timer.pins().unknown == 0,
          "Forced action initializes the output latch");
  timer.advance(3);
  require(timer.read(flags1) == 1 && timer.pins().levels == 1,
          "A real compare sets flag and output");
  static_cast<void>(read_word(timer, channel0));
  require(timer.read(flags1) == 1, "Output-compare reads do not perform capture fast clear");
  timer.write({.address = tios, .value = 1});
  timer.advance(1);
  require(read_word(timer, counter_address) == 4,
          "Writing unchanged TIOS does not reactivate a matched comparator");
  write_word(timer, {.address = channel0, .value = 7});
  require(timer.read(flags1) == 0, "Output-compare writes acknowledge their flag with TFFCA");
  timer.write({.address = outputs_low, .value = 1});
  timer.write({.address = force, .value = 1});
  require(timer.pins().levels == 0 && timer.read(flags1) == 0,
          "Forced toggle updates output without CnF");
  timer.write({.address = force, .value = 1});
  require(timer.pins().levels == 1, "Repeated forced toggle uses the saved timer latch");
}

void test_all_compare_channel_modes() {
  for (unsigned channel = 0; channel < 8; ++channel) {
    for (unsigned mode = 0; mode < 4; ++mode) {
      blofeld::McuTimer timer;
      const auto bit = static_cast<std::uint8_t>(1U << channel);
      const auto address = static_cast<std::uint16_t>(channel0 + (channel * 2));
      const auto control = static_cast<std::uint16_t>(channel < 4 ? outputs_low : outputs_high);
      const auto shift = (channel % 4) * 2;
      timer.write({.address = tios, .value = bit});
      write_word(timer, {.address = address, .value = 2});
      timer.write({.address = control, .value = static_cast<std::uint8_t>(2U << shift)});
      timer.write({.address = force, .value = bit});
      timer.write({.address = control, .value = static_cast<std::uint8_t>(mode << shift)});
      timer.write({.address = control1, .value = timer_on});
      timer.advance(2);
      const auto expected = mode == 1 || mode == 3 ? bit : 0;
      require(timer.pins().levels == expected && timer.read(flags1) == bit,
              "Each output channel supports disconnected, toggle, clear and set actions");
      require(timer.pins().outputs == (mode == 0 ? 0 : bit),
              "Compare action controls pad ownership");
    }
  }
}

void test_channel7_priority_and_counter_reset() {
  blofeld::McuTimer timer;
  constexpr unsigned channels = 0x81;
  timer.write({.address = tios, .value = channels});
  write_word(timer, {.address = channel0, .value = 3});
  write_word(timer, {.address = channel7, .value = 3});
  timer.write({.address = outputs_low, .value = 1});
  timer.write({.address = mask7, .value = channels});
  timer.write({.address = data7, .value = 1});
  timer.write({.address = control1, .value = timer_on});
  require(timer.pins().unknown == channels, "Unspecified initial timer latches remain unknown");
  timer.advance(3);
  require(timer.pins().levels == 1 && timer.pins().unknown == 0 && timer.read(flags1) == channels,
          "Channel 7 override wins simultaneous compares and resolves unknown masked latches");
  timer.reset();
  timer.write({.address = tios, .value = timer_on});
  write_word(timer, {.address = channel7, .value = 3});
  timer.write({.address = control2, .value = 9}); // TCRE and divide by two.
  timer.write({.address = control1, .value = timer_on});
  timer.advance(6);
  require(read_word(timer, counter_address) == 3,
          "TC7 is held for one bus cycle even with prescaling");
  timer.advance(1);
  require(read_word(timer, counter_address) == 0 && timer.read(flags2) == 0,
          "TCRE resets on the following bus cycle");
  timer.advance(6);
  require(read_word(timer, counter_address) == 3, "TC7 times prescaler plus one bus-cycle period");
  timer.reset();
  timer.write({.address = tios, .value = timer_on});
  write_word(timer, {.address = channel7, .value = word_max});
  timer.write({.address = control2, .value = 8});
  timer.write({.address = control1, .value = timer_on});
  timer.advance(word_wrap);
  require(read_word(timer, counter_address) == 0 && timer.read(flags2) == 0,
          "TC7=FFFF counter reset suppresses ordinary overflow");
}

void test_overflow_toggle_and_unsupported_cases() {
  blofeld::McuTimer timer;
  timer.write({.address = tios, .value = 1});
  write_word(timer, {.address = channel0, .value = 1});
  timer.write({.address = outputs_low, .value = 2});
  timer.write({.address = force, .value = 1});
  timer.write({.address = outputs_low, .value = 1});
  timer.write({.address = toggle_overflow, .value = 1});
  timer.write({.address = control1, .value = timer_on});
  timer.advance(1);
  require(timer.pins().levels == 1, "Compare toggles initialized latch");
  timer.advance(word_max);
  require(timer.pins().levels == 0 && timer.read(flags2) == timer_on,
          "TTOV toggles on a counter overflow");
  reject([&timer] { timer.write({.address = interrupt_enable, .value = 1}); },
         "Unimplemented channel interrupts fail");
  reject([&timer] { timer.write({.address = control2, .value = timer_on}); },
         "Unimplemented overflow interrupts fail");
  constexpr unsigned pulse_control = 0x60;
  reject([&timer] { static_cast<void>(timer.read(pulse_control)); },
         "Pulse accumulator is not a successful stub");
  timer.reset();
  timer.write({.address = tios, .value = 1});
  timer.write({.address = control1, .value = timer_on});
  reject([&timer] { timer.advance(1); }, "Unverified current-count compare activation fails");
}

void test_bus_capture_routing_and_mux_ownership() {
  const std::array<std::uint8_t, 32768> flash{};
  blofeld::McuBus bus(flash);
  bus.set_pin_drive(blofeld::McuPort::t, {.mask = 15});
  bus.sample_timer_inputs(3);
  bus.write(edges_low, 6);
  bus.write(control1, timer_fast_clear);
  bus.advance_timer(10);
  bus.set_pin_drive(blofeld::McuPort::t, {.mask = 15, .levels = 2});
  bus.sample_timer_inputs(3);
  require(read_word(bus, channel0 + 2) == 10, "Routed channel 1 rising edge captures TCNT");
  bus.advance_timer(5);
  bus.set_pin_drive(blofeld::McuPort::t, {.mask = 15, .levels = 1});
  bus.sample_timer_inputs(3);
  bus.advance_timer(1);
  bus.set_pin_drive(blofeld::McuPort::t, {.mask = 15});
  bus.sample_timer_inputs(3);
  require(read_word(bus, channel0) == 16 && bus.read(flags1) == 0,
          "Channel 0 falling capture and per-channel fast clear");
  bus.write(module_routing, 1);
  constexpr unsigned m_external = 0x37;
  constexpr unsigned b_external = 0xfb;
  bus.set_pin_drive(blofeld::McuPort::m, {.mask = m_external});
  bus.set_pin_drive(blofeld::McuPort::b, {.mask = b_external});
  bus.write(tios, 4);
  bus.write(channel0 + 4, 0);
  constexpr unsigned future_compare = 18;
  bus.write(channel0 + 5, future_compare);
  constexpr unsigned channel2_set = 0x30;
  bus.write(outputs_low, channel2_set);
  bus.write(force, 4);
  constexpr unsigned port_m_input = 0x249;
  require(bus.read(port_m_input) == 8 && bus.read(1) == 4,
          "64-pin timer output drives PM3 and the documented PB2 mirror");
  bus.write(module_routing, 5);
  constexpr unsigned all_m_pins = 0x3f;
  bus.set_pin_drive(blofeld::McuPort::m, {.mask = all_m_pins});
  require(bus.read(port_m_input) == 0 && bus.read(1) == 4,
          "Optional timer route releases PM3 and uses Port B only");
  constexpr unsigned s_external = 0xdf;
  bus.set_pin_drive(blofeld::McuPort::s, {.mask = s_external});
  constexpr unsigned sci_control = 0xcb;
  bus.write(sci_control, 8);
  constexpr unsigned port_s_input = 0x271;
  require(bus.read(port_s_input) == 0x20 && bus.read(1) == 4,
          "SCI and timer retain independent pad ownership");
}

void test_unknown_peripheral_pad_latches() {
  blofeld::McuGpio gpio;
  gpio.set_function(blofeld::McuPort::s, {.outputs = 1, .unknown = 1});
  gpio.set_drive(blofeld::McuPort::s, {.mask = 1, .levels = 1});
  reject([&gpio] { static_cast<void>(gpio.read_pad(blofeld::McuPort::s, 1)); },
         "An external driver cannot silently resolve an unknown internal output latch");
  gpio.set_function(blofeld::McuPort::s, {.outputs = 1, .levels = 1});
  require(gpio.read_pad(blofeld::McuPort::s, 1) == 1, "A known output resolves the peripheral pad");
}
}

int main() {
  try {
    test_reset_and_prescalers();
    test_delayed_prescaler_update_and_stop();
    test_all_capture_channels_and_edges();
    test_capture_flags_and_unknown_inputs();
    test_overflow_and_fast_clear();
    test_compare_force_and_fast_clear();
    test_all_compare_channel_modes();
    test_channel7_priority_and_counter_reset();
    test_overflow_toggle_and_unsupported_cases();
    test_bus_capture_routing_and_mux_ownership();
    test_unknown_peripheral_pad_latches();
    std::cout << "Timer counting, captures, compare actions, flags and routing checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
