// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/st7565.hpp"
#include "display/st7565_parallel.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string_view>

namespace {
namespace pin = blofeld::st7565_pin;
constexpr unsigned byte_mask = 255;
constexpr unsigned idle = pin::reset_n | pin::cs1_n;
constexpr unsigned selected = pin::reset_n | pin::cs2;
constexpr unsigned reading = selected | pin::read | pin::data;
constexpr unsigned writing = selected | pin::data;
constexpr unsigned reset_command = 0xe2;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string_view reason) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    require(std::string_view(fault.what()).contains(reason), "LCD bus fault identifies its cause");
    return;
  }
  throw blofeld::Fault("Expected an explicit LCD bus qualification fault");
}

blofeld::LcdHostPins pins(const unsigned controls,
                          const std::optional<std::uint8_t> value = std::nullopt) {
  return {.control = static_cast<std::uint8_t>(controls),
          .known = pin::all,
          .drive = static_cast<std::uint8_t>(value ? byte_mask : 0),
          .data = {.value = value.value_or(0),
                   .known = static_cast<std::uint8_t>(value ? byte_mask : 0)}};
}

void reset(blofeld::St7565Parallel &bus) {
  const auto result = bus.sample({.known = pin::reset_n});
  require(result.drive == 0 && !result.completed, "Reset aborts rather than completes a cycle");
  bus.sample(pins(idle));
}

void byte(const blofeld::LcdByte data, const unsigned value, const unsigned known = byte_mask) {
  require(data.value == value && data.known == known, "Independent literal LCD output oracle");
}

blofeld::LcdCycle completed(const blofeld::LcdBusResult &result) {
  if (!result.completed) { throw blofeld::Fault("Expected one completed LCD bus cycle"); }
  return *result.completed;
}

void write(blofeld::St7565Parallel &bus, const blofeld::LcdCycle request) {
  const unsigned low = request.data ? writing : selected;
  bus.sample(pins(low, request.byte.value));
  const auto high = bus.sample(pins(low | pin::enable, request.byte.value));
  require(!high.completed && high.drive == 0, "A write high phase does not mutate the controller");
  const auto result = bus.sample(pins(low, request.byte.value));
  require(result.drive == 0 && result.completed && !result.completed->read &&
              result.completed->data == request.data,
          "A complete selected write has one command/data event");
  byte(completed(result).byte, request.byte.value);
}

void data(blofeld::St7565Parallel &bus, const unsigned value) {
  write(bus, {.data = true, .byte = {.value = static_cast<std::uint8_t>(value)}});
}

void command(blofeld::St7565Parallel &bus, const unsigned value) {
  write(bus, {.byte = {.value = static_cast<std::uint8_t>(value)}});
}

void test_write_boundaries() {
  blofeld::St7565Parallel bus;
  reset(bus);
  constexpr unsigned first = 0x11;
  constexpr unsigned final = 0x55;
  bus.sample(pins(writing, first));
  bus.sample(pins(writing | pin::enable, first));
  byte(bus.controller().ram(0, 0), 0, 0);
  bus.sample(pins(writing | pin::enable, final));
  byte(bus.controller().ram(0, 0), 0, 0);
  require(bus.controller().state().column == 0, "High phases cannot advance RAM addresses");
  const auto result = bus.sample(pins(writing, final));
  require(result.completed && result.completed->data && !result.completed->read,
          "Falling enable completes exactly one stable data write");
  byte(bus.controller().ram(0, 0), final);
  require(bus.controller().state().column == 1, "The completed native write advances once");
  require(!bus.sample(pins(writing, final)).completed,
          "Repeated low samples do not duplicate writes");
  command(bus, reset_command);
  byte(bus.controller().ram(0, 0), final);
  require(bus.controller().state().column == 0, "A command cycle reaches the real reset decoder");
}

void test_select_while_enable_high_and_release() {
  blofeld::St7565Parallel bus;
  reset(bus);
  constexpr unsigned first = 0x12;
  constexpr unsigned second = 0x34;
  data(bus, first);
  data(bus, second);
  command(bus, 16);
  command(bus, 0);
  const auto deselected = reading | pin::enable | pin::cs1_n;
  require(bus.sample(pins(deselected)).drive == 0,
          "The controller is high impedance while chip select is inactive, even with E high");
  const auto output = bus.sample(pins(reading | pin::enable));
  require(output.drive == byte_mask && !output.completed,
          "Asserting CS while E is already high enables combinational data output");
  byte(output.data, 0);
  for (unsigned index = 0; index < 8; ++index) {
    const auto stable = bus.sample(pins(reading | pin::enable));
    byte(stable.data, 0);
    require(!stable.completed && bus.controller().state().column == 0,
            "Repeated read observations never consume the dummy or advance a column");
  }
  const auto dummy = bus.sample(pins(reading));
  require(dummy.drive == 0 && dummy.completed && dummy.completed->read,
          "Ending the first read overlap releases data and completes the dummy read");
  byte(completed(dummy).byte, 0);
  require(bus.controller().state().column == 0, "Dummy completion preserves column zero");
  byte(bus.sample(pins(reading | pin::enable)).data, first);
  require(bus.controller().state().column == 0,
          "A valid read cannot advance the address before its overlap ends");
  byte(bus.sample(pins(reading | pin::enable)).data, first);
  require(bus.controller().state().column == 0,
          "Further high observations cannot consume the valid read early");
  const auto no_cs2 = (reading | pin::enable) & ~pin::cs2;
  const auto finished = bus.sample(pins(no_cs2));
  require(finished.drive == 0 && finished.completed && finished.completed->read,
          "Deasserting CS ends the qualified overlap while E stays high");
  byte(completed(finished).byte, first);
  require(bus.controller().state().column == 1, "Only the completed valid read increments");
  byte(bus.sample(pins(reading | pin::enable)).data, second);
  const auto next = bus.sample(pins(reading));
  byte(completed(next).byte, second);
  require(bus.controller().state().column == 2, "A new CS overlap can produce the next read");
}

void test_unknown_data_and_reset_abort() {
  blofeld::St7565Parallel bus;
  reset(bus);
  const auto unknown = bus.sample(pins(reading | pin::enable));
  require(unknown.drive == byte_mask, "An unknown read remains an active electrical output");
  byte(unknown.data, 0, 0);
  bus.sample(pins(reading));
  constexpr unsigned value = 0xa6;
  command(bus, reset_command);
  data(bus, value);
  command(bus, 0);
  byte(bus.sample(pins(reading | pin::enable)).data, 0);
  reset(bus);
  require(bus.controller().state().column == 0,
          "Reset aborts an unfinished read with no increment");
  byte(bus.controller().ram(0, 0), value);
  bus.sample(pins(writing | pin::enable, 0));
  reset(bus);
  byte(bus.controller().ram(0, 0), value);
  require(bus.sample(pins(idle)).drive == 0, "Reset also abandons an unfinished write");
}

void test_zero_address_hold_at_completion() {
  blofeld::St7565Parallel bus;
  reset(bus);
  constexpr unsigned value = 0xa5;
  bus.sample(pins(writing | pin::enable, value));
  // Figure 38 permits A0/R/W to change at the completed overlap (tAH6=0).
  const auto written = bus.sample(pins(idle, value));
  const auto write_cycle = completed(written);
  require(write_cycle.data && !write_cycle.read && written.drive == 0,
          "Boundary A0 changes retain the completed data write's old address selection");
  byte(bus.controller().ram(0, 0), value);
  command(bus, 0);
  byte(bus.sample(pins(reading | pin::enable)).data, 0);
  const auto dummy = bus.sample(pins(idle));
  require(completed(dummy).read && completed(dummy).data,
          "Boundary R/W changes complete the previous read instead of starting a write");
  byte(completed(dummy).byte, 0);
  require(bus.controller().state().column == 0, "Completed dummy preserves the selected column");
  byte(bus.sample(pins(reading | pin::enable)).data, value);
  require(bus.controller().state().column == 0, "Valid read remains unconsumed during high E");
  const auto selected_end = bus.sample(pins(idle | pin::enable));
  require(selected_end.drive == 0 && completed(selected_end).read,
          "CS deassertion can end a read with E high and new address/control levels");
  byte(completed(selected_end).byte, value);
  require(bus.controller().state().column == 1, "The boundary read advances exactly once");
  bus.sample(pins(reading | pin::enable));
  const auto unknown_after =
      bus.sample({.control = pin::reset_n | pin::cs1_n, .known = pin::reset_n | pin::cs1_n});
  require(unknown_after.drive == 0 && completed(unknown_after).read,
          "Address uncertainty after a proven end cannot invalidate the already sampled read");
  reject([&] { bus.sample(pins(selected | pin::enable | pin::read)); }, "status read");
}

void test_unknown_controls_and_status() {
  blofeld::St7565Parallel bus;
  reject([&] { bus.sample(pins(reading | pin::enable)); }, "before observed hardware reset");
  reset(bus);
  reject([&] { bus.sample({}); }, "Unknown ST7565 reset input");
  const auto inactive =
      bus.sample({.control = pin::reset_n | pin::cs1_n, .known = pin::reset_n | pin::cs1_n});
  require(inactive.drive == 0, "A known inactive CS suffices without guessing other controls");
  reject([&] { bus.sample({.control = pin::reset_n, .known = pin::reset_n}); }, "chip select");
  reject([&] { bus.sample({.control = selected, .known = pin::reset_n | pin::cs1_n | pin::cs2}); },
         "enable input");
  reject(
      [&] {
        bus.sample({.control = selected | pin::enable,
                    .known = pin::reset_n | pin::cs1_n | pin::cs2 | pin::enable});
      },
      "R/W or A0");
  reject([&] { bus.sample(pins(selected | pin::enable | pin::read)); }, "status read");
  reject([&] { bus.sample(pins(reading | pin::enable, 0)); }, "host drives data during read");
  auto invalid = pins(idle);
  invalid.data.known = 1;
  reject([&] { bus.sample(invalid); }, "pin/known/drive masks");
  invalid = pins(idle);
  invalid.control |= pin::all + 1U;
  reject([&] { bus.sample(invalid); }, "pin/known/drive masks");
}

void test_unstable_and_partial_write() {
  blofeld::St7565Parallel bus;
  reset(bus);
  constexpr unsigned old_value = 0x55;
  constexpr unsigned new_value = 0xaa;
  bus.sample(pins(writing | pin::enable, old_value));
  reject([&] { bus.sample(pins(writing, new_value)); }, "not stable and fully driven");
  byte(bus.controller().ram(0, 0), 0, 0);
  reset(bus);
  auto partial = pins(writing | pin::enable, new_value);
  constexpr unsigned partial_mask = 0xfe;
  partial.data.known = partial_mask;
  bus.sample(partial);
  partial.control &= ~pin::enable;
  reject([&] { bus.sample(partial); }, "not stable and fully driven");
  byte(bus.controller().ram(0, 0), 0, 0);
  reset(bus);
  bus.sample(pins(writing | pin::enable, old_value));
  reject([&] { bus.sample(pins(selected | pin::enable, old_value)); }, "R/W or A0 changed");
  byte(bus.controller().ram(0, 0), 0, 0);
  reset(bus);
  bus.sample(pins(reading | pin::enable));
  reject([&] { bus.sample(pins(reading | pin::enable, old_value)); },
         "host drives data during read");
  require(bus.controller().state().column == 0, "A rejected read drive cannot consume the frame");
  reset(bus);
  data(bus, old_value);
  byte(bus.controller().ram(0, 0), old_value);
}
}

int main() {
  try {
    test_write_boundaries();
    test_select_while_enable_high_and_release();
    test_unknown_data_and_reset_abort();
    test_zero_address_hold_at_completion();
    test_unknown_controls_and_status();
    test_unstable_and_partial_write();
    std::cout << "ST7565 parallel overlap, read drive/completion, reset and unknown/fault checks "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
