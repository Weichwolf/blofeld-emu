// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_midi.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

namespace {
constexpr unsigned bus_hz = 30000000;
constexpr unsigned bit_cycles = 960;
constexpr unsigned byte_cycles = 9600;
constexpr unsigned byte_count = 256;
constexpr unsigned rxd = 4;
constexpr unsigned unmasked = 0xc0;
constexpr unsigned expected_status = 0xe0;
constexpr unsigned baud_divisor = 60;
constexpr unsigned receive_and_interrupt = 0x24;
constexpr unsigned irq_vector = 0xfff2;
constexpr blofeld::ExecutionContext context{.pc = 0x8123, .ccr = unmasked};

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string rejects(const auto &operation, const char *message) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault(message);
}

std::unique_ptr<blofeld::McuBus> bus() {
  constexpr unsigned flash_size = 32768;
  const std::array<std::uint8_t, flash_size> flash{};
  auto result = std::make_unique<blofeld::McuBus>(flash);
  result->set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  return result;
}

void configure(blofeld::McuBus &board, const bool small = false) {
  board.write(blofeld::hc12::sci_baud_high, 0);
  board.write(blofeld::hc12::sci_baud_low, baud_divisor);
  board.write(blofeld::hc12::sci_control1, 0);
  board.write(blofeld::hc12::modrr, small ? 1 : 0);
  board.write(blofeld::hc12::sci_control2, receive_and_interrupt);
}

void check_received(blofeld::McuBus &board, const unsigned byte) {
  const auto request = board.interrupt_request(unmasked);
  if (!request) {
    throw blofeld::Fault("Actual SCI receive source did not request its native vector");
  }
  constexpr unsigned sci_vector = 0xffd6;
  require(request->vector == sci_vector &&
              board.read(blofeld::hc12::sci_status1) == expected_status &&
              board.read(blofeld::hc12::sci_data_low) == byte && !board.interrupt_request(unmasked),
          "Serialized MIDI must reach real SCI storage/flags and acknowledge through the "
          "documented read sequence");
}

void test_all_bytes_and_pin_routes() {
  for (const bool small : {false, true}) {
    auto board = bus();
    configure(*board, small);
    const auto port = small ? blofeld::McuPort::s : blofeld::McuPort::r;
    board->set_pin_drive(port, {.mask = 1, .levels = 1});
    blofeld::McuMidiInput input(bus_hz,
                                small ? blofeld::SciReceivePin::ps4 : blofeld::SciReceivePin::pr2);
    input.advance_sci(*board, context, bit_cycles);
    for (unsigned value = 0; value < byte_count; ++value) {
      const std::array<std::uint8_t, 1> bytes{static_cast<std::uint8_t>(value)};
      input.enqueue(bytes);
      input.advance_sci(*board, context, byte_cycles);
      check_received(*board, value);
      const auto neighbor = board->observe_pads(port, 1);
      require(neighbor.known == 1 && neighbor.levels == 1 && input.queued_bytes() == 0,
              "RX transitions must preserve unrelated external observations and consume exactly "
              "one byte");
    }
  }
}

void test_partial_frame_checkpoint_and_fractional_clock() {
  for (const bool fractional : {false, true}) {
    auto board = bus();
    configure(*board);
    constexpr unsigned fractional_clock = 30000001;
    const auto frequency = fractional ? fractional_clock : bus_hz;
    blofeld::McuMidiInput input(frequency, blofeld::SciReceivePin::pr2);
    input.advance_sci(*board, context, bit_cycles + 1);
    constexpr std::array<std::uint8_t, 1> bytes{0xa5};
    input.enqueue(bytes);
    constexpr unsigned partial = (4 * bit_cycles) + 17;
    input.advance_sci(*board, context, partial);
    const auto frozen_bus = board->clone();
    const auto frozen_input = input;
    const auto total = fractional ? byte_cycles + 1 : byte_cycles;
    input.advance_sci(*board, context, total - partial);
    check_received(*board, bytes.front());
    auto replay = frozen_bus->clone();
    auto restored = frozen_input;
    constexpr unsigned fragment = 19;
    unsigned remaining = total - partial;
    while (remaining != 0) {
      const auto count = remaining < fragment ? remaining : fragment;
      restored.advance_sci(*replay, context, count);
      remaining -= count;
    }
    check_received(*replay, bytes.front());
    require(
        input.queued_bytes() == 0 && restored.queued_bytes() == 0,
        "SCI and source snapshots must retain partial RX bits and rational source phase together");
  }
}

void test_partial_pin_update_validation() {
  blofeld::McuGpio gpio;
  gpio.set_drive(blofeld::McuPort::r, {.mask = 3, .levels = 1, .floating = 8});
  gpio.update_drive(blofeld::McuPort::r, rxd, {.mask = rxd, .levels = rxd});
  const auto before = gpio.observe_pads(blofeld::McuPort::r, 15);
  require(
      before.known == 7 && before.levels == 5,
      "Partial pin update must preserve neighboring high/low drives and unknown floating input");
  static_cast<void>(rejects(
      [&gpio] { gpio.update_drive(blofeld::McuPort::r, rxd, {.mask = rxd, .levels = 1}); },
      "Levels outside the new drive mask must reject before merging with a known neighbor"));
  const auto after = gpio.observe_pads(blofeld::McuPort::r, 15);
  require(before.known == after.known && before.levels == after.levels,
          "Invalid partial update must leave every external drive unchanged");
  gpio.update_drive(blofeld::McuPort::r, rxd, {});
  require(gpio.observe_pads(blofeld::McuPort::r, 15).known == 3,
          "Selected drive can become unknown without losing its neighbors");
  auto board = bus();
  board->set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  board->update_pin_drive(blofeld::McuPort::e, 2, {.mask = 2});
  const auto interrupt = board->interrupt_request(unmasked);
  require(interrupt && interrupt->vector == irq_vector &&
              board->observe_pads(blofeld::McuPort::e, 3).levels == 1,
          "Partial E1 observation must request IRQ without changing the independent E0 input");
}

void test_visible_failure_and_coordinated_restore() {
  auto board = bus();
  configure(*board, true);
  blofeld::McuMidiInput input(bus_hz, blofeld::SciReceivePin::pr2);
  const auto frozen_bus = board->clone();
  const auto frozen_input = input;
  const auto error =
      rejects([&board, &input] { input.advance_sci(*board, context, bit_cycles); },
              "A disconnected active PS4 receiver must not get a fabricated idle level");
  require(input.faulted() && error.contains("pc=0x8123") && error.contains("ccr=0xc0") &&
              error.contains("0x0270") && error.contains("Unknown MCU GPIO"),
          "Wrong explicit connection must retain receive operation, actual missing address and CPU "
          "context");
  static_cast<void>(rejects([&board, &input] { input.advance_sci(*board, context, 1); },
                            "Rejected transport must not resume a partially clocked receiver"));
  constexpr std::array<std::uint8_t, 1> bytes{0x90};
  static_cast<void>(rejects([&input, &bytes] { input.enqueue(bytes); },
                            "Rejected transport cannot silently accept more input"));
  board = frozen_bus->clone();
  input = frozen_input;
  board->write(blofeld::hc12::modrr, 0);
  input.advance_sci(*board, context, bit_cycles);
  input.enqueue(bytes);
  input.advance_sci(*board, context, byte_cycles);
  check_received(*board, bytes.front());
  require(!input.faulted(), "Only an explicitly restored healthy source and receiver may continue");
}
}

int main() {
  try {
    test_all_bytes_and_pin_routes();
    test_partial_frame_checkpoint_and_fractional_clock();
    test_partial_pin_update_validation();
    test_visible_failure_and_coordinated_restore();
    std::cout << "MIDI source to native SCI, pin routes, partial replay, isolation and failure "
                 "context passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
