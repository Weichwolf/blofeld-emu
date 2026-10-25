// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "parallel_update_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <span>

namespace {
constexpr unsigned caller = 0x1700;
// Leave legal RAM above SP for the own caller's stack-passed byte arguments.
constexpr unsigned stack = 0x25e0;
constexpr unsigned masked = 0xd0;
constexpr unsigned byte_mask = 255;
constexpr unsigned display_flags = 0x1815;
constexpr unsigned display_state_word = 0x1814;
constexpr unsigned transition_state = 0x184a;
constexpr unsigned buffered_display = 0x20;
constexpr unsigned clear_display = 0x8d70;
constexpr unsigned write_pixel = 0x8d9f;
constexpr unsigned flush_display = 0xbfbb;
constexpr unsigned ready_poll = 0xb98d;
constexpr unsigned initialization = 0x8b94;
constexpr unsigned external_u_mask = 0x3f;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

struct Call {
  unsigned entry;
  unsigned value = 0;
};

void invoke(blofeld::McuBus &bus, blofeld::Hc12 &cpu, const Call call, std::ostream &trace) {
  // Own RAM caller; all callees and their nested instructions remain original.
  constexpr unsigned jsr = 0x16;
  bus.write(caller, jsr);
  bus.write(caller + 1, static_cast<std::uint8_t>(call.entry >> 8U));
  bus.write(caller + 2, static_cast<std::uint8_t>(call.entry & byte_mask));
  cpu.set_state(
      {.pc = caller, .d = static_cast<std::uint16_t>(call.value), .sp = stack, .ccr = masked});
  unsigned instructions = 0;
  constexpr unsigned limit = 2000000;
  while (cpu.state().pc != caller + 3) {
    require(instructions < limit, "Original LCD routine must return within its instruction bound");
    if (cpu.state().pc == ready_poll) {
      bus.set_pin_drive(blofeld::McuPort::u, {.mask = external_u_mask, .levels = 1});
    }
    if (cpu.state().pc == initialization) { bus.set_pin_drive(blofeld::McuPort::b, {.mask = 8}); }
    const auto result = cpu.step();
    require(result.instruction_executed && !result.interrupt_vector,
            "Isolated native LCD call has no supplied interrupt or peripheral return stub");
    ++instructions;
  }
  require(cpu.state().sp == stack, "Native LCD call restores its own real return frame");
  trace << "call entry=" << blofeld::hex(call.entry) << " instructions=" << instructions << '\n';
}

struct Point {
  unsigned x;
  unsigned y;
};

void pixel(blofeld::McuBus &bus, blofeld::Hc12 &cpu, const Point point, std::ostream &trace) {
  // At native callee entry: y at SP+2, x at SP+3, operation in B.
  bus.write(stack, static_cast<std::uint8_t>(point.y));
  bus.write(stack + 1, static_cast<std::uint8_t>(point.x));
  invoke(bus, cpu, {.entry = write_pixel}, trace);
}

blofeld::LcdImage capture(const blofeld::test::LcdFixture &display,
                          const std::span<const Point> expected) {
  // Logical UI orientation recovered from the bounded native pixel routines;
  // this is explicitly not measured physical panel wiring.
  const auto image = capture_lcd(display.controller(), {.width = 128, .reverse_commons = true});
  for (unsigned y = 0; y < blofeld::LcdImage::height; ++y) {
    for (unsigned x = 0; x < image.width(); ++x) {
      const bool on = std::ranges::any_of(
          expected, [&](const Point point) { return point.x == x && point.y == y; });
      require(image.pixel(x, y) == (on ? blofeld::LcdPixel::on : blofeld::LcdPixel::off),
              "Native LCD drawing must match independently specified logical pixels");
    }
  }
  return image;
}

void buffered(blofeld::McuBus &bus,
              blofeld::Hc12 &cpu,
              const blofeld::test::LcdFixture &display,
              const blofeld::LcdImage &direct,
              std::ostream &trace) {
  const auto before = display.counts();
  bus.write(display_flags, buffered_display);
  invoke(bus, cpu, {.entry = clear_display}, trace);
  constexpr Point point{.x = 3, .y = 14};
  pixel(bus, cpu, point, trace);
  const auto after = display.counts();
  require(before.commands == after.commands && before.reads == after.reads &&
              before.writes == after.writes &&
              capture_lcd(display.controller(), {.width = 128, .reverse_commons = true}) == direct,
          "Native buffered clear/drawing never reaches the LCD or changes a captured frame");
  bus.write(transition_state, 0); // Explicit full-frame branch; transition timing is unqualified.
  bus.write(display_state_word, byte_mask);
  invoke(bus, cpu, {.entry = flush_display}, trace);
  require(display.counts().writes - after.writes == 1024 && display.counts().reads == after.reads,
          "Native full-frame flush sends all 1024 bytes through physical GPIO writes");
  static_cast<void>(capture(display, std::array{point}));
  require(bus.read(display_state_word) == 0 && bus.read(display_flags) == buffered_display,
          "Flush retains the original word-wide state clear and saved display flags");
}

struct Inputs {
  std::filesystem::path reference;
  std::filesystem::path trace;
};

void run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.reference);
  blofeld::McuBus bus(firmware.mcu);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  blofeld::test::boot(bus, cpu, trace);
  blofeld::test::LcdFixture display(bus);
  constexpr unsigned e_mask = 0xeb;
  constexpr unsigned serial_inputs = 0xe8;
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = e_mask, .levels = 3});
  bus.set_pin_drive(blofeld::McuPort::b, {.mask = serial_inputs});
  bus.set_observer([&display](const blofeld::BusEvent &event) { display.observe(event); });
  constexpr std::array<unsigned, 3> setup{0xb938, 0xb955, initialization};
  for (const auto entry : setup) { invoke(bus, cpu, {.entry = entry}, trace); }
  bus.write(display_flags, 0); // Explicit direct-I/O experiment, not a full application boot.
  invoke(bus, cpu, {.entry = clear_display}, trace);
  static_cast<void>(capture(display, {}));
  constexpr std::array<Point, 4> points{
      {{.x = 0, .y = 0}, {.x = 127, .y = 63}, {.x = 9, .y = 26}, {.x = 9, .y = 29}}};
  for (const auto point : points) { pixel(bus, cpu, point, trace); }
  const auto revision = bus.revision();
  const auto direct = capture(display, points);
  require(bus.revision() == revision, "Capturing actual native LCD RAM causes no MCU bus mutation");
  auto bitmap = inputs.trace;
  bitmap += ".pbm";
  blofeld::write_private_file(bitmap, encode_lcd_pbm(direct));
  auto screenshot = inputs.trace;
  screenshot += ".png";
  blofeld::write_private_file(screenshot, encode_lcd_png(direct));
  buffered(bus, cpu, display, direct, trace);
  const auto image = bus.power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Native display drawing/flush and host captures preserve all MCU NVM/history");
  bus.set_observer({});
  std::cout << "Original LCD: native pixel orientation, buffered visibility, 1024-byte GPIO flush "
               "and read-only PBM/PNG capture passed. Conditional GPIO/entry/ready fixture; "
               "no hardware timing, MIDI-driven UI or full board boot claim.\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-lcd-image-test firmware.zip private.trace");
    run({.reference = arguments[1], .trace = arguments[2]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
