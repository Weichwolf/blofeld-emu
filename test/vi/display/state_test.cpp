// SPDX-License-Identifier: GPL-3.0-only
#include "display/st7565.hpp"
#include "display/state.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
constexpr std::uint8_t display_on = 0xaf, volume_command = 0x81, column_low = 4;
constexpr std::uint8_t column_high = 0x10, all_bits = 0xff, volume_mask = 63;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void write(blofeld::vi::St7565 &lcd, const std::uint8_t value) {
  lcd.command(column_high);
  lcd.command(column_low);
  lcd.write_data(value);
  lcd.command(volume_command);
  lcd.command(value & volume_mask);
}

void snapshots() {
  blofeld::vi::DisplayState channel;
  blofeld::vi::St7565 lcd;
  lcd.hardware_reset();
  lcd.command(display_on);
  require(!channel.take(), "There is no invented controller snapshot");
  constexpr std::uint8_t first = 0xa5;
  constexpr std::uint8_t second = 0x5a;
  write(lcd, first);
  channel.capture(lcd);
  write(lcd, second);
  const auto captured = channel.take();
  require(captured && captured->ram(0, column_low).value == first &&
              captured->ram(0, column_low).known == all_bits &&
              captured->ram(0, column_low + 1).known == 0 &&
              captured->state().volume == (first & volume_mask),
          "A requested snapshot owns its complete actual state and unknown cells");
  channel.capture(lcd);
  require(!channel.take(), "No new request performs a new capture");
  channel.request();
  channel.request();
  channel.capture(lcd);
  const auto next = channel.take();
  require(next && next->ram(0, column_low).value == second && !channel.take(),
          "Repeated requests coalesce; a completed snapshot is taken only once");
}

void concurrent_snapshots() {
  blofeld::vi::DisplayState channel;
  std::atomic<bool> done{false};
  constexpr unsigned passes = 4096;
  std::jthread owner([&] {
    blofeld::vi::St7565 lcd;
    lcd.hardware_reset();
    lcd.command(display_on);
    for (unsigned pass = 0; pass < passes; ++pass) {
      write(lcd, static_cast<std::uint8_t>(pass & all_bits));
      channel.capture(lcd);
    }
    channel.request();
    channel.capture(lcd);
    done.store(true, std::memory_order_release);
  });
  unsigned observed = 0;
  std::uint8_t last = 0;
  bool coherent = true;
  const auto take = [&] {
    if (const auto value = channel.take()) {
      const auto byte = value->ram(0, column_low);
      coherent = coherent && byte.known == all_bits && value->state().display_on &&
                 value->state().volume == (byte.value & volume_mask);
      last = byte.value;
      ++observed;
    }
  };
  while (!done.load(std::memory_order_acquire)) {
    channel.request();
    take();
    std::this_thread::yield();
  }
  owner.join();
  take();
  require(coherent && observed != 0 && last == all_bits,
          "Concurrent captures/takes retain coherent RAM/registers and the final actual state");
}
}

int main() try {
  snapshots();
  concurrent_snapshots();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
