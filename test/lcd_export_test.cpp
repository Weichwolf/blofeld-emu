// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/lcd_export.hpp"
#include "display/lcd_image.hpp"
#include "display/st7565.hpp"
#include "lcd_image_fixture.hpp"
#include "storage/private_file.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace {
using blofeld::PrivateFilePhase;
using blofeld::test::reject;
using blofeld::test::require;
using blofeld::test::select;
constexpr unsigned maximum_bytes = 16384;

void check_files(const blofeld::PrivateDirectory &directory, const blofeld::LcdImage &image) {
  require(directory.read("display.pbm", maximum_bytes) == encode_lcd_pbm(image),
          "Published PBM is a complete expected frame");
  const auto bytes = directory.read("display.txt", maximum_bytes);
  require(std::string(bytes.begin(), bytes.end()) == encode_lcd_text(image),
          "Published text is a complete expected frame");
}

void test_failures(const std::filesystem::path &path) {
  require(std::filesystem::create_directory(path), "Create isolated publisher test directory");
  const blofeld::PrivateDirectory directory(path);
  auto lcd = blofeld::test::blank();
  const auto first = capture_lcd(lcd, {.width = 128});
  lcd.command(blofeld::test::lcd_reverse_on);
  const auto second = capture_lcd(lcd, {.width = 128});
  blofeld::LcdPublisher publisher(path);
  require(publisher.publish(first), "Initial complete frame is published");
  reject([&] { const blofeld::LcdPublisher competing(path); }, "writer lease");
  unsigned observations = 0;
  const bool changed = publisher.publish(first, [&](PrivateFilePhase) { ++observations; });
  require(!changed && observations == 0, "Unchanged pixels cause no filesystem operation");
  for (const auto failure :
       {PrivateFilePhase::created, PrivateFilePhase::written, PrivateFilePhase::synced}) {
    reject(
        [&] {
          publisher.publish(second, [&](const PrivateFilePhase phase) {
            if (phase == failure) { throw blofeld::Fault("Injected host publication failure"); }
          });
        },
        "Injected host publication failure");
    check_files(directory, first);
    require(publisher.publish(first), "A failed publication invalidates the cache for safe repair");
  }
  reject(
      [&] {
        publisher.publish(second, [](const PrivateFilePhase phase) {
          if (phase == PrivateFilePhase::published) {
            throw blofeld::Fault("Injected post-rename failure");
          }
        });
      },
      "Injected post-rename failure");
  require(directory.read("display.pbm", maximum_bytes) == encode_lcd_pbm(second),
          "Post-rename failure exposes a complete new PBM, never a partial frame");
  const auto text = directory.read("display.txt", maximum_bytes);
  require(std::string(text.begin(), text.end()) == encode_lcd_text(first),
          "PBM/text are explicitly separate publications");
  require(publisher.publish(first), "Returning to the old pixels repairs a partial new prefix");
  check_files(directory, first);
  reject(
      [&] {
        publisher.publish(second, [](const PrivateFilePhase phase) {
          if (phase == PrivateFilePhase::published) {
            throw blofeld::Fault("Injected retry failure");
          }
        });
      },
      "Injected retry failure");
  require(publisher.publish(second), "Retry after a partial publication must not be suppressed");
  check_files(directory, second);
  blofeld::St7565 unknown;
  unknown.hardware_reset();
  unknown.command(blofeld::test::lcd_display_on);
  reject([&] { publisher.publish(capture_lcd(unknown, {.width = 128})); },
         "Unknown LCD screenshot");
  check_files(directory, second);
  for (const auto &entry : std::filesystem::directory_iterator(path)) {
    require(!entry.path().filename().string().contains(".tmp."),
            "Failed temporary writes are removed");
  }
}

void stream(const std::filesystem::path &path) {
  auto lcd = blofeld::test::blank();
  const auto first = capture_lcd(lcd, {.width = 128});
  lcd.command(blofeld::test::lcd_reverse_on);
  const auto second = capture_lcd(lcd, {.width = 128});
  blofeld::LcdPublisher publisher(path);
  require(publisher.publish(first), "Publish initial reader fixture");
  const blofeld::PrivateDirectory directory(path);
  directory.write("black.png", encode_lcd_png(second));
  auto pattern = blofeld::test::blank();
  select(pattern, {.page = 0, .column = 0});
  pattern.write_data(1);
  select(pattern, {.page = 0, .column = 8});
  pattern.write_data(1);
  select(pattern, {.page = 7, .column = 7});
  constexpr unsigned top_bit = 0x80;
  pattern.write_data(top_bit);
  directory.write("pattern.png", encode_lcd_png(capture_lcd(pattern, {.width = 9})));
  std::cout << "ready\n" << std::flush;
  std::string command;
  while (std::getline(std::cin, command)) {
    if (command == "stop") { return; }
    if (command == "same") {
      require(!publisher.publish(first), "Unchanged publication is suppressed");
    } else if (command == "burst") {
      constexpr unsigned frames = 64;
      for (unsigned index = 0; index < frames; ++index) {
        require(publisher.publish((index & 1U) == 0 ? second : first), "Publish alternating frame");
      }
    } else {
      throw blofeld::Fault("Unknown internal publisher test command");
    }
    std::cout << "done\n" << std::flush;
  }
  throw blofeld::Fault("Publisher test input closed without its stop command");
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 2, "Usage: lcd-export-test private-directory");
    const std::filesystem::path directory(arguments[1]);
    test_failures(directory / "failures");
    stream(directory);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
