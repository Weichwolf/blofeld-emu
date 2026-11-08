// SPDX-License-Identifier: GPL-3.0-only
#include "display/lcd_image.hpp"
#include "display/publication.hpp"
#include "display/st7565.hpp"

#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  blofeld::vi::St7565 lcd;
  lcd.hardware_reset();
  constexpr blofeld::vi::LcdPanel panel{.width = 128, .first_segment = 4};
  const auto blank = capture_lcd(lcd, panel);
  constexpr std::size_t frame_bytes = 1034;
  const auto bitmap = blofeld::app::encode_pbm(blank);
  require(bitmap.size() == frame_bytes && bitmap[0] == 'P' && bitmap[1] == '4' &&
              bitmap.back() == 0,
          "Exact complete 128x64 binary PBM");
  constexpr unsigned display_on = 0xaf;
  lcd.command(display_on);
  const auto unknown = capture_lcd(lcd, panel);
  require(blofeld::app::encode_text(unknown).front() == '?', "Text preserves unknown pixels");
  bool rejected = false;
  try {
    static_cast<void>(blofeld::app::encode_pbm(unknown));
  } catch (const std::runtime_error &) { rejected = true; }
  require(rejected, "Unknown pixels must not become a fake PBM");
  constexpr unsigned all_points = 0xa5;
  lcd.command(all_points);
  const auto full = capture_lcd(lcd, panel);
  constexpr unsigned full_byte = 0xff;
  require(blofeld::app::encode_pbm(full).back() == full_byte, "All-points PBM");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned publication directory");
  {
    blofeld::app::DisplayPublication publication(root);
    require(publication.publish(blank) && !publication.publish(blank), "Unchanged frame skips I/O");
    require(publication.publish(full), "Changed frame is published");
    std::ifstream text(root / "display.txt");
    const std::string data{std::istreambuf_iterator<char>(text), std::istreambuf_iterator<char>()};
    require(data.front() == '#' && data.back() == '\n', "Complete text publication");
  }
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << "\n";
  return 1;
}
