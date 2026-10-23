// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "blofeld/lcd_image.hpp"
#include "blofeld/private_file.hpp"
#include "keyboard_initial_fixture.hpp"
#include "lcd_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_initial;

struct Expectations {
  transport::Templates download;
  unsigned branch = 0;
};

Expectations expectations(const char *path) {
  std::ifstream input(path);
  input >> std::hex;
  Expectations result;
  for (auto &word : result.download) { word = transport::number(input, gp::word_mask); }
  result.branch = transport::number(input, gp::word_mask);
  input >> std::ws;
  require(input.eof(), "Unexpected independent initial-sender expectations");
  return result;
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 5, "Expected original main firmware, Keyboard packets, templates and trace");
    const auto firmware = Firmware::load(arguments[1]);
    const auto blocks = transport::read_blocks(arguments[2], layouts);
    const auto own = expectations(arguments[3]);
    std::ofstream trace(arguments[4]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, gp::phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        Fixture run(firmware, blocks, own.download, own.branch, trace, phase);
        run.execute(order, phase);
        run.replay();
        run.retain();
        const auto image =
            capture_lcd(run.display->controller(), {.width = 128, .reverse_commons = true});
        const auto base =
            std::string(arguments[4]) + "." + std::to_string(order) + "." + std::to_string(phase);
        write_private_file(base + ".pbm", encode_lcd_pbm(image));
        write_private_file(base + ".png", encode_lcd_png(image));
      }
    }
    std::cout << "Original initial Keyboard sender and display-helper observations completed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
