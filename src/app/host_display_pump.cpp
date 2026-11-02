// SPDX-License-Identifier: GPL-3.0-only
#include "app/host_display_pump.hpp"

#include "board/reference_board.hpp"
#include "common/bus.hpp"
#include "display/lcd_image.hpp"

#include <chrono>
#include <filesystem>

namespace blofeld::reference {
namespace {
constexpr auto frame_interval = std::chrono::milliseconds{20};
}

HostDisplayPump::HostDisplayPump(const std::filesystem::path &directory) : publisher_(directory) {}

void HostDisplayPump::poll(const Board &board) {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_) { return; }
  next_ = now + frame_interval;
  capture(board);
}

void HostDisplayPump::capture(const Board &board) {
  constexpr LcdPanel panel{.width = 128, .reverse_commons = true};
  const auto image = board.display_image(panel);
  if (!image) { throw Fault("Host display publication requires an attached LCD"); }
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned x = 0; x < image->width(); ++x) {
      if (image->pixel(x, y) == LcdPixel::unknown) { return; }
    }
  }
  publisher_.publish(*image);
}
}
