// SPDX-License-Identifier: GPL-3.0-only
#include "app/display/display_pump.hpp"

#include "common/bus.hpp"
#include "vi/display/lcd_image.hpp"
#include "vi/vi.hpp"

#include <chrono>
#include <filesystem>

namespace blofeld::app {
namespace {
constexpr auto frame_interval = std::chrono::milliseconds{20};
}

DisplayPump::DisplayPump(const std::filesystem::path &directory) : publisher_(directory) {}

void DisplayPump::poll(const vi::VirtualInstrument &instrument) {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_) { return; }
  next_ = now + frame_interval;
  capture(instrument);
}

void DisplayPump::capture(const vi::VirtualInstrument &instrument) {
  constexpr LcdPanel panel{.width = 128, .reverse_commons = true};
  const auto image = instrument.display_image(panel);
  if (!image) { throw Fault("Host display publication requires an attached LCD"); }
  for (unsigned y = 0; y < LcdImage::height; ++y) {
    for (unsigned x = 0; x < image->width(); ++x) {
      if (image->pixel(x, y) == LcdPixel::unknown) { return; }
    }
  }
  publisher_.publish(*image);
}
}
