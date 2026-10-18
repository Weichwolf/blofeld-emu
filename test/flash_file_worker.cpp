// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/flash_file.hpp"
#include "blofeld/flash_image.hpp"

#include <array>
#include <charconv>
#include <cstdint>
#include <exception>
#include <iostream>
#include <ostream>
#include <string_view>
#include <system_error>

namespace {
unsigned number(const std::string_view input) {
  unsigned value = 0;
  const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
  if (result.ec != std::errc{} || result.ptr != input.data() + input.size()) {
    throw blofeld::Fault("Invalid worker number");
  }
  return value;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) { throw blofeld::Fault("Usage: nvm-file-worker output.bfnvm pattern phase"); }
    const auto pattern = number(arguments[2]);
    const auto requested = number(arguments[3]);
    constexpr unsigned byte_mask = 0xff;
    if (pattern > byte_mask ||
        requested > static_cast<unsigned>(blofeld::FlashFilePhase::directory_synced)) {
      throw blofeld::Fault("Invalid worker pattern or phase");
    }
    std::array<std::uint8_t, blofeld::McuFlashImage::byte_count> bytes{};
    bytes.fill(static_cast<std::uint8_t>(pattern));
    const auto image = blofeld::McuFlashImage::from_known(bytes);
    blofeld::save_flash_image(
        arguments[1], image, [requested](const blofeld::FlashFilePhase phase) {
          if (static_cast<unsigned>(phase) != requested) { return; }
          std::cout << requested << '\n' << std::flush;
          char resume = 0;
          if (!std::cin.get(resume)) { throw blofeld::Fault("Worker transaction was not resumed"); }
        });
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
