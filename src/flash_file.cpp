// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/flash_file.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/private_file.hpp"

#include <filesystem>
#include <string>

namespace blofeld {
namespace {
void extension(const std::filesystem::path &path) {
  if (path.extension() != ".bfnvm") {
    throw Fault("MCU NVM files require the private .bfnvm extension");
  }
}
}

McuFlashImage load_flash_image(const std::filesystem::path &path) {
  extension(path);
  return decode_flash_image(read_private_file(path, flash_image_file_bytes));
}

void save_flash_image(const std::filesystem::path &path,
                      const McuFlashImage &image,
                      const FlashFileObserver &observer) {
  extension(path);
  const auto bytes = encode_flash_image(image);
  try {
    write_private_file(path, bytes, observer);
  } catch (const Fault &error) {
    throw Fault(std::string("Cannot publish MCU NVM file: ") + path.string() + ": " + error.what());
  }
}
}
