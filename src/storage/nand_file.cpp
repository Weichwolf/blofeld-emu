// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_file.hpp"

#include "common/bus.hpp"
#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"

#include <filesystem>

namespace blofeld {
namespace {
void extension(const std::filesystem::path &path) {
  if (path.extension() != ".bfnand") {
    throw Fault("NAND state files require the private .bfnand extension");
  }
}
}

NandImage load_nand_image(const std::filesystem::path &path) {
  extension(path);
  return decode_nand_image(read_private_file(path, nand_image_max_bytes));
}

void save_nand_image(const std::filesystem::path &path,
                     const NandImage &image,
                     const NandFileObserver &observer) {
  extension(path);
  const PrivateDirectory directory(path.parent_path().empty() ? std::filesystem::path(".")
                                                              : path.parent_path());
  const PrivateFileLease lease(directory, path.filename().string() + ".lock");
  directory.write(path.filename(), encode_nand_image(image), observer);
}
}
