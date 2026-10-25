// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"

namespace blofeld {
NandArray::NandArray(const NandImage &image) {
  image.validate();
  contents_ = image.pages;
  blocks_ = image.blocks;
}

NandImage NandArray::image() const {
  NandImage result{.pages = contents_, .blocks = blocks_};
  result.validate();
  return result;
}

Nand::Nand(const NandImage &image) : contents_(image) {}

NandImage Nand::persistent_image() const {
  if (phase_ != Phase::cold) {
    fail("persistent export", "explicit power-off projection required");
  }
  return contents_.image();
}
}
