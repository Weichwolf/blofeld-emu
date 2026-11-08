// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_image.hpp"

namespace blofeld {
Nand::Nand(const NandImage &image) : contents_(image) {}

Nand::Nand(const NandImage &image, const NandReadContinuation continuation) : Nand(continuation) {
  contents_ = NandArray(image);
}

NandImage Nand::persistent_image() const {
  if (phase_ != Phase::cold) {
    fail("persistent export", "explicit power-off projection required");
  }
  return contents_.image();
}

NandImage Nand::persistent_image(const NandRegion region) const {
  if (phase_ != Phase::cold) {
    fail("persistent export", "explicit power-off projection required");
  }
  return contents_.image(region);
}
}
