// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand_image.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"

namespace blofeld {
namespace {
void validate_page(const unsigned index, const NandArray::Page &page) {
  if (index >= NandArray::pages) {
    throw Fault("NAND image page exceeds geometry: page=" + hex(index, 6));
  }
  if (page.programs && (page.programs->main > 1 || page.programs->spare > 2)) {
    throw Fault("Invalid NAND image program history: page=" + hex(index, 6));
  }
  for (unsigned column = 0; column < NandArray::page_bytes; ++column) {
    const auto cell = page.cells.at(column);
    if ((cell.value & ~cell.known) != 0) {
      throw Fault("Noncanonical NAND image cell: page=" + hex(index, 6) +
                  " column=" + hex(column, 3));
    }
    if (!page.programs) { continue; }
    const auto count = column < NandArray::main_bytes ? page.programs->main : page.programs->spare;
    if (count == 0 && (cell.known & ~cell.value) != 0) {
      throw Fault("NAND image fresh history conflicts with programmed bits: page=" + hex(index, 6) +
                  " column=" + hex(column, 3));
    }
  }
}
}

void NandImage::validate() const {
  if (pages.size() > NandArray::pages || blocks.size() > NandArray::blocks) {
    throw Fault("NAND image record count exceeds geometry");
  }
  for (const auto &[index, page] : pages) { validate_page(index, page); }
  constexpr unsigned protected_blocks = 200;
  for (const auto &[index, block] : blocks) {
    if (index >= NandArray::blocks) {
      throw Fault("NAND image block exceeds geometry: block=" + hex(index, 4));
    }
    if (index >= protected_blocks &&
        (block.program_protected == true || block.erase_protected == true)) {
      throw Fault("NAND image protection exceeds supported blocks: block=" + hex(index, 4));
    }
  }
}
}
