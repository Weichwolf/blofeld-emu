// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/nand_image.hpp"
#include "storage/private_file.hpp"

#include <cstdint>

namespace blofeld {
struct ImageJournalPosition {
  std::uint64_t offset = 0;
  std::uint64_t sequence = 0;
};

// A version-1 image followed by independently checksummed, committed page updates.
// Only a trailing incomplete update is ignored; complete corrupt records fail.
[[nodiscard]] NandImage load_image_journal(const PrivateFile &file, ImageJournalPosition &position);
void append_image_journal(PrivateFile &file,
                          ImageJournalPosition &position,
                          const NandImage &changes,
                          const PrivateFileObserver &observer = {});
}
