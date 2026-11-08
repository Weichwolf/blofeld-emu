// SPDX-License-Identifier: GPL-3.0-only
#include "firmware.hpp"

#include "archive/zip.hpp"
#include "files/read.hpp"
#include "storage/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>

namespace blofeld::app {
vi::FirmwareUpdate load_firmware(const std::filesystem::path &path) {
  constexpr std::size_t maximum_bytes = std::size_t{64} * 1024 * 1024;
  const auto bytes = host::read_file(path, maximum_bytes);
  constexpr std::array<std::uint8_t, 4> local_signature{'P', 'K', 3, 4};
  if (bytes.size() >= local_signature.size() &&
      std::equal(local_signature.begin(), local_signature.end(), bytes.begin())) {
    return vi::FirmwareUpdate::from_midi(host::zip_member(bytes, ".mid", maximum_bytes));
  }
  return vi::FirmwareUpdate::from_midi(bytes);
}
}
