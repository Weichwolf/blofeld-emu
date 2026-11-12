// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/program.hpp"
#include "files/read.hpp"
#include "firmware.hpp"
#include "profile.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace blofeld::app::aot {
vi::dsp::Program load_dsp_program(const Profile &profile, const vi::dsp::BootRom boot) {
  if (profile.firmware.empty() || profile.dsp_catalogue.empty()) {
    throw std::invalid_argument("AOT requires firmware and dsp-catalogue profile paths");
  }
  constexpr std::size_t maximum_bytes = std::size_t{16} * 1024 * 1024;
  const auto input = host::read_file(profile.dsp_catalogue, maximum_bytes);
  return prepare_dsp_catalogue(
      std::string(input.begin(), input.end()), load_firmware(profile.firmware), boot);
}
}
