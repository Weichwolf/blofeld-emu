// SPDX-License-Identifier: GPL-3.0-only
#include "profile.hpp"

#include "boot_rom.hpp"
#include "files/read.hpp"
#include "process/configuration.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <istream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld::app {
Profile load_profile(const std::filesystem::path &path) {
  constexpr std::size_t maximum_bytes = 16384;
  const auto bytes = host::read_file(path, maximum_bytes);
  if (bytes.empty()) { throw std::runtime_error("Instrument profile is empty"); }
  const std::string text(reinterpret_cast<const char *>(bytes.data()), bytes.size());
  if (text.contains('\0')) { throw std::runtime_error("NUL in instrument profile"); }
  std::istringstream input(text);
  Profile profile;
  std::set<std::string> seen;
  std::string line;
  const auto base = std::filesystem::absolute(path).parent_path();
  while (std::getline(input, line)) {
    std::istringstream fields(line);
    fields >> std::ws;
    if (fields.eof() || fields.peek() == '#') { continue; }
    std::string key;
    std::string value;
    std::string extra;
    if (!(fields >> key >> std::quoted(value)) || value.empty() || (fields >> extra) ||
        !seen.insert(key).second) {
      throw std::runtime_error("Invalid or duplicate instrument profile field: " + line);
    }
    const auto file = (base / value).lexically_normal();
    using Field = std::pair<std::string_view, std::filesystem::path *>;
    const std::array<Field, 5> fields_by_name{{{"firmware", &profile.firmware},
                                               {"mcu-catalogue", &profile.mcu_catalogue},
                                               {"dsp-catalogue", &profile.dsp_catalogue},
                                               {"device-state", &profile.device_state},
                                               {"nand", &profile.storage}}};
    const auto *const found = std::ranges::find(fields_by_name, key, &Field::first);
    if (found != fields_by_name.end()) {
      *found->second = file;
    } else if (key == "boot-rom" || key == "boot-prefix") {
      if (!profile.boot_rom.empty()) { throw std::runtime_error("Conflicting DSP boot inputs"); }
      profile.boot_rom = file;
      profile.boot_format = key == "boot-prefix" ? BootFormat::spi_prefix : BootFormat::rom;
    } else {
      throw std::runtime_error("Unknown instrument profile field: " + key);
    }
  }
  if (profile.firmware.empty() || profile.boot_rom.empty() || profile.dsp_catalogue.empty() ||
      profile.device_state.empty()) {
    throw std::runtime_error(
        "Profile requires firmware, one DSP boot input, dsp-catalogue and device-state paths");
  }
  return profile;
}

std::filesystem::path default_profile() {
  return host::configuration_home() / "blofeld-emu/default.profile";
}
}
