// SPDX-License-Identifier: GPL-3.0-only
#include "profile.hpp"

#include "files/read.hpp"
#include "process/configuration.hpp"

#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <istream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

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
    if (key == "firmware") {
      profile.firmware = file;
    } else if (key == "boot-rom") {
      profile.boot_rom = file;
    } else if (key == "device-state") {
      profile.device_state = file;
    } else if (key == "nand") {
      profile.storage = file;
    } else if (key == "display") {
      profile.display = file;
    } else {
      throw std::runtime_error("Unknown instrument profile field: " + key);
    }
  }
  if (profile.boot_rom.empty() || profile.device_state.empty()) {
    throw std::runtime_error("Profile requires boot-rom and device-state paths");
  }
  return profile;
}

std::filesystem::path default_profile() {
  return host::configuration_home() / "blofeld-emu/default.profile";
}
}
