// SPDX-License-Identifier: GPL-3.0-only
#include "process/configuration.hpp"

#include <filesystem>
#include <stdexcept>
#include <string_view>
#include <unistd.h>

namespace blofeld::host {
namespace {
std::string_view inherited(const std::string_view name) {
  for (const auto *entry = environ; *entry != nullptr; ++entry) {
    const std::string_view value(*entry);
    if (value.starts_with(name) && value.size() > name.size() && value[name.size()] == '=') {
      return value.substr(name.size() + 1);
    }
  }
  return {};
}
}

std::filesystem::path configuration_home() {
  const auto configured = inherited("XDG_CONFIG_HOME");
  const auto home = inherited("HOME");
  if (configured.empty() && home.empty()) {
    throw std::runtime_error("Configuration home is unset");
  }
  const auto root = configured.empty() ? std::filesystem::path(home) / ".config"
                                       : std::filesystem::path(configured);
  if (!root.is_absolute()) { throw std::runtime_error("Configuration home must be absolute"); }
  return root;
}
}
