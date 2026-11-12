// SPDX-License-Identifier: GPL-3.0-only
#include "process/configuration.hpp"

#include <filesystem>
#if defined(__APPLE__)
#include <crt_externs.h>
#include <vector>
#else
#include <string>
#endif
#include <stdexcept>
#include <string_view>
#include <unistd.h>

namespace blofeld::host {
std::string_view inherited_setting(const std::string_view name) {
#if defined(__APPLE__)
  auto *const environment = *_NSGetEnviron();
#else
  auto *const environment = environ;
#endif
  for (const auto *entry = environment; *entry != nullptr; ++entry) {
    const std::string_view value(*entry);
    if (value.starts_with(name) && value.size() > name.size() && value[name.size()] == '=') {
      return value.substr(name.size() + 1);
    }
  }
  return {};
}

std::filesystem::path configuration_home() {
  const auto configured = inherited_setting("XDG_CONFIG_HOME");
  const auto home = inherited_setting("HOME");
  if (configured.empty() && home.empty()) {
    throw std::runtime_error("Configuration home is unset");
  }
  const auto root = configured.empty() ? std::filesystem::path(home) / ".config"
                                       : std::filesystem::path(configured);
  if (!root.is_absolute()) { throw std::runtime_error("Configuration home must be absolute"); }
  return root;
}

std::filesystem::path runtime_home() {
  const auto configured = inherited_setting("XDG_RUNTIME_DIR");
#if defined(__APPLE__)
  auto root = std::filesystem::path(configured);
  if (root.empty()) {
    const auto required = ::confstr(_CS_DARWIN_USER_TEMP_DIR, nullptr, 0);
    if (required == 0) { throw std::runtime_error("User runtime directory is unavailable"); }
    std::vector<char> path(required);
    if (::confstr(_CS_DARWIN_USER_TEMP_DIR, path.data(), path.size()) != required) {
      throw std::runtime_error("User runtime directory changed during lookup");
    }
    root = path.data();
  }
#else
  const auto root = configured.empty()
                        ? std::filesystem::path("/run/user") / std::to_string(::getuid())
                        : std::filesystem::path(configured);
#endif
  if (!root.is_absolute()) { throw std::runtime_error("Runtime home must be absolute"); }
  return root;
}
}
