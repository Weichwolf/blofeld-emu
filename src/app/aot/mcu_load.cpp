// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"
#include "files/read.hpp"
#include "firmware.hpp"
#include "mcu/program.hpp"
#include "profile.hpp"

#include "llvm/Support/Error.h"
#include "llvm/Support/JSON.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::app::aot {
namespace {
constexpr std::size_t maximum_bytes = std::size_t{16} * 1024 * 1024;
using Sources = std::map<std::string, std::vector<std::uint8_t>>;

Sources load_sources(const std::string_view document, const std::filesystem::path &directory) {
  auto parsed = llvm::json::parse(document);
  if (!parsed) { throw std::invalid_argument(llvm::toString(parsed.takeError())); }
  const auto *root = parsed->getAsObject();
  const auto *input = root != nullptr ? root->getObject("inputs") : nullptr;
  if (input == nullptr) { throw std::invalid_argument("MCU catalogue requires input identities"); }
  Sources result;
  if (input->get("blobs") == nullptr) { return result; }
  const auto *blobs = input->getArray("blobs");
  if (blobs == nullptr) { throw std::invalid_argument("MCU catalogue blobs must be an array"); }
  for (const auto &value : *blobs) {
    const auto *blob = value.getAsObject();
    const auto name = blob != nullptr ? blob->getString("name") : std::nullopt;
    const auto path = blob != nullptr ? blob->getString("path") : std::nullopt;
    if (!name || name->empty() || !path || path->empty() || result.contains(name->str())) {
      throw std::invalid_argument("MCU catalogue blob requires a unique name and path");
    }
    result.emplace(name->str(), host::read_file(directory / path->str(), maximum_bytes));
  }
  return result;
}
}

vi::mcu::Program load_mcu_program(const Profile &profile) {
  if (profile.firmware.empty() || profile.mcu_catalogue.empty()) {
    throw std::invalid_argument("AOT requires firmware and mcu-catalogue profile paths");
  }
  const auto input = host::read_file(profile.mcu_catalogue, maximum_bytes);
  const std::string document(input.begin(), input.end());
  const auto owned = load_sources(document, profile.mcu_catalogue.parent_path());
  std::vector<McuSource> sources;
  for (const auto &[name, bytes] : owned) { sources.push_back({.name = name, .bytes = bytes}); }
  return prepare_mcu_catalogue(document, load_firmware(profile.firmware), sources);
}
}
