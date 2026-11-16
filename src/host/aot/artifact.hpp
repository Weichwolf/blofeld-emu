// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace blofeld::host::aot {
// Startup-only relocatable preparation data, keyed before lowering by inputs,
// native target/features and the complete host compiler pipeline identity.
class Artifact {
public:
  Artifact(std::filesystem::path root, std::span<const std::byte> inputs);
  [[nodiscard]] std::optional<std::vector<std::byte>> load() const;
  void store(std::span<const std::byte> bytes) const;

private:
  std::filesystem::path root_;
  std::array<std::uint8_t, 32> key_{};
  std::string name_;
};
}
