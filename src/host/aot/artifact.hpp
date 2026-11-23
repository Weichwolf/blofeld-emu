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
enum class Preparation : std::uint8_t { allow, load_only };

// Startup-only relocatable preparation data, keyed before lowering by inputs,
// native target/features and the complete host compiler pipeline identity.
class Artifact {
public:
  enum class Kind : std::uint8_t { semantic, native };
  Artifact(std::filesystem::path root,
           std::span<const std::byte> inputs,
           Kind kind = Kind::semantic);
  [[nodiscard]] std::optional<std::vector<std::byte>> load() const;
  void store(std::span<const std::byte> bytes) const;

private:
  std::filesystem::path root_;
  std::array<std::uint8_t, 32> key_{};
  std::string name_;
};
}
