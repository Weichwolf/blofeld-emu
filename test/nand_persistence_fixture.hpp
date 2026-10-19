// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/nand.hpp"
#include "blofeld/nand_array.hpp"
#include "blofeld/nand_image.hpp"

#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <system_error>

namespace blofeld::test {
inline constexpr unsigned persisted_row = 37;
inline constexpr unsigned persisted_block = persisted_row / Nand::block_pages;
inline constexpr unsigned persisted_neighbor = (persisted_block + 1) * Nand::block_pages;
inline constexpr unsigned persisted_cookie = 0x42;
inline constexpr unsigned persisted_mask = 0xff;

inline void persistence_require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

inline void persistence_reject(const auto &action, const std::string &reason) {
  try {
    action();
  } catch (const Fault &error) {
    persistence_require(std::string(error.what()).contains(reason),
                        "Unexpected rejection: " + std::string(error.what()));
    return;
  }
  throw Fault("Invalid persistent state accepted: " + reason);
}

class PersistenceDirectory {
public:
  PersistenceDirectory() {
    std::random_device entropy;
    constexpr unsigned attempts = 64;
    for (unsigned attempt = 0; attempt < attempts; ++attempt) {
      const auto name =
          "blofeld-nand-" + std::to_string(entropy()) + "-" + std::to_string(entropy());
      const auto candidate = std::filesystem::temp_directory_path() / name;
      if (!std::filesystem::create_directory(candidate)) { continue; }
      std::filesystem::permissions(candidate, std::filesystem::perms::owner_all);
      path_ = candidate;
      return;
    }
    throw Fault("Cannot create private NAND test directory");
  }

  ~PersistenceDirectory() {
    std::error_code error;
    static_cast<void>(std::filesystem::remove_all(path_, error));
  }

  PersistenceDirectory(const PersistenceDirectory &) = delete;
  PersistenceDirectory &operator=(const PersistenceDirectory &) = delete;

  [[nodiscard]] std::filesystem::path file(const std::string &name = "state.bfnand") const {
    return path_ / name;
  }

private:
  std::filesystem::path path_;
};

inline NandImage erased_nand_fixture() {
  NandImage image;
  image.blocks[persisted_block] = {
      .valid = true, .program_protected = false, .erase_protected = false};
  NandArray::Page erased;
  erased.cells.fill({.value = persisted_mask, .known = persisted_mask});
  // Explicit prior-erase observation; these counts are not inferred from FF.
  erased.programs = NandProgramCount{};
  for (unsigned page = persisted_block * Nand::block_pages;
       page < (persisted_block + 1) * Nand::block_pages;
       ++page) {
    image.pages.emplace(page, erased);
  }
  image.pages[persisted_neighbor].cells.at(0) = {.value = persisted_cookie,
                                                 .known = persisted_mask};
  return image;
}

inline void prepare_persistent_program(auto &device) {
  device.observe_power_ready();
  device.observe_wp(true);
  constexpr unsigned program = 0x80;
  device.command(program);
  for (const auto address : {0U, persisted_row, 0U, 0U}) {
    device.address(static_cast<std::uint8_t>(address));
  }
}
}
