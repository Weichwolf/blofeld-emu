// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "aot/code.hpp"
#include "mcu/program.hpp"
#include "profile.hpp"
#include "storage/firmware.hpp"

#include <array>
#include <cpu12/core.hpp>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace blofeld::app::aot {
class McuBinding {
public:
  McuBinding();
  McuBinding(const McuBinding &) = delete;
  McuBinding &operator=(const McuBinding &) = delete;
  [[nodiscard]] static std::span<const std::byte> bitcode() noexcept;

  [[nodiscard]] std::span<const host::aot::Import> imports() const noexcept { return imports_; }

private:
  static constexpr std::size_t channel_count = 18;
  std::array<std::string, channel_count> signatures_;
  std::array<host::aot::Import, channel_count> imports_;
};

struct McuSource {
  std::string_view name;
  std::span<const std::uint8_t> bytes;
};

[[nodiscard]] cpu12::NativeImage
prepare_mcu(cpu12::CodeImage image, std::span<const cpu12::CodeTransition> transitions = {});
[[nodiscard]] vi::mcu::Program
prepare_mcu_program(cpu12::CodeImage image,
                    std::span<const cpu12::CodeTransition> transitions = {});
[[nodiscard]] vi::mcu::Program prepare_mcu_catalogue(std::string_view document,
                                                     const vi::FirmwareUpdate &firmware,
                                                     std::span<const McuSource> sources = {});
[[nodiscard]] vi::mcu::Program load_mcu_program(const Profile &profile);

}
