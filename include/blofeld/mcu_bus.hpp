// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"

#include <array>
#include <functional>
#include <span>
#include <string_view>

namespace blofeld {

struct BusEvent {
  bool write = false;
  std::uint16_t address = 0;
  std::uint8_t value = 0;
  std::string_view name;
};

// Initial strict bring-up model. Unimplemented reads fail rather than returning
// fabricated ready flags. Register and pin coverage is expanded from traces.
class McuBus final : public Bus {
public:
  explicit McuBus(std::span<const std::uint8_t> flash);
  std::uint8_t read(std::uint16_t address) override;
  void write(std::uint16_t address, std::uint8_t value) override;
  [[nodiscard]] std::uint8_t peek(std::uint16_t address) const override;

  [[nodiscard]] std::uint64_t revision() const override { return revision_; }

  void check_execute(InstructionFetch fetch) const override;
  void set_observer(std::function<void(const BusEvent &)> observer);

private:
  static bool ram_mapped(std::uint16_t address);
  static std::string_view latch_name(std::uint16_t address);
  std::uint64_t revision_ = 0;
  std::array<std::uint8_t, 65536> bytes_{};
  std::function<void(const BusEvent &)> observer_;
};

} // namespace blofeld
