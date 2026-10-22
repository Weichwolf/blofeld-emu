// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld {
struct DspInterruptRequest {
  std::uint32_t vector = 0;
  unsigned level = 0;
  bool operator==(const DspInterruptRequest &) const = default;
};

// DSP56371 IRQA/SHI/TIM priorities. Other enabled sources fail.
class DspInterrupts {
public:
  void reset();
  [[nodiscard]] static bool handles(std::uint32_t address);
  [[nodiscard]] std::uint32_t read(std::uint32_t address) const;
  void write(std::uint32_t address, std::uint32_t value);
  // Logical synchronized pin observations; nullopt withdraws input knowledge.
  void observe_irqa(std::optional<bool> level);
  [[nodiscard]] std::optional<DspInterruptRequest> irqa_request(unsigned mask) const;
  void acknowledge_irqa();
  [[nodiscard]] std::optional<unsigned> priority(std::uint32_t vector) const;
  [[nodiscard]] std::optional<DspInterruptRequest> select(std::span<const std::uint32_t> pending,
                                                          unsigned mask) const;

private:
  std::uint32_t peripheral_ = 0;
  std::uint32_t core_ = 0;
  std::optional<bool> irqa_;
  std::optional<bool> irqa_edge_ = false;
};
}
