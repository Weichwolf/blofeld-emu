// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <functional>
#include <span>

namespace blofeld {

struct GpioPadSample {
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  std::uint32_t conflicts = 0;
};

struct GpioNetDriver {
  std::uint32_t active = 0;
  std::uint32_t known = 0;
  std::uint32_t levels = 0;
  std::uint32_t reduced = 0;
  std::uint32_t pull_up = 0;
  std::uint32_t pull_down = 0;
};

using GpioPadReader = std::function<GpioPadSample(std::uint32_t)>;

// Resolved pad input is outside chip checkpoints. Copies keep the obligation to
// reconnect it, never a callback into another live board. Register state is separate.
class GpioPadBinding {
public:
  GpioPadBinding() = default;
  GpioPadBinding(const GpioPadBinding &source);
  GpioPadBinding &operator=(const GpioPadBinding &source);
  GpioPadBinding(GpioPadBinding &&source) noexcept;
  GpioPadBinding &operator=(GpioPadBinding &&source) noexcept;
  ~GpioPadBinding() = default;

  void attach(GpioPadReader reader);
  [[nodiscard]] bool required() const;
  [[nodiscard]] GpioPadSample sample(std::uint32_t mask) const;

private:
  bool required_ = false;
  GpioPadReader reader_;
};

// Full/reduced outputs both dominate passive pulls. Known opposing active levels
// remain unknown; an approximate current ratio never selects a voltage winner.
[[nodiscard]] GpioPadSample resolve_gpio_net(std::uint32_t mask,
                                             std::span<const GpioNetDriver> drivers);
[[nodiscard]] GpioPadSample
resolve_gpio_net(std::uint32_t mask, GpioNetDriver first, GpioNetDriver second);
void require_known_gpio_pads(GpioPadSample sample, std::uint32_t mask);

} // namespace blofeld
