// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/gpio_pads.hpp"

#include "blofeld/bus.hpp"

#include <cstdint>
#include <utility>

namespace blofeld {
namespace {
void validate_sample(const GpioPadSample sample, const std::uint32_t mask) {
  if (((sample.known | sample.conflicts) & ~mask) != 0 || (sample.levels & ~sample.known) != 0 ||
      (sample.known & sample.conflicts) != 0) {
    throw Fault("Invalid resolved GPIO pad observation");
  }
}

void validate_driver(const GpioNetDriver driver, const std::uint32_t mask) {
  if (((driver.active | driver.pull_up | driver.pull_down) & ~mask) != 0 ||
      (driver.known & ~driver.active) != 0 || (driver.levels & ~driver.known) != 0 ||
      (driver.reduced & ~driver.active) != 0 || (driver.pull_up & driver.pull_down) != 0 ||
      ((driver.pull_up | driver.pull_down) & driver.active) != 0) {
    throw Fault("Invalid GPIO net driver description");
  }
}
}

GpioPadBinding::GpioPadBinding(const GpioPadBinding &source) : required_(source.required_) {}

GpioPadBinding &GpioPadBinding::operator=(const GpioPadBinding &source) {
  if (this != &source) {
    required_ = source.required_;
    reader_ = {};
  }
  return *this;
}

GpioPadBinding::GpioPadBinding(GpioPadBinding &&source) noexcept : required_(source.required_) {}

GpioPadBinding &GpioPadBinding::operator=(GpioPadBinding &&source) noexcept {
  if (this != &source) {
    required_ = source.required_;
    reader_ = {};
  }
  return *this;
}

void GpioPadBinding::attach(GpioPadReader reader) {
  if (!reader || reader_) { throw Fault("Invalid or already attached resolved GPIO pad source"); }
  reader_ = std::move(reader);
  required_ = true;
}

bool GpioPadBinding::required() const {
  return required_;
}

GpioPadSample GpioPadBinding::sample(const std::uint32_t mask) const {
  if (mask == 0) { return {}; }
  if (!reader_) { throw Fault("Detached resolved GPIO pad source mask=" + hex(mask, 6)); }
  const auto result = reader_(mask);
  validate_sample(result, mask);
  return result;
}

GpioPadSample
resolve_gpio_net(const std::uint32_t mask, const GpioNetDriver first, const GpioNetDriver second) {
  validate_driver(first, mask);
  validate_driver(second, mask);
  const auto unknown = (first.active & ~first.known) | (second.active & ~second.known);
  const auto opposed = first.known & second.known & (first.levels ^ second.levels);
  const auto idle = mask & ~(first.active | second.active);
  const auto up = first.pull_up | second.pull_up;
  const auto down = first.pull_down | second.pull_down;
  const auto bias_conflict = idle & up & down;
  const auto known = (((first.known | second.known) & ~unknown & ~opposed) |
                      (idle & (up | down) & ~bias_conflict)) &
                     mask;
  return {.known = known,
          .levels = (first.levels | second.levels | (idle & up)) & known,
          .conflicts = opposed | bias_conflict};
}

void require_known_gpio_pads(const GpioPadSample sample, const std::uint32_t mask) {
  validate_sample(sample, mask);
  if (sample.known != mask || sample.conflicts != 0) {
    throw Fault("Unqualified GPIO net voltage mask=" + hex(mask & ~sample.known, 6) +
                " conflicts=" + hex(sample.conflicts, 6));
  }
}
} // namespace blofeld
