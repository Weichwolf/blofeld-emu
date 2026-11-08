// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/ports.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <optional>
#include <stdexcept>

namespace blofeld::vi::dsp {
namespace {
struct Spec {
  dsp56300::Space space;
  dsp56300::Word base, mask;
};

constexpr std::array specs{Spec{.space = dsp56300::Space::x, .base = 0xffffbd, .mask = 0xfff},
                           Spec{.space = dsp56300::Space::x, .base = 0xffffd5, .mask = 3},
                           Spec{.space = dsp56300::Space::y, .base = 0xffff9d, .mask = 0xfff},
                           Spec{.space = dsp56300::Space::y, .base = 0xfffff5, .mask = 0x7ff}};

std::optional<std::size_t> decode(const dsp56300::Space space,
                                  const dsp56300::Word address) noexcept {
  for (std::size_t index = 0; index < specs.size(); ++index) {
    if (space == specs[index].space && address >= specs[index].base &&
        address - specs[index].base < 3) {
      return index;
    }
  }
  return {};
}
}

Ports::Ports() {
  registers_[1].known = 3; // PDRD resets to zero; C/E/F data latches are retained.
}

void Ports::reset() noexcept {
  for (auto &port : registers_) {
    port.direction = 0;
    port.function = 0;
    port.input = {};
  }
  registers_[1].data = 0;
  registers_[1].known = 3;
}

bool Ports::contains(const dsp56300::Space space, const dsp56300::Word address) noexcept {
  return decode(space, address).has_value();
}

bool Ports::read(const dsp56300::Space space,
                 const dsp56300::Word address,
                 dsp56300::Word &value) const noexcept {
  const auto index = decode(space, address);
  if (!index) { return false; }
  const auto &port = registers_[*index];
  const auto &spec = specs[*index];
  if (address == spec.base + 1) {
    value = port.direction;
    return true;
  }
  if (address == spec.base + 2) {
    value = port.function;
    return true;
  }
  const auto inputs = port.function & ~port.direction & spec.mask;
  // GPIO inputs have documented pull-downs. Disconnected latches are retained
  // when known; no serial peripheral input or peer response is synthesized.
  auto known = (port.known & ~inputs) | inputs;
  auto sample = port.data & ~inputs;
  const auto attached = port.input.mask & inputs;
  known = (known & ~attached) | (port.input.known & attached);
  sample = (sample & ~attached) | (port.input.value & attached);
  if (*index == 3) { known &= ~(port.function & port.direction); }
  if ((known & spec.mask) != spec.mask) { return false; }
  value = sample & spec.mask;
  return true;
}

bool Ports::write(const Write access) noexcept {
  const auto index = decode(access.space, access.address);
  if (!index || access.value > dsp56300::word_mask) { return false; }
  auto &port = registers_[*index];
  const auto &spec = specs[*index];
  const auto value = access.value & spec.mask;
  switch (access.address - spec.base) {
    case 0:
      port.data = value;
      port.known = spec.mask;
      return true;
    case 1: port.direction = value; return true;
    case 2: port.function = value; return true;
    default: return false;
  }
}

void Ports::input(const Port port, const Input sample) {
  const auto index = static_cast<std::size_t>(port);
  if (index >= registers_.size()) { throw std::out_of_range("Unknown DSP port"); }
  registers_[index].input = sample;
}

Ports::Output Ports::output(const Port port) const {
  const auto index = static_cast<std::size_t>(port);
  if (index >= registers_.size()) { throw std::out_of_range("Unknown DSP port"); }
  const auto &value = registers_[index];
  return {.mask = value.direction & ~value.function & specs[index].mask,
          .known = value.known,
          .value = value.data};
}

dsp56300::Word Ports::peripheral(const Port port) const {
  const auto index = static_cast<std::size_t>(port);
  if (index >= registers_.size()) { throw std::out_of_range("Unknown DSP port"); }
  const auto &value = registers_[index];
  return index == 3 ? 0 : value.direction & value.function & specs[index].mask;
}
}
