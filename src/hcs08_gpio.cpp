// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_gpio.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/gpio_pads.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned registers_per_port = 4;
constexpr unsigned data = 0, pull = 1, slew = 2, direction = 3;
constexpr std::array<unsigned, 5> masks{0xff, 0xff, 0xff, 0x1f, 0x3f};

unsigned index(const Hcs08GpioPort port) {
  const auto value = static_cast<unsigned>(port);
  if (value >= masks.size()) { throw Fault("Unknown HCS08 GPIO port"); }
  return value;
}

void require_address(const unsigned address) {
  if (address >= masks.size() * registers_per_port) {
    throw Fault("unsupported register or peripheral ownership");
  }
}

std::string location(const unsigned address, const ExecutionContext context) {
  return " address=" + hex(address) + " pc=" + hex(context.pc) + " ccr=" + hex(context.ccr);
}
}

Hcs08Gpio::Hcs08Gpio(const Hcs08GpioContract contract) {
  if (contract != Hcs08GpioContract::candidate_gt8a) {
    throw Fault("HCS08 GPIO requires the explicit candidate GT8A contract");
  }
}

void Hcs08Gpio::reset() {
  registers_.fill({});
}

bool Hcs08Gpio::contains(const std::uint16_t address) {
  return address < masks.size() * registers_per_port;
}

void Hcs08Gpio::attach_pad_source(const Hcs08GpioPort port, GpioPadReader reader) {
  pads_[index(port)].attach(std::move(reader));
}

Hcs08GpioRegisters Hcs08Gpio::state(const Hcs08GpioPort port) const {
  return registers_[index(port)];
}

GpioNetDriver Hcs08Gpio::driver(const Hcs08GpioPort port, const std::uint8_t gpio_owned) const {
  const auto n = index(port);
  if ((gpio_owned & ~masks[n]) != 0) { throw Fault("Invalid HCS08 GPIO ownership mask"); }
  const auto s = registers_[n];
  const auto active = static_cast<unsigned>(s.direction & gpio_owned);
  return {.active = active,
          .known = active,
          .levels = s.latch & active,
          .pull_up = s.pull_enable & ~static_cast<unsigned>(s.direction) & gpio_owned};
}

std::uint8_t Hcs08Gpio::read(const std::uint16_t address, const ExecutionContext context) const
    try {
  require_address(address);
  const auto n = address / registers_per_port;
  const auto s = registers_[n];
  switch (address % registers_per_port) {
    case pull: return s.pull_enable;
    case slew: return s.slew_enable;
    case direction: return s.direction;
    case data: {
      const auto inputs = masks[n] & ~static_cast<unsigned>(s.direction);
      const auto sample = pads_[n].sample(inputs);
      require_known_gpio_pads(sample, inputs);
      return static_cast<std::uint8_t>((s.latch & s.direction) | sample.levels);
    }
    default: throw Fault("invalid register selector");
  }
} catch (const Fault &error) {

  throw Fault("HCS08 GPIO read" + location(address, context) + ": " + error.what());
}

void Hcs08Gpio::write(const ByteWrite access, const ExecutionContext context) try {
  require_address(access.address);
  const auto n = access.address / registers_per_port;
  const auto value = static_cast<std::uint8_t>(access.value & masks[n]);
  auto &s = registers_[n];
  // Only designated non-pin fields in GT chapter6 figures read zero and have
  // no storage. This is not a general policy for other reserved registers.
  switch (access.address % registers_per_port) {
    case data: s.latch = value; return;
    case pull: s.pull_enable = value; return;
    case slew: s.slew_enable = value; return;
    case direction: s.direction = value; return;
    default: throw Fault("invalid register selector");
  }
} catch (const Fault &error) {

  throw Fault("HCS08 GPIO write" + location(access.address, context) + ": " + error.what());
}
} // namespace blofeld
