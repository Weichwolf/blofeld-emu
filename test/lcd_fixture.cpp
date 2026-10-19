// SPDX-License-Identifier: GPL-3.0-only
#include "lcd_fixture.hpp"

#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/st7565.hpp"
#include "blofeld/st7565_parallel.hpp"

#include <array>
#include <cstdint>

namespace blofeld::test {
namespace {
namespace pin = st7565_pin;
constexpr unsigned byte_mask = 255;
constexpr unsigned enable_pad = 0x10;

struct Wire {
  McuPort port;
  std::uint8_t mask;
  unsigned signal;
};
}

LcdFixture::LcdFixture(McuBus &bus) : bus_(bus) {
  device_.sample({.known = pin::reset_n});
  device_.sample({.control = pin::reset_n | pin::cs1_n, .known = pin::all});
}

LcdFixture::LcdFixture(McuBus &bus, const LcdFixture &source)
    : bus_(bus),
      device_(source.device_),
      counts_(source.counts_),
      armed_(source.armed_),
      withdraw_(source.withdraw_) {}

void LcdFixture::observe(const BusEvent &event) {
  if (!event.write || !gpio_address(event.address)) { return; }
  if (!armed_) {
    const auto enable = bus_.observe_pads(McuPort::e, enable_pad);
    if (event.address != hc12::ddre || (bus_.read(hc12::ddre) & enable_pad) == 0 ||
        enable.known != enable_pad || enable.levels != 0) {
      return;
    }
    armed_ = true;
  }
  const auto result = device_.sample(sample());
  if (result.completed) {
    if (result.completed->read) {
      ++counts_.reads;
    } else if (result.completed->data) {
      ++counts_.writes;
    } else {
      ++counts_.commands;
    }
  }
  const auto data = withdraw_ && result.drive != 0 ? LcdByte{} : result.data;
  bus_.set_pin_drive(
      McuPort::a,
      {.mask = data.known,
       .levels = data.value,
       .floating = static_cast<std::uint8_t>(byte_mask & ~static_cast<unsigned>(result.drive))});
}

bool LcdFixture::gpio_address(const std::uint16_t address) {
  return address == hc12::porta || address == hc12::portb || address == hc12::porte ||
         address == hc12::ddra || address == hc12::ddrb || address == hc12::ddre;
}

LcdHostPins LcdFixture::sample() const {
  constexpr std::array<Wire, 5> wires{{{.port = McuPort::b, .mask = 0x80, .signal = pin::cs1_n},
                                       {.port = McuPort::b, .mask = 0x40, .signal = pin::cs2},
                                       {.port = McuPort::e, .mask = 0x10, .signal = pin::enable},
                                       {.port = McuPort::b, .mask = 0x20, .signal = pin::read},
                                       {.port = McuPort::b, .mask = 1, .signal = pin::data}}};
  unsigned control = pin::reset_n;
  unsigned known = pin::reset_n;
  for (const auto wire : wires) {
    const auto pad = bus_.observe_pads(wire.port, wire.mask);
    if (pad.known == wire.mask) { known |= wire.signal; }
    if (pad.levels != 0) { control |= wire.signal; }
  }
  const auto drive = bus_.read(hc12::ddra);
  const auto host = bus_.observe_pads(McuPort::a, drive);
  return {.control = static_cast<std::uint8_t>(control),
          .known = static_cast<std::uint8_t>(known),
          .drive = drive,
          .data = {.value = host.levels, .known = host.known}};
}
}
