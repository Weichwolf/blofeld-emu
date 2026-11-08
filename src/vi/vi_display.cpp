// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "display/lcd_image.hpp"
#include "display/st7565_parallel.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "vi/vi.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <ostream>

namespace blofeld::vi {
namespace {
namespace pin = st7565_pin;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned mebi_end = 0x10;

struct Wire {
  McuPort port;
  std::uint8_t mask;
  unsigned signal;
};
}

LcdHostPins VirtualInstrument::lcd_pins() const {
  if (!config_.lcd_parallel) { throw Fault("LCD parallel route is absent"); }
  if (schedule_ticks() < config_.lcd_parallel->reset_release_ticks) {
    return {.known = pin::reset_n};
  }
  // Explicit inferred parallel routing; no PC-dependent attachment or reply.
  constexpr std::array wires{Wire{.port = McuPort::b, .mask = 0x80, .signal = pin::cs1_n},
                             Wire{.port = McuPort::b, .mask = 0x40, .signal = pin::cs2},
                             Wire{.port = McuPort::e, .mask = 0x10, .signal = pin::enable},
                             Wire{.port = McuPort::b, .mask = 0x20, .signal = pin::read},
                             Wire{.port = McuPort::b, .mask = 1, .signal = pin::data}};
  unsigned known = pin::reset_n;
  unsigned control = pin::reset_n;
  for (const auto wire : wires) {
    const auto pads = vm_.mcu.bus().observe_pads(wire.port, wire.mask);
    if (pads.known == wire.mask) { known |= wire.signal; }
    if (pads.levels != 0) { control |= wire.signal; }
  }
  const auto data = vm_.mcu.bus().gpio_outputs(McuPort::a);
  return {.control = static_cast<std::uint8_t>(control),
          .known = static_cast<std::uint8_t>(known),
          .drive = data.mask,
          .data = {.value = data.levels, .known = data.known}};
}

void VirtualInstrument::prepare_lcd() {
  if (!config_.lcd_parallel) { return; }
  vm_.lcd.emplace();
  // The opt-in nominal-reset contract explicitly supplies the peripheral reset.
  // It does not claim a recovered physical /RES net, pulse width or busy latency.
  vm_.lcd->sample({.known = pin::reset_n});
  observe_lcd(hc12::porta);
}

void VirtualInstrument::observe_lcd(const unsigned address) {
  if (!vm_.lcd || address >= mebi_end) { return; }
  try {
    const auto result = vm_.lcd->sample(lcd_pins());
    const auto known = static_cast<unsigned>(result.data.known);
    const auto driven = static_cast<unsigned>(result.drive);
    vm_.mcu.bus().update_pin_drive(McuPort::a,
                                   byte_mask,
                                   {.mask = result.data.known,
                                    .levels = result.data.value,
                                    .floating = static_cast<std::uint8_t>(byte_mask & ~driven),
                                    .unknown = static_cast<std::uint8_t>(driven & ~known)});
    vm_.lcd_epoch = vm_.mcu.run_clock().native_epoch();
    if (!result.completed) { return; }
    const auto cycle = *result.completed;
    if (execution_.trace != nullptr) {
      *execution_.trace << "LCD-CYCLE tick=" << schedule_ticks() << " read=" << cycle.read
                        << " data=" << cycle.data << " value=" << hex(cycle.byte.value, 2)
                        << " known=" << hex(cycle.byte.known, 2) << " mcu-pc=" << mcu_pc_context()
                        << '\n';
    }
  } catch (const Fault &error) {
    throw Fault("LCD parallel operation=observe at=" + hex(address) +
                " mcu-pc=" + mcu_pc_context() + " dsp-pc=" + hex(vm_.dsp.cpu().registers().pc, 6) +
                ": " + error.what());
  }
}

void VirtualInstrument::observe_lcd_epoch() {
  if (!vm_.lcd || !config_.lcd_parallel) { return; }
  const bool release =
      vm_.lcd->reset_active() && schedule_ticks() >= config_.lcd_parallel->reset_release_ticks;
  if (release || vm_.lcd_epoch != vm_.mcu.run_clock().native_epoch()) { observe_lcd(hc12::porta); }
}

std::optional<LcdImage> VirtualInstrument::display_image(const LcdPanel panel) const {
  require_idle();
  return vm_.lcd ? std::optional{capture_lcd(vm_.lcd->controller(), panel)} : std::nullopt;
}
}
