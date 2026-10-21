// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usbip_runtime.hpp"
#include "usb_device_native_fixture.hpp"
#include "usb_host_fixture.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <utility>

namespace blofeld::test::usbip_runtime {
using namespace blofeld::test::usb_host;
namespace native = blofeld::test::usb_device_native;
inline constexpr unsigned default_probe_budget = 1000;

inline UsbipRuntimeOptions options() {
  return {.tcp = {},
          .location = {.path = "/own/usb/1-1", .busid = "1-1", .bus = 1, .device = 1},
          .after_ticks = 10,
          .period_ticks = 1,
          .probe_timeout_ticks = default_probe_budget};
}

struct Fixture {
  explicit Fixture(UsbipRuntimeOptions config = options())
      : firmware(native::firmware()),
        bus(firmware),
        cpu(bus, BLOFELD_DEFAULT_SLA),
        runtime(std::move(config)),
        boundary{.owner = std::make_shared<const int>(0)} {
    native::prepare_native(bus);
    cpu.set_state(native::initial);
  }

  void poll(const std::uint64_t ticks) {
    boundary.ticks = ticks;
    const auto state = cpu.state();
    static_cast<void>(runtime.poll(bus, {.pc = state.pc, .ccr = state.ccr}, boundary));
  }

  void capture() {
    poll(9);
    require(!runtime.started() && (word(bus, status) & 1U) == 0,
            "No host SETUP or synthetic initialization precedes explicit admission");
    finish_capture(10);
  }

  void finish_capture(const std::uint64_t start) {
    for (std::uint64_t stage = 0; stage < 3; ++stage) {
      const auto tick = start + (stage * 3);
      poll(tick);
      native::interrupt(cpu, {.vector = native::setup_vector}, native::setup_instructions);
      poll(tick + 1);
      native::interrupt(cpu, {.vector = native::input_vector}, 3);
      poll(tick + 2);
      native::interrupt(cpu, {.vector = native::output_vector}, 3);
    }
    require(runtime.published(), "Only complete own native control transfers publish descriptors");
  }

  std::array<std::uint8_t, 32768> firmware;
  McuBus bus;
  Hc12 cpu;
  UsbipRuntime runtime;
  UsbipRuntimeBoundary boundary;
};

}
