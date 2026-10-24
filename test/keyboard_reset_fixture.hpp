// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/gpio_pads.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_bdc_wire.hpp"
#include "blofeld/hcs08_reset.hpp"
#include "blofeld/mcu_machine.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "hcs08_wire_fixture.hpp"

#include <cstdint>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_reset {
constexpr unsigned helper_bytes = 1018, oscillator_hz = 12000000, step_limit = 1000000;
constexpr unsigned caller = 0x2000, returned = caller + 3, stack = 0x2100;
constexpr unsigned sync_entry = 0x138c, sync_request = 0x13c9, helper_base = 0x1200;
constexpr unsigned reset_pin = 0x80, bkgd_pin = 2, target_period = 15, byte_mask = 255;

struct Saved {
  McuCheckpoint host;
  hcs08_programmer::Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08ResetSequence reset;
  Hcs08BdcCheckpoint bdc;
  Hcs08BdcWireCheckpoint wire;
  std::uint64_t next_edge, origin, steps, received;
};

struct Fixture {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  McuMachine host;
  hcs08_wire::Target target;
  std::optional<Hcs08ResetSequence> reset;
  std::optional<Hcs08Bdc> bdc;
  std::optional<Hcs08BdcWire> wire;
  std::uint64_t next_edge, origin = 0, steps = 0, received = 0;
  ExecutionContext context;
  std::ostream &trace;
  Fixture(const Firmware &image,
          const std::vector<std::uint8_t> &code,
          unsigned first_edge,
          unsigned timer_prescale,
          std::ostream &output,
          const hcs08_programmer::Memory *memory = nullptr);
  void bind();
  [[nodiscard]] Hcs08Bdc &controller();
  [[nodiscard]] const Hcs08Bdc &controller() const;
  [[nodiscard]] Hcs08BdcWire &transport();
  [[nodiscard]] const Hcs08BdcWire &transport() const;
  [[nodiscard]] Hcs08ResetSequence &reset_sequence();
  [[nodiscard]] const Hcs08ResetSequence &reset_sequence() const;
  [[nodiscard]] Hcs08ResetDrive reset_drive() const;
  [[nodiscard]] Hcs08BdcDrive bkgd_drive() const;
  [[nodiscard]] GpioPadSample pads(McuPort port, std::uint32_t mask) const;
  void observe(const BusEvent &event);
  void advance(Hc12BusCycle cycle);
  void edge(std::uint64_t ticks);
  void step();
  void finish(unsigned timer_prescale);
  [[nodiscard]] Saved checkpoint() const;
  void restore(const Saved &saved);
  [[nodiscard]] std::string endpoint() const;
};
}
