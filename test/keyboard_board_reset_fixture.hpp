// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_wire.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_reset.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_board_reset {
constexpr unsigned helper_bytes = 1018, oscillator_hz = 12000000, step_limit = 1000000;
constexpr unsigned caller = 0x2000, returned = caller + 3, stack = 0x2100;
constexpr unsigned sync_entry = 0x138c, sync_request = 0x13c9, helper_base = 0x1200;
constexpr unsigned reset_pin = 0x80, bkgd_pin = 2, target_period = 15, byte_mask = 255;

struct Saved {
  McuCheckpoint host;
  Hcs08BoardWireCheckpoint target;
  Hcs08ResetSequence reset;
  std::uint64_t next_edge, origin, steps, received;
};

struct Fixture {
  const Firmware &firmware;
  const std::vector<std::uint8_t> &helper;
  std::unique_ptr<McuMachine> owned_host;
  McuMachine &host;
  Hcs08Board pre_reset;
  std::optional<Hcs08ResetSequence> reset;
  std::optional<Hcs08BoardWire> target;
  std::uint64_t next_edge, origin = 0, steps = 0, received = 0;
  ExecutionContext context;
  std::ostream &trace;
  Fixture(const Firmware &image,
          const std::vector<std::uint8_t> &code,
          unsigned first_edge,
          unsigned timer_prescale,
          std::ostream &output);
  void bind();
  [[nodiscard]] Hcs08BoardWire &target_owner();
  [[nodiscard]] const Hcs08BoardWire &target_owner() const;
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
} // namespace blofeld::test::keyboard_board_reset
