// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "hcs08_programmer_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"
#include "keyboard_bdc_wire_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_target {
constexpr unsigned frame = 0x2100, prepare_entry = 0x124d, prepare_end = 0x1281;
constexpr unsigned start_entry = 0x12fc, target_first = 0x80, target_last = 0x124;
constexpr unsigned source_algorithm = 0x1546, algorithm_offset = source_algorithm - 0x1200;
constexpr unsigned algorithm_bytes = 132, page_bytes = 512, record_bytes = 514;
constexpr unsigned saved_trim = 0x1602, trim_address = 0xffbe, expected_trim = 0xa5;
constexpr unsigned initial_pc = 0xe200, initial_sp = 0x3ab, initial_hx = 0x1207;
constexpr unsigned initial_a = 0x5a, initial_ccr = 0x79, canary = 0xcc;
constexpr unsigned strict_site = 0xb2, strict_periods = 26, strict_retired = 8;

struct Saved {
  McuCheckpoint host;
  hcs08_programmer::Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08BdcWireCheckpoint wire;
  Hcs08RunCheckpoint clock;
  std::vector<unsigned> expected;
  unsigned cursor;
  std::uint64_t steps, received, pin_writes;
  bool go_checked;
};

struct Runner {
  keyboard_bdc_wire::Fixture pair;
  Hcs08RunClock clock;
  bool secure;
  bool detailed = false, go_checked = false;
  Hcs08FlashImage initial_flash;

  Runner(const Firmware &firmware,
         const std::vector<std::uint8_t> &helper,
         unsigned phase,
         bool secured,
         std::ostream &trace,
         const hcs08_programmer::Memory *snapshot = nullptr);
  void target_tick(std::uint64_t ticks);
  void step();
  void prepare();
  void buffer(const std::vector<std::uint8_t> &records);
  void start(const std::vector<std::uint8_t> &records);
  void until_fault();
  void guards() const;
  [[nodiscard]] Saved checkpoint() const;
  void restore(const Saved &saved);
  [[nodiscard]] std::string endpoint() const;
};
} // namespace blofeld::test::keyboard_target
