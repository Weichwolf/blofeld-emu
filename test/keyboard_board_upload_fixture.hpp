// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/hcs08_board_wire.hpp"
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard_board_transport_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::keyboard_board_upload {
constexpr unsigned frame = 0x2100, prepare_entry = 0x124d, prepare_end = 0x1281;
constexpr unsigned start_entry = 0x12fc, target_first = 0x80, target_last = 0x124;
constexpr unsigned source_algorithm = 0x1546, algorithm_offset = source_algorithm - 0x1200;
constexpr unsigned algorithm_bytes = 132, page_bytes = 512, record_bytes = 514;
constexpr unsigned saved_trim = 0x1602, trim_address = 0xffbe, expected_trim = 0xa5;
constexpr unsigned strict_site = 0xb2, strict_periods = 26, strict_retired = 8;

struct Saved {
  McuCheckpoint host;
  Hcs08BoardWireCheckpoint target;
  std::vector<unsigned> expected;
  unsigned cursor;
  std::uint64_t steps, received, pin_writes;
  bool go_checked;
};

struct Runner {
  keyboard_board_transport::Fixture pair;
  bool secure;
  bool detailed = false, go_checked = false;
  Hcs08FlashImage initial_flash;

  Runner(const Firmware &firmware,
         const std::vector<std::uint8_t> &helper,
         unsigned phase,
         bool secured,
         std::ostream &trace,
         const Hcs08BoardWireCheckpoint *snapshot = nullptr);
  [[nodiscard]] static Hcs08FlashImage preimage(bool secure);
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
} // namespace blofeld::test::keyboard_board_upload
