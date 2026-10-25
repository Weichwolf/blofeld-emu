// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <iosfwd>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace blofeld::test::workspace {
inline constexpr unsigned mask = 0xffffff;
inline constexpr unsigned memory_mode = 0x600389;
inline constexpr unsigned allocate = 0x2b56;
inline constexpr unsigned remove_entry = 0x2b8c;
inline constexpr unsigned cleanup = 0x2b7b;
inline constexpr unsigned module_end = 0x2bce;
inline constexpr unsigned active = 0x6d2;
inline constexpr unsigned occupancy = 0x2f1c;
inline constexpr unsigned owners = 0x2f1e;
inline constexpr unsigned voice = 0xc500;
inline constexpr unsigned stride = 0x106;
inline constexpr unsigned voices = 25;
inline constexpr unsigned owner_count = 24;
inline constexpr unsigned cursor_offset = 17;
inline constexpr unsigned callback_offset = 62;
inline constexpr unsigned value_offset = 63;
inline constexpr unsigned idle = 0x160e;
inline constexpr unsigned target_slot = 0x800;
inline constexpr unsigned mode_slot = 0x801;
inline constexpr unsigned argument_slot = 0x803;
inline constexpr unsigned mode_mask = 0x200c00;
inline constexpr unsigned shift_pc = 0x2bb5;
inline constexpr unsigned fallback_pc = 0x2bca;
inline constexpr unsigned instruction_bound = 20000;

struct Case {
  unsigned id = 0;
  unsigned entry = 0;
  unsigned mode = 0;
  unsigned request = 0;
  unsigned selected = 0;
  unsigned x0 = 0;
  std::vector<std::pair<unsigned, unsigned>> writes;
};

struct Source {
  explicit Source(const char *path);
  Firmware firmware;
  std::map<unsigned, unsigned> program;
};

class Fixture {
public:
  Fixture(const Source &source, std::ostream &trace);
  void run(const Case &input);
  void report() const;

private:
  void caller();
  void prepare(const Case &input);
  void step();
  void until(unsigned pc);
  void state(const char *label) const;
  [[nodiscard]] std::vector<std::uint32_t> ram();
  static void compare_ram(const std::vector<std::uint32_t> &before,
                          const std::vector<std::uint32_t> &after);
  void verify(const Case &input) const;
  void replay(const DspCheckpoint &checkpoint,
              const DspDebugState &expected,
              const std::vector<std::uint32_t> &memory,
              unsigned prefix_steps);
  void put(unsigned address, unsigned value);
  [[nodiscard]] unsigned get(unsigned address);
  DspMachine machine_;
  const Source &source_;
  std::ostream &trace_;
  std::set<unsigned> visited_;
  std::array<unsigned, 14> arguments_{};
  unsigned begin_ = 0;
  unsigned returned_ = 0;
  unsigned steps_ = 0;
  unsigned maximum_sp_ = 0;
  unsigned calls_ = 0;
  unsigned snapshots_ = 0;
  unsigned replays_ = 0;
  bool quiet_ = false;
};
}
