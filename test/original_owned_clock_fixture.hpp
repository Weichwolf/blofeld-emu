// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_machine.hpp"
#include "lcd_fixture.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld::test::owned_clock {
inline constexpr unsigned input_read = 0x8301;
inline constexpr unsigned main_wait = 0x8314;
inline constexpr unsigned main_resume = 0x8315;
inline constexpr unsigned main_stack = 0x25f3;
inline constexpr unsigned wait_stack = 0x25ea;
inline constexpr unsigned saved_input = 0x182f;
inline constexpr unsigned realtime_vector = 0xfff0;
inline constexpr unsigned realtime_handler = 0x88a5;
inline constexpr unsigned return_interrupt = 0x0b;
inline constexpr unsigned byte_mask = 0xff;
inline constexpr std::uint64_t prefix_cycles = 137638116;
inline constexpr std::uint64_t main_cycles = 141809799;
inline constexpr std::uint64_t main_ticks = 283620262;
inline constexpr std::uint64_t main_oscillator = 56724052;

[[nodiscard]] std::filesystem::path artifact(const std::filesystem::path &trace,
                                             std::string_view suffix);
[[nodiscard]] bool same_cpu(const CpuState &first, const CpuState &second);
void check_nvm(const McuMachine &machine, const McuFlashImage &initial);

struct Inputs {
  void observe(McuMachine &machine);
  bool phy = false;
  bool capture = false;
};

struct Census {
  struct Row {
    std::string instruction;
    std::array<unsigned, 6> bytes{};
    std::array<std::uint64_t, 16> flags{};
  };

  void before(McuMachine &machine);
  void retire(CpuState before, CpuStep result);
  void write(const std::filesystem::path &path) const;
  std::map<unsigned, Row> rows;
  std::uint64_t retired = 0;
};

struct FlashPhases {
  void observe(Hc12BusCycle cycle, std::uint64_t before);
  void write(const std::filesystem::path &path) const;
  std::map<std::pair<unsigned, unsigned>, unsigned> words;
  std::optional<std::pair<unsigned, unsigned>> pending;
  std::uint64_t high_before = 0;
  unsigned high = 0;
  unsigned low = 0;
};

struct Interrupts {
  void observe(CpuState before, CpuStep result, const Hc12 &cpu, unsigned opcode);
  std::optional<CpuState> foreground;
  unsigned entries = 0;
  unsigned returns = 0;
  unsigned reused = 0;
};

// These are explicit observations, not a measured board profile or DSP peer.
void attach(McuMachine &machine, LcdFixture &display, FlashPhases &phases, std::ostream &trace);
void detach(McuMachine &machine);

struct PrefixCheckpoint {
  McuCheckpoint machine;
  LcdFixture display;
};

[[nodiscard]] PrefixCheckpoint
run_prefix(McuMachine &machine, const std::filesystem::path &path, std::ostream &trace);
void run_continuations(McuMachine &machine,
                       const PrefixCheckpoint &prefix,
                       const std::filesystem::path &path,
                       std::ostream &trace);
void verify_midi_reference_replays(McuMachine &machine,
                                   const LcdFixture &display,
                                   const std::filesystem::path &path,
                                   std::ostream &trace,
                                   unsigned level);
}
