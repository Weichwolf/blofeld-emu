// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_pll.hpp"
#include "blofeld/dsp_shi.hpp"
#include "blofeld/dsp_timers.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld {

enum class DspSpace : std::uint8_t { p, x, y };

struct DspAddress {
  DspSpace space = DspSpace::p;
  std::uint32_t address = 0;
};

struct DspStore {
  DspSpace space = DspSpace::p;
  std::uint32_t address = 0;
  std::uint32_t value = 0;
};

enum class DspBitOperation : std::uint8_t { test, clear, set, change };

struct DspBitAccess {
  DspSpace space = DspSpace::x;
  std::uint32_t address = 0;
  unsigned bit = 0;
  DspBitOperation operation = DspBitOperation::test;
};

struct DspRom {
  DspSpace space = DspSpace::p;
  std::uint32_t start = 0;
  std::span<const std::uint32_t> words;
};

struct DspCodeChanges {
  bool all = false;
  std::vector<std::uint32_t> addresses;
};

class DspBus {
public:
  DspBus();
  void reset();
  void reset_peripherals();
  [[nodiscard]] std::uint32_t read(DspAddress access);
  void write(DspStore access);
  [[nodiscard]] std::optional<bool> access_peripheral_bit(DspBitAccess access);
  [[nodiscard]] std::uint32_t peek_program(std::uint32_t address) const;
  void install_rom(DspRom image);
  void observe_omr(std::uint32_t value);
  void advance_instruction_cycles(unsigned cycles);
  [[nodiscard]] DspCodeChanges take_code_changes();

  [[nodiscard]] DspShi &shi() { return shi_; }

  [[nodiscard]] DspPll &pll() { return pll_; }

  [[nodiscard]] DspGpio &gpio(DspPort port);

  [[nodiscard]] DspTimers &timers() { return timers_; }

  [[nodiscard]] std::optional<std::uint32_t> once_latch() const { return once_latch_; }

private:
  enum class Operation : std::uint8_t { read, write };

  struct Memory {
    std::vector<std::uint32_t> words;
    std::vector<std::uint8_t> known;
  };

  struct Location {
    std::size_t bank = 0;
    std::size_t offset = 0;
    bool rom = false;
  };

  [[nodiscard]] Location locate(DspAddress access, Operation operation = Operation::read) const;
  [[nodiscard]] Location locate_program(std::uint32_t address, Operation operation) const;
  [[nodiscard]] Location locate_data(DspAddress access, Operation operation) const;
  [[nodiscard]] std::uint32_t read_peripheral(DspAddress access);
  void write_peripheral(DspStore access);
  std::array<Memory, 3> ram_;
  std::array<Memory, 3> rom_;
  DspShi shi_;
  DspPll pll_;
  DspTimers timers_;
  std::array<DspGpio, 3> gpio_{DspGpio(DspPort::c), DspGpio(DspPort::e), DspGpio(DspPort::f)};
  std::uint32_t bus_control_ = 0;
  DspCodeChanges changes_;
  bool switched_ = false;
  bool requested_switch_ = false;
  unsigned switch_delay_ = 0;
  std::optional<std::uint32_t> once_latch_;
};

}
