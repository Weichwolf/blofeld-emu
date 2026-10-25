// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld {

struct SpiInputs {
  bool selected = false; // Active-low SS, expressed as logical selection.
  bool clock = false;
  std::optional<bool> mosi = std::nullopt;
};

struct ShiOutput {
  bool driven = false;
  std::optional<bool> level; // An active but uninitialized shift output remains unknown.
};

struct ShiOutputs {
  ShiOutput miso;
  ShiOutput request; // Physical HREQ level, active low.
};

struct DspWordWrite {
  std::uint32_t address = 0;
  std::uint32_t value = 0;
};

// DSP56371 SHI SPI slave, driven by explicit pin events and DSP instruction cycles.
// No host word-injection shortcut, implicit clock or bootstrap replacement.
class DspShi {
public:
  DspShi();
  void reset();
  [[nodiscard]] std::uint32_t read(std::uint32_t address);
  void write(DspWordWrite access);
  void observe_spi(SpiInputs pins);
  void advance_instruction_cycles(unsigned cycles);
  [[nodiscard]] ShiOutputs outputs() const;
  [[nodiscard]] std::vector<std::uint32_t> interrupt_vectors() const;

private:
  [[nodiscard]] unsigned width() const;
  [[nodiscard]] bool phase() const;
  [[nodiscard]] bool polarity() const;
  [[nodiscard]] std::uint32_t status() const;
  void write_control(std::uint32_t value);
  void clear_paths();
  void load_transmit();
  void begin_word();
  void capture_bit(bool value);
  void finish_word();
  void set_miso();
  void validate_pins(const SpiInputs &pins) const;
  void clock_edge(const SpiInputs &pins);
  std::uint32_t clock_control_ = 1;
  std::uint32_t control_ = 0;
  std::uint32_t slave_address_ = 0;
  std::uint32_t errors_ = 0;
  std::uint32_t clear_armed_ = 0;
  std::uint32_t interrupt_enable_ = 0;
  bool interrupt_delay_ = false;
  bool reset_pending_ = false;
  bool active_ = false;
  bool reset_configuration_changed_ = false;
  std::optional<SpiInputs> pins_;
  std::optional<std::uint32_t> transmit_;
  std::uint32_t shift_ = 0;
  std::uint32_t shift_known_ = 0;
  std::array<std::uint32_t, 10> receive_{};
  std::size_t receive_head_ = 0;
  std::size_t receive_count_ = 0;
  unsigned captured_ = 0;
  bool word_started_ = false;
  bool first_clock_ = false;
  bool complete_ = false;
  bool transmit_fresh_ = false;
  std::optional<bool> miso_;
};

}
