// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "board/hcs08_board_memory.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_adc.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace blofeld {
enum class Hcs08BoardContract : std::uint8_t { unqualified, candidate_gt8a_reference_retirement };
enum class Hcs08BoardPhase : std::uint8_t { boundary, devices, cpu, bdc };

struct Hcs08BoardInputs {
  std::optional<Hcs08AdcReference> adc_reference;
  std::array<std::optional<std::uint32_t>, hcs08_adc::channels> analog{};
  std::optional<unsigned> timer_phase;
};

struct Hcs08BoardState {
  std::uint64_t attempted_periods = 0, completed_periods = 0;
  Hcs08BoardPhase phase = Hcs08BoardPhase::boundary;
  bool operator==(const Hcs08BoardState &) const = default;
};

class Hcs08BoardCheckpoint {
public:
  Hcs08BoardCheckpoint(const Hcs08BoardCheckpoint &) = default;
  Hcs08BoardCheckpoint &operator=(const Hcs08BoardCheckpoint &) = default;

private:
  friend class Hcs08Board;
  Hcs08BoardCheckpoint(Hcs08BoardMemory memory,
                       const Hcs08Checkpoint &cpu,
                       const Hcs08BdcCheckpoint &bdc,
                       const Hcs08RunCheckpoint &clock,
                       Hcs08BoardState state);
  Hcs08BoardMemory memory_;
  Hcs08Checkpoint cpu_;
  Hcs08BdcCheckpoint bdc_;
  Hcs08RunCheckpoint clock_;
  Hcs08BoardState state_;
};

struct Hcs08BoardResetRequest {
  Hcs08ResetCause cause = Hcs08ResetCause::unqualified;
  Hcs08ResetEntry entry = Hcs08ResetEntry::unqualified;
  std::optional<std::uint8_t> reset_sources = std::nullopt;
  Hcs08FlashAbort retention = Hcs08FlashAbort::unqualified_retention;
};

// A new logical reset boundary,not a resumed instruction or physical pin sequence.
// Warm entry retains only a healthy CPU's architectural A/X knowledge;power-on
// discards it. NVM uncertainty and detached pad obligations belong to the copy.
class Hcs08BoardResetCheckpoint {
public:
  Hcs08BoardResetCheckpoint(const Hcs08BoardResetCheckpoint &) = default;
  Hcs08BoardResetCheckpoint &operator=(const Hcs08BoardResetCheckpoint &) = default;

private:
  friend class Hcs08Board;
  Hcs08BoardResetCheckpoint(Hcs08BoardMemory memory,
                            std::optional<Hcs08Checkpoint> cpu,
                            Hcs08BoardResetRequest request);
  Hcs08BoardMemory memory_;
  std::optional<Hcs08Checkpoint> cpu_;
  Hcs08BoardResetRequest request_;
};

// Native CPU,actual target memory and BDC share one reference-period owner.
// No host synthesizer,entry-PC override,clock calibration or reset duration is
// supplied. Each caller-provided period advances devices before CPU retirement.
class Hcs08Board {
public:
  Hcs08Board(const Hcs08FlashImage &image,
             const std::filesystem::path &specification,
             Hcs08BoardContract contract,
             const Hcs08BoardInputs &inputs);
  Hcs08Board(const Hcs08BoardCheckpoint &checkpoint, const std::filesystem::path &specification);
  Hcs08Board(const Hcs08BoardResetCheckpoint &checkpoint,
             const std::filesystem::path &specification,
             const Hcs08BoardInputs &inputs);
  Hcs08Board(const Hcs08Board &) = delete;
  Hcs08Board &operator=(const Hcs08Board &) = delete;
  void attach_pad_source(Hcs08GpioPort port, GpioPadReader reader);
  void advance_reference_period();
  void receive_bdc(std::uint8_t byte);
  // Explicit byte-service boundary only;no pin encoding or physical grant is supplied.
  void service_bdc(Hcs08BdcGrant grant);
  [[nodiscard]] std::uint8_t read_bdc_reply();
  [[nodiscard]] Hcs08BoardCheckpoint checkpoint() const;
  [[nodiscard]] Hcs08BoardResetCheckpoint
  reset_checkpoint(const Hcs08BoardResetRequest &request) const;
  [[nodiscard]] Hcs08BoardState state() const;
  [[nodiscard]] Hcs08State cpu_state() const;
  [[nodiscard]] Hcs08RunState run_state() const;
  [[nodiscard]] Hcs08BdcState bdc_state() const;
  [[nodiscard]] const Hcs08BoardMemory &memory() const;
  [[nodiscard]] bool faulted() const;

private:
  void healthy(const std::string &operation) const;
  [[noreturn]] void fail(const std::string &operation, const std::string &reason) const;
  Hcs08BoardMemory memory_;
  Hcs08 cpu_;
  Hcs08Bdc bdc_;
  Hcs08RunClock clock_;
  Hcs08BoardState state_;
  bool healthy_ = true;
};
} // namespace blofeld
