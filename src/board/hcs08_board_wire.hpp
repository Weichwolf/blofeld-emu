// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_bdc_wire.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

namespace blofeld {
struct Hcs08BoardWireInputs {
  Hcs08BdcWireContract contract = Hcs08BdcWireContract::unqualified;
  Hcs08BdcDrive initial_host = Hcs08BdcDrive::unknown;
  Hcs08BdcWireClock clock;
};

class Hcs08BoardWireCheckpoint {
public:
  Hcs08BoardWireCheckpoint(const Hcs08BoardWireCheckpoint &) = default;
  Hcs08BoardWireCheckpoint &operator=(const Hcs08BoardWireCheckpoint &) = default;

private:
  friend class Hcs08BoardWire;
  Hcs08BoardWireCheckpoint(const Hcs08BoardCheckpoint &board,
                           const Hcs08BdcWireCheckpoint &wire,
                           Hcs08BdcWireClock clock);
  Hcs08BoardCheckpoint board_;
  Hcs08BdcWireCheckpoint wire_;
  Hcs08BdcWireClock clock_;
};

// One native CPU/memory/BDC/wire owner. The caller supplies wire clock edges and
// separate BUSCLK periods;their ratio,reset pin sequence and arbitration are not
// inferred. There is no direct byte-command bypass through this interface.
class Hcs08BoardWire {
public:
  Hcs08BoardWire(const Hcs08BoardResetCheckpoint &reset,
                 const std::filesystem::path &specification,
                 const Hcs08BoardInputs &board_inputs,
                 const Hcs08BoardWireInputs &wire_inputs);
  Hcs08BoardWire(const Hcs08BoardWireCheckpoint &checkpoint,
                 const std::filesystem::path &specification);
  Hcs08BoardWire(const Hcs08BoardWire &) = delete;
  Hcs08BoardWire &operator=(const Hcs08BoardWire &) = delete;
  void host_drive(Hcs08BdcDrive drive,
                  ExecutionContext context,
                  std::optional<std::uint64_t> time_ticks = std::nullopt);
  void clock_bdc();
  void advance_reference_period();
  void attach_pad_source(Hcs08GpioPort port, GpioPadReader reader);
  [[nodiscard]] bool level() const;
  [[nodiscard]] Hcs08BdcDrive target_drive() const;
  [[nodiscard]] Hcs08BoardState board_state() const;
  [[nodiscard]] Hcs08State cpu_state() const;
  [[nodiscard]] Hcs08RunState run_state() const;
  [[nodiscard]] Hcs08BdcState bdc_state() const;
  [[nodiscard]] Hcs08BdcWireState wire_state() const;
  [[nodiscard]] const Hcs08BoardMemory &memory() const;
  [[nodiscard]] bool faulted() const;
  [[nodiscard]] Hcs08BoardWireCheckpoint checkpoint() const;
  [[nodiscard]] Hcs08BoardResetCheckpoint
  reset_checkpoint(const Hcs08BoardResetRequest &request) const;

private:
  void healthy() const;
  [[noreturn]] void fail(const std::string &operation, const std::string &reason) const;
  void begin_wire();
  void end_wire();
  Hcs08Board board_;
  Hcs08BdcWire wire_;
  Hcs08BdcWireClock clock_;
  bool healthy_ = true;
};
} // namespace blofeld
