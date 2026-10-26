// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/gpio_pads.hpp"
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_gpio.hpp"
#include "keyboard/hcs08_spi.hpp"
#include "keyboard/hcs08_spi_clock.hpp"
#include "keyboard/hcs08_spi_run_clock.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace blofeld {
struct Hcs08BoardSerialContract {
  Hcs08SpiRunContract order = Hcs08SpiRunContract::unqualified;
  Hcs08SpiLaunchContract launch = Hcs08SpiLaunchContract::unqualified;
  Hcs08PrefetchContract program = Hcs08PrefetchContract::unqualified;
};

class Hcs08BoardSerialCheckpoint {
public:
  Hcs08BoardSerialCheckpoint(const Hcs08BoardSerialCheckpoint &) = default;
  Hcs08BoardSerialCheckpoint &operator=(const Hcs08BoardSerialCheckpoint &) = default;

private:
  friend class Hcs08BoardSerial;
  Hcs08BoardSerialCheckpoint(const Hcs08BoardCheckpoint &board,
                             const Hcs08SpiClockCheckpoint &spi,
                             const Hcs08SpiRunCheckpoint &run,
                             Hcs08BoardSerialContract contract);
  Hcs08BoardCheckpoint board_;
  Hcs08SpiClockCheckpoint spi_;
  Hcs08SpiRunCheckpoint run_;
  Hcs08BoardSerialContract contract_;
};

// Transfers a trusted complete instruction boundary into native IRQ/prefetch
// ownership. Every period advances real COP/TPM/ADC first,then CPU/SPI in the
// declared reference order. No idle byte,status,clock calibration or IRQ is supplied.
class Hcs08BoardSerial {
public:
  Hcs08BoardSerial(const Hcs08BoardCheckpoint &board,
                   const std::filesystem::path &specification,
                   Hcs08BoardSerialContract contract);
  Hcs08BoardSerial(const Hcs08BoardSerialCheckpoint &checkpoint,
                   const std::filesystem::path &specification);
  Hcs08BoardSerial(const Hcs08BoardSerial &) = delete;
  Hcs08BoardSerial &operator=(const Hcs08BoardSerial &) = delete;
  [[nodiscard]] std::optional<Hcs08SpiPins> advance_reference_period(
      std::optional<bool> miso,
      Hcs08BoardClockObservation observation = Hcs08BoardClockObservation::unobserved);
  void attach_pad_source(Hcs08GpioPort port, GpioPadReader reader);
  [[nodiscard]] Hcs08BoardSerialCheckpoint checkpoint() const;
  [[nodiscard]] Hcs08BoardResetCheckpoint
  reset_checkpoint(const Hcs08BoardResetRequest &request) const;
  [[nodiscard]] Hcs08BoardState board_state() const;
  [[nodiscard]] Hcs08State cpu_state() const;
  [[nodiscard]] Hcs08SpiRunState run_state() const;
  [[nodiscard]] Hcs08SpiClockState clock_state() const;
  [[nodiscard]] const Hcs08BoardMemory &memory() const;
  [[nodiscard]] bool faulted() const;

private:
  [[nodiscard]] ExecutionContext context() const;
  void healthy() const;
  [[noreturn]] void fail(const std::string &reason) const;
  Hcs08Board board_;
  Hcs08SpiClock spi_;
  Hcs08SpiRunClock run_;
  Hcs08BoardSerialContract contract_;
  bool healthy_ = true;
};
} // namespace blofeld
