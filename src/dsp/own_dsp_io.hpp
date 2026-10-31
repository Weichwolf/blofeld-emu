// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <exception>
#include <optional>

namespace blofeld {
// Converts synchronous core accesses to the strict board bus. This does not
// schedule clocks, DMA, interrupts or a complete device.
class OwnDspIo final : public dsp56300::Io {
public:
  enum class Operation : std::uint8_t { read, write, bit, reset };

  struct Failure {
    Operation operation;
    dsp56300::Space space;
    dsp56300::Word address, pc, value;
    dsp56300::BitAction action;
    std::exception_ptr cause;
  };

  explicit OwnDspIo(DspBus &bus) noexcept : bus_(bus) {}

  [[nodiscard]] const std::optional<Failure> &failure() const noexcept { return failure_; }

  void clear_failure() noexcept { failure_.reset(); }

  bool read(dsp56300::Space space,
            dsp56300::Word address,
            dsp56300::Word pc,
            dsp56300::Word &value) noexcept override;
  bool write(dsp56300::Space space,
             dsp56300::Word address,
             dsp56300::Word pc,
             dsp56300::Word value) noexcept override;
  bool bit(dsp56300::Space space,
           dsp56300::Word address,
           dsp56300::Word pc,
           dsp56300::Word bit,
           dsp56300::BitAction action,
           bool &previous) noexcept override;
  bool reset_peripherals(dsp56300::Word pc) noexcept override;

private:
  friend class OwnDspMachine;
  bool fail(Operation operation,
            dsp56300::Space space,
            dsp56300::Word address,
            dsp56300::Word pc,
            dsp56300::Word value = 0,
            dsp56300::BitAction action = dsp56300::BitAction::test) noexcept;

  DspBus &bus_;
  std::optional<Failure> failure_;
};
} // namespace blofeld
