// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <optional>
#include <span>

namespace blofeld {
struct DspProgramObserver {
  void *context = nullptr;
  void (*completed)(void *context, unsigned address) = nullptr;
};

// Required wiring/protocol observation after a real peripheral access or RESET.
// No invented whole-word value is needed for a bit operation.
struct DspPeripheralObserver {
  void *context = nullptr;
  void (*preparing)(void *context, unsigned pc) = nullptr;
  void (*completed)(void *context, unsigned pc, bool reschedule) = nullptr;
};

// Converts synchronous core accesses to the strict board bus. This does not
// schedule clocks, DMA, interrupts or a complete device.
class DspIo final : public dsp56300::Io {
public:
  enum class Operation : std::uint8_t { read, write, bit, reset };

  struct Failure {
    Operation operation;
    dsp56300::Space space;
    dsp56300::Word address, pc, value;
    dsp56300::BitAction action;
    std::exception_ptr cause;
  };

  explicit DspIo(DspBus &bus) noexcept : bus_(bus) {}

  [[nodiscard]] std::optional<dsp56300::Word>
  inspect_program(const dsp56300::Word address) const noexcept override {
    return bus_.inspect_known_program(address);
  }

  [[nodiscard]] std::uint64_t program_revision() const noexcept override {
    return bus_.program_revision();
  }

  [[nodiscard]] std::span<const dsp56300::MemoryWindow>
  memory_windows(dsp56300::Space space) noexcept override;

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
  friend class DspMachine;
  friend class DspExecution;
  void prepare_peripheral(DspAddress target, unsigned pc) const;
  void observe_peripheral(DspAddress target, unsigned pc, bool reschedule = false) const;
  bool fail(Operation operation,
            dsp56300::Space space,
            dsp56300::Word address,
            dsp56300::Word pc,
            dsp56300::Word value = 0,
            dsp56300::BitAction action = dsp56300::BitAction::test) noexcept;

  DspBus &bus_;
  std::optional<Failure> failure_;
  DspProgramObserver program_observer_;
  std::array<std::array<dsp56300::MemoryWindow, 3>, 2> windows_{};
  DspPeripheralObserver peripheral_observer_;
};
} // namespace blofeld
