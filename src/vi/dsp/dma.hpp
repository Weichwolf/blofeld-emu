// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "execution.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/memory.hpp>
#include <optional>

namespace blofeld::vi::dsp {
// DSP56300FM 10: transaction-driven DMA. One DSP execution owner; no bus clocks.
class Dma {
public:
  struct Write {
    dsp56300::Word address = 0, value = 0;
  };

  struct Address {
    dsp56300::Space space = dsp56300::Space::x;
    dsp56300::Word address = 0;
  };

  Dma() noexcept { reset(); }

  void reset() noexcept;
  [[nodiscard]] static bool contains(dsp56300::Word address) noexcept;
  [[nodiscard]] bool read(dsp56300::Word address, dsp56300::Word &value) const noexcept;
  [[nodiscard]] bool write(Write access) noexcept;
  [[nodiscard]] bool trigger(unsigned source) noexcept;

  [[nodiscard]] bool pending() const noexcept { return pending_ != 0; }

  [[nodiscard]] unsigned interrupts() const noexcept { return interrupts_; }

  [[nodiscard]] const std::optional<Failure> &failure() const noexcept { return failure_; }

  // Ordinary RAM is accessed directly by the consumer. Peripheral operands use
  // their real data-register contracts. Failed transfers never advance pointers.
  template <typename Read, typename Store>
  [[nodiscard]] bool service(const dsp56300::Word pc, Read &&read, Store &&store) {
    if (failure_) { return false; }
    while (pending()) {
      const auto channel = select();
      const auto plan = prepare(channel);
      if (!plan) {
        if (failure_) { failure_->pc = pc; }
        return false;
      }
      active_ = channel;
      done_ &= ~(1U << channel);
      dsp56300::Word value = 0;
      if (!read(plan->source, value)) {
        return fail_operand(
            plan->source, {.operation = Access::read, .pc = pc}, "Unknown DMA source data");
      }
      if (!store(plan->destination, value)) {
        return fail_operand(plan->destination,
                            {.operation = Access::write, .pc = pc, .value = value},
                            "DMA destination rejected data");
      }
      active_.reset();
      advance(channel, *plan);
    }
    return true;
  }

private:
  struct Channel {
    dsp56300::Word source = 0, destination = 0, count = 0, reload = 0, control = 0;
    unsigned known = 0;
  };

  struct Plan {
    Address source, destination;
    dsp56300::Word next_source = 0, next_destination = 0, next_count = 0;
    bool line_end = false, block_end = false;
  };

  [[nodiscard]] unsigned select() const noexcept;
  [[nodiscard]] std::optional<Plan> prepare(unsigned index) noexcept;
  void advance(unsigned index, const Plan &plan) noexcept;
  [[nodiscard]] bool fail_operand(Address address, Failure context, const char *reason) noexcept;
  [[nodiscard]] bool fail_register(dsp56300::Word address, const char *reason) noexcept;
  [[nodiscard]] std::optional<dsp56300::Word> increment(unsigned mode,
                                                        const Channel &channel,
                                                        unsigned low_bits,
                                                        unsigned middle_bits) noexcept;
  std::array<Channel, 6> channels_{};
  std::array<std::optional<dsp56300::Word>, 4> offsets_;
  std::array<unsigned, 4> last_{5, 5, 5, 5};
  std::array<bool, 4> last_valid_{};
  std::optional<unsigned> active_;
  std::optional<Failure> failure_;
  unsigned pending_ = 0, done_ = 0, interrupts_ = 0;
};
}
