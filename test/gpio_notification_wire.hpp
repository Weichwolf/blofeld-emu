// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "dsp_norm_fixture.hpp"

#include <cstdint>
#include <ostream>
#include <vector>

namespace blofeld::test::gpio_notification {
// Independent ideal 31250-baud 8-N-1 sink. It only observes actual PR3 pads.
class Wire {
public:
  explicit Wire(std::ostream &trace) : trace_(trace) {}

  void sample(McuBus &bus, const std::uint64_t cycle) {
    constexpr unsigned tx = 8;
    const auto pads = bus.observe_pads(McuPort::r, tx);
    require(pads.known == tx && pads.conflicts == 0, "Actual SCI TX pad must be known");
    const bool high = (pads.levels & tx) != 0;
    if (high != previous_) { trace_ << "TX_EDGE cycle=" << cycle << " high=" << high << '\n'; }
    if (!active_ && previous_ && !high) {
      start_ = cycle;
      active_ = true;
      slot_ = 0;
      value_ = 0;
    }
    if (active_ && cycle == start_ + half_bit + (std::uint64_t{slot_} * bit_cycles)) {
      if (slot_ == 0) {
        require(!high, "Independent serial sink must sample a low start bit");
      } else if (slot_ <= data_bits) {
        value_ |= static_cast<unsigned>(high) << (slot_ - 1);
      } else {
        require(high, "Independent serial sink must sample a high stop bit");
        trace_ << "TX_BYTE index=" << bytes_.size() << " value=" << value_ << " start=" << start_
               << " stop=" << cycle << '\n';
        bytes_.push_back(value_);
        active_ = false;
      }
      ++slot_;
    }
    previous_ = high;
  }

  [[nodiscard]] const std::vector<unsigned> &bytes() const { return bytes_; }

  [[nodiscard]] bool active() const { return active_; }

private:
  static constexpr unsigned bit_cycles = 960, half_bit = 480, data_bits = 8;
  std::ostream &trace_;
  std::vector<unsigned> bytes_;
  std::uint64_t start_ = 0;
  unsigned slot_ = 0, value_ = 0;
  bool previous_ = true, active_ = false;
};
} // namespace blofeld::test::gpio_notification
