// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "usb/usb_transfer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace blofeld {
class McuBus;

// One poll attempts at most one packet per logical endpoint. The board scheduler
// supplies actual MCU execution between polls; this queue never executes/patches firmware.
class UsbHostQueue {
public:
  void submit(UsbTransferRequest request);
  [[nodiscard]] bool cancel(std::uint32_t sequence);
  [[nodiscard]] std::vector<UsbTransferCompletion> poll(McuBus &bus, ExecutionContext context);
  // Host-side bus reset state only. The caller must separately deliver a qualified
  // controller reset/PHY observation, never infer an original guest's readiness.
  void reset();

  [[nodiscard]] std::size_t pending() const { return pending_.size(); }

  [[nodiscard]] bool faulted() const { return faulted_; }

private:
  enum class Phase : std::uint8_t { setup, data, status, complete };

  struct Pending {
    UsbTransferRequest request;
    UsbTransferCompletion completion;
    Phase phase = Phase::data;
    bool extra_zero = false;
  };

  void packet(McuBus &bus, Pending &pending);
  void data_packet(McuBus &bus, Pending &pending);
  static void status_packet(McuBus &bus, Pending &pending);
  void input(McuBus &bus, Pending &pending);
  void output(McuBus &bus, Pending &pending);
  std::deque<Pending> pending_;
  std::array<bool, 256> data1_{};
  bool faulted_ = false;
};
}
