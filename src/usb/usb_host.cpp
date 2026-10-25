// SPDX-License-Identifier: GPL-3.0-only
#include "usb/usb_host.hpp"

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_transfer.hpp"

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace blofeld {
void UsbHostQueue::submit(UsbTransferRequest request) {
  if (faulted_) { throw Fault("Faulted USB host queue requires a new reset/session"); }
  validate_usb_transfer(request);
  if (pending_.size() == maximum_usb_pending ||
      std::ranges::any_of(
          pending_, [&request](const auto &p) { return p.request.sequence == request.sequence; })) {
    throw Fault("Duplicate or excessive pending USB host transfers");
  }
  constexpr unsigned input_bit = 0x80;
  UsbTransferCompletion completion{.sequence = request.sequence,
                                   .status = UsbTransferStatus::success,
                                   .transferred = 0,
                                   .input = {},
                                   .input_direction = (request.address & input_bit) != 0};
  const auto phase = (request.address & 15U) == 0 ? Phase::setup : Phase::data;
  pending_.push_back({.request = std::move(request),
                      .completion = std::move(completion),
                      .phase = phase,
                      .extra_zero = false});
}

bool UsbHostQueue::cancel(const std::uint32_t sequence) {
  return std::erase_if(pending_,
                       [sequence](const auto &p) { return p.request.sequence == sequence; }) != 0;
}

void UsbHostQueue::reset() {
  pending_.clear();
  data1_.fill(false);
  faulted_ = false;
}

void UsbHostQueue::packet(McuBus &bus, Pending &pending) {
  if (pending.phase == Phase::setup) {
    const auto response = bus.receive_usb_setup(pending.request.setup);
    if (response == UsbResponse::nak || response == UsbResponse::none) { return; }
    if (response != UsbResponse::ack) { throw Fault("Unexpected USB SETUP handshake"); }
    constexpr unsigned input_address = 0x80;
    data1_[0] = true;
    data1_[input_address] = true;
    pending.phase = pending.request.length == 0 ? Phase::status : Phase::data;
  } else if (pending.phase == Phase::data) {
    data_packet(bus, pending);
  } else if (pending.phase == Phase::status) {
    status_packet(bus, pending);
  } else {
    throw Fault("Completed USB transfer remained in the pending queue");
  }
}

std::vector<UsbTransferCompletion> UsbHostQueue::poll(McuBus &bus, const ExecutionContext context) {
  if (faulted_) { throw Fault("Faulted USB host queue requires a new reset/session"); }
  std::vector<UsbTransferCompletion> result;
  std::bitset<256> visited;
  for (auto current = pending_.begin(); current != pending_.end();) {
    auto &pending = *current;
    const auto address = pending.request.address;
    const std::size_t key = (address & 15U) == 0 ? 0U : address;
    if (visited.test(key)) {
      ++current;
      continue;
    }
    visited.set(key);
    try {
      packet(bus, pending);
    } catch (const Fault &fault) {
      faulted_ = true;
      constexpr std::array<std::string_view, 4> phases{"setup", "data", "status", "complete"};
      throw Fault("USB host request seq=" + std::to_string(pending.request.sequence) +
                  " endpoint=" + hex(address, 2) +
                  " phase=" + std::string(phases.at(static_cast<unsigned>(pending.phase))) +
                  " mcu-pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) + ": " +
                  fault.what());
    }
    if (pending.phase == Phase::complete) {
      result.push_back(std::move(pending.completion));
      current = pending_.erase(current);
    } else {
      ++current;
    }
  }
  return result;
}
}
