// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/usbip_session.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usb_transfer.hpp"
#include "blofeld/usbip.hpp"
#include "blofeld/usbip_export.hpp"
#include "usbip_management_detail.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <utility>
#include <variant>

namespace blofeld {
namespace {
UsbipManagementReply absent(const UsbipManagementRequest &request) {
  using namespace usbip_management;
  if (std::holds_alternative<UsbipListRequest>(request)) {
    auto bytes = header(list, 0);
    append(bytes, 0, Width::longword);
    return {.bytes = std::move(bytes)};
  }
  return {.bytes = header(import, 1)};
}
}

bool UsbipSession::wants_input() const {
  constexpr std::size_t reply_reserve = 128; // More than a chunk of minimum-size commands.
  constexpr auto byte_reserve = usbip_input_chunk * 2;
  return !input_ended_ && (state_ == State::management || state_ == State::transfers) &&
         output_.size() < maximum_usbip_replies - reply_reserve &&
         output_bytes_ <= maximum_usbip_output - byte_reserve;
}

bool UsbipSession::closed() const {
  return state_ == State::closed || state_ == State::faulted;
}

void UsbipSession::manage(const std::shared_ptr<UsbipExport> &publication) {
  const auto &request = management_.request();
  if (!request) { throw Fault("Incomplete USB/IP management request reached dispatch"); }
  auto reply = publication ? publication->reply(*request) : absent(*request);
  management_.finish();
  if (reply.imported) {
    if (!publication) { throw Fault("USB/IP import lacks a captured device publication"); }
    bound_ = publication;
    imported_ = true;
    decoder_.emplace(bound_->device_id());
    state_ = State::transfers;
  } else {
    state_ = State::draining;
  }
  append({.bytes = std::move(reply.bytes), .sequence = std::nullopt});
}

void UsbipSession::command(UsbipCommand value) {
  const auto sequence = std::visit([](const auto &entry) { return entry.sequence; }, value);
  if (!live_.insert(sequence).second) { throw Fault("Duplicate live USB/IP sequence"); }
  if (auto *request = std::get_if<UsbTransferRequest>(&value)) {
    // Linux USB/IP also uses a hub-class SET_FEATURE(PORT_RESET) proxy for
    // remote device reset. It is not a class request to the Blofeld firmware.
    constexpr unsigned port_request_type = 0x23;
    constexpr unsigned set_feature = 3;
    constexpr unsigned port_reset = 4;
    if (request->address == 0 && request->setup[0] == port_request_type &&
        request->setup[1] == set_feature && request->setup[2] == port_reset &&
        request->setup[3] == 0) {
      throw Fault("Unqualified USB/IP remote device reset; board-owned reset required");
    }
    queue_.submit(std::move(*request));
  } else {
    const auto unlink = std::get<UsbUnlinkRequest>(value);
    const auto cancelled = queue_.cancel(unlink.target);
    if (cancelled) { live_.erase(unlink.target); }
    append({.bytes = encode_usbip_unlink(unlink.sequence, cancelled), .sequence = sequence});
  }
}

UsbipInputProgress UsbipSession::receive(const std::span<const std::uint8_t> bytes,
                                         const std::shared_ptr<UsbipExport> &publication) {
  try {
    if (!wants_input() || bytes.size() > usbip_input_chunk) {
      throw Fault("USB/IP input exceeds connection state/backpressure/chunk limits");
    }
    UsbipInputProgress progress;
    auto remaining = bytes;
    if (state_ == State::management) {
      remaining = remaining.subspan(management_.consume(remaining));
      if (management_.request()) {
        manage(publication);
        progress.completed_frame = true;
      }
    }
    if (!remaining.empty()) {
      if (state_ != State::transfers || !decoder_) {
        throw Fault("Trailing USB/IP bytes after a closing management operation");
      }
      auto &decoder = *decoder_;
      while (!remaining.empty()) {
        const auto count = std::min(remaining.size(), decoder.input_space());
        if (count == 0) { throw Fault("USB/IP frame buffer cannot make progress"); }
        decoder.feed(remaining.first(count));
        remaining = remaining.subspan(count);
        while (auto value = decoder.next()) {
          command(std::move(*value));
          progress.completed_frame = true;
        }
      }
    }
    constexpr auto maximum_frame = maximum_usb_transfer + usbip_transfer_header;
    progress.frame_pending =
        state_ == State::management ||
        (state_ == State::transfers && decoder_->input_space() != maximum_frame);
    return progress;
  } catch (const Fault &) {
    fail();
    throw;
  }
}

void UsbipSession::poll(McuBus &bus, const ExecutionContext context) {
  if (faulted()) { throw Fault("Faulted USB/IP session cannot service the device"); }
  try {
    // Reserve a complete bounded completion batch. Unsent replies exert real
    // transport backpressure; no additional device packet is ACKed meanwhile.
    if (state_ == State::transfers && output_.empty()) {
      for (const auto &completion : queue_.poll(bus, context)) {
        append({.bytes = encode_usbip_completion(completion), .sequence = completion.sequence});
      }
    }
    finish_if_drained();
  } catch (const Fault &) {
    fail();
    throw;
  }
}
}
