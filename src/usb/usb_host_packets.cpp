// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_usb.hpp"
#include "usb/usb_host.hpp"
#include "usb/usb_transfer.hpp"

#include <algorithm>
#include <cstddef>
#include <span>

namespace blofeld {
void UsbHostQueue::input(McuBus &bus, Pending &pending) {
  const auto endpoint = bus.usb_host_endpoint(pending.request.address);
  const auto packet = bus.request_usb_in(endpoint.buffer);
  if (packet.response == UsbResponse::nak || packet.response == UsbResponse::none) { return; }
  if (packet.response == UsbResponse::stall) {
    pending.completion.status = UsbTransferStatus::stalled;
    pending.phase = Phase::complete;
    return;
  }
  const auto remaining = pending.request.length - pending.completion.transferred;
  if (packet.response != UsbResponse::data || packet.size > remaining ||
      packet.data1 != data1_[pending.request.address]) {
    throw Fault("Unqualified USB host IN response, packet overrun or DATA PID");
  }
  pending.completion.input.insert(pending.completion.input.end(),
                                  packet.data.begin(),
                                  packet.data.begin() + static_cast<std::ptrdiff_t>(packet.size));
  bus.finish_usb_in(true);
  data1_[pending.request.address] = !data1_[pending.request.address];
  pending.completion.transferred += packet.size;
  const bool short_packet = packet.size < endpoint.maximum_packet;
  if (short_packet || pending.completion.transferred == pending.request.length) {
    if (short_packet && pending.completion.transferred < pending.request.length &&
        pending.request.short_not_ok) {
      pending.completion.status = UsbTransferStatus::short_packet;
    }
    pending.phase = (pending.request.address & 15U) == 0 ? Phase::status : Phase::complete;
  }
}

void UsbHostQueue::output(McuBus &bus, Pending &pending) {
  const auto endpoint = bus.usb_host_endpoint(pending.request.address);
  if (!endpoint.data1) { throw Fault("Unknown USB OUT DATA PID after endpoint reset"); }
  if (*endpoint.data1 != data1_[pending.request.address]) {
    throw Fault(
        "USB host OUT DATA PID disagrees with device; host/controller ownership is unqualified");
  }
  const auto remaining = pending.request.length - pending.completion.transferred;
  const auto count = std::min(remaining, endpoint.maximum_packet);
  if (count == 0 && pending.request.length != 0 && !pending.extra_zero) {
    throw Fault("USB host OUT reached an unqualified extra zero packet");
  }
  const auto data =
      std::span(pending.request.output).subspan(pending.completion.transferred, count);
  const auto response = bus.receive_usb_out(
      {.buffer = endpoint.buffer, .data = data, .data1 = data1_[pending.request.address]});
  if (response == UsbResponse::nak || response == UsbResponse::none) { return; }
  if (response == UsbResponse::stall) {
    pending.completion.status = UsbTransferStatus::stalled;
    pending.phase = Phase::complete;
    return;
  }
  if (response != UsbResponse::ack) { throw Fault("Unexpected USB OUT handshake"); }
  data1_[pending.request.address] = !data1_[pending.request.address];
  pending.completion.transferred += count;
  if (pending.completion.transferred == pending.request.length) {
    if (count != 0 && pending.request.zero_packet && count == endpoint.maximum_packet) {
      pending.extra_zero = true;
      return;
    }
    pending.extra_zero = false;
    pending.phase = (pending.request.address & 15U) == 0 ? Phase::status : Phase::complete;
  }
}

void UsbHostQueue::data_packet(McuBus &bus, Pending &pending) {
  const auto endpoint = bus.usb_host_endpoint(pending.request.address);
  if ((pending.request.address & 15U) != 0 && endpoint.type != 2) {
    throw Fault("USB host queue currently supports only control and bulk endpoints");
  }
  constexpr unsigned input_bit = 0x80;
  if ((pending.request.address & input_bit) != 0) {
    input(bus, pending);
  } else {
    output(bus, pending);
  }
}

void UsbHostQueue::status_packet(McuBus &bus, Pending &pending) {
  constexpr unsigned input_bit = 0x80;
  const bool status_in = pending.request.length == 0 || (pending.request.address & input_bit) == 0;
  if (status_in) {
    const auto packet = bus.request_usb_in(UsbBuffer::control_in);
    if (packet.response == UsbResponse::nak || packet.response == UsbResponse::none) { return; }
    if (packet.response == UsbResponse::stall) {
      pending.completion.status = UsbTransferStatus::stalled;
    } else {
      if (packet.response != UsbResponse::data || packet.size != 0 || !packet.data1) {
        throw Fault("Unqualified USB control IN status packet");
      }
      bus.finish_usb_in(true);
    }
  } else {
    const auto endpoint = bus.usb_host_endpoint(0);
    if (!endpoint.data1 || !*endpoint.data1) {
      throw Fault("Unqualified USB control OUT status DATA PID");
    }
    const auto response =
        bus.receive_usb_out({.buffer = UsbBuffer::control_out, .data = {}, .data1 = true});
    if (response == UsbResponse::nak || response == UsbResponse::none) { return; }
    if (response == UsbResponse::stall) {
      pending.completion.status = UsbTransferStatus::stalled;
    } else if (response != UsbResponse::ack) {
      throw Fault("Unexpected USB control OUT status handshake");
    }
  }
  pending.phase = Phase::complete;
}
}
