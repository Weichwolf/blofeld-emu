// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usb_device.hpp"
#include "blofeld/usb_transfer.hpp"
#include "usb_descriptor_detail.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace blofeld {
UsbTransferRequest UsbDeviceProbe::request() const {
  using namespace usb_descriptor;
  constexpr unsigned input_address = 0x80;
  constexpr unsigned get_descriptor = 6;
  constexpr unsigned byte_mask = 0xff;
  unsigned length = device_bytes;
  unsigned type = device_type;
  if (stage_ == Stage::header || stage_ == Stage::configuration) {
    type = configuration_type;
    length = stage_ == Stage::header ? configuration_header_bytes : configuration_length(header_);
  } else if (stage_ != Stage::device) {
    throw Fault("Completed USB descriptor probe has no next request");
  }
  return {.sequence = static_cast<unsigned>(stage_) + 1,
          .address = input_address,
          .length = length,
          .setup = {input_address,
                    get_descriptor,
                    0,
                    static_cast<std::uint8_t>(type),
                    0,
                    0,
                    static_cast<std::uint8_t>(length & byte_mask),
                    static_cast<std::uint8_t>(length >> 8U)},
          .output = {},
          .short_not_ok = true,
          .zero_packet = false};
}

void UsbDeviceProbe::complete(const UsbTransferCompletion &completion,
                              const unsigned maximum_packet) {
  const auto expected = request();
  if (completion.sequence != expected.sequence || completion.status != UsbTransferStatus::success ||
      !completion.input_direction || completion.transferred != expected.length ||
      completion.input.size() != expected.length) {
    throw Fault("USB descriptor probe requires a complete successful control transfer");
  }
  if (stage_ == Stage::device) {
    usb_descriptor::validate_device(completion.input);
    if (completion.input[7] != maximum_packet) {
      throw Fault("USB descriptor/controller default packet sizes disagree");
    }
    device_ = completion.input;
    stage_ = Stage::header;
  } else if (stage_ == Stage::header) {
    static_cast<void>(usb_descriptor::configuration_length(completion.input));
    header_ = completion.input;
    stage_ = Stage::configuration;
  } else {
    if (!std::ranges::equal(header_, std::span(completion.input).first(header_.size()))) {
      throw Fault("USB configuration changed between header and complete transfer");
    }
    descriptors_ = UsbDeviceDescriptors(device_, completion.input);
    stage_ = Stage::ready;
  }
  submitted_ = false;
}

bool UsbDeviceProbe::poll(McuBus &bus, const ExecutionContext context) {
  if (faulted_) { throw Fault("Faulted USB descriptor probe requires a new reset/session"); }
  if (ready()) { return true; }
  try {
    if (!submitted_) {
      queue_.submit(request());
      submitted_ = true;
    }
    const auto completed = queue_.poll(bus, context);
    if (!completed.empty()) {
      complete(completed.front(), bus.usb_host_endpoint(0x80).maximum_packet);
    }
    return ready();
  } catch (const Fault &fault) {
    faulted_ = true;
    descriptors_.reset();
    constexpr std::array<std::string_view, 4> stages{"device", "header", "configuration", "ready"};
    throw Fault(
        "USB descriptor probe stage=" + std::string(stages.at(static_cast<unsigned>(stage_))) +
        " mcu-pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) + ": " + fault.what());
  }
}

const UsbDeviceDescriptors &UsbDeviceProbe::descriptors() const {
  if (faulted_ || !descriptors_) { throw Fault("USB descriptor capture is unavailable"); }
  return *descriptors_;
}

void UsbDeviceProbe::reset() {
  queue_.reset();
  stage_ = Stage::device;
  submitted_ = false;
  faulted_ = false;
  device_.clear();
  header_.clear();
  descriptors_.reset();
}
}
