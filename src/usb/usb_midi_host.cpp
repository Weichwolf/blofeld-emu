// SPDX-License-Identifier: GPL-3.0-only
#include "usb/usb_midi_host.hpp"

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"
#include "usb/usb_descriptor_detail.hpp"
#include "usb/usb_transfer.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
constexpr unsigned input_bit = 0x80;
constexpr unsigned bulk_type = 2;
constexpr std::uint32_t input_sequence = 7;
constexpr unsigned event_bytes = 4;
}

UsbMidiHost::UsbMidiHost(const std::uint64_t frame_ticks, const std::uint64_t timeout_ticks)
    : frame_ticks_(frame_ticks), timeout_ticks_(timeout_ticks) {
  if (frame_ticks == 0 || frame_ticks > std::numeric_limits<std::uint64_t>::max() / 2 ||
      timeout_ticks == 0) {
    throw Fault("USB MIDI host requires a positive frame period and enumeration timeout");
  }
}

void UsbMidiHost::select_endpoints() {
  const auto bytes = probe_.descriptors().configuration();
  configuration_ = bytes[5];
  bool selected = false;
  bool found = false;
  for (std::size_t offset = 0; offset < bytes.size(); offset += bytes[offset]) {
    const auto descriptor = bytes.subspan(offset, bytes[offset]);
    constexpr unsigned interface_type = 4;
    constexpr unsigned endpoint_type = 5;
    if (descriptor[1] == interface_type) {
      selected =
          descriptor[3] == 0 && descriptor[5] == 1 && descriptor[6] == 3 && descriptor[7] == 0;
      if (selected) {
        if (found) { throw Fault("Ambiguous USB MIDI streaming interfaces"); }
        found = true;
        interface_ = descriptor[2];
      }
    } else if (selected && descriptor[1] == endpoint_type) {
      const auto address = descriptor[2];
      auto &endpoint = (address & input_bit) != 0 ? input_ : output_;
      if (endpoint.maximum_packet != 0 || (descriptor[3] & 3U) != bulk_type) {
        throw Fault("USB MIDI host requires one bulk endpoint per direction");
      }
      endpoint.address = address;
      endpoint.maximum_packet = usb_descriptor::little_word(descriptor, 4);
      endpoint.type = bulk_type;
      if (endpoint.maximum_packet % event_bytes != 0) {
        throw Fault("USB MIDI endpoint cannot hold complete four-byte event packets");
      }
    }
  }
  if (!found || input_.maximum_packet == 0 || output_.maximum_packet == 0) {
    throw Fault("USB descriptors contain no bidirectional default USB MIDI streaming interface");
  }
}

void UsbMidiHost::submit_control() {
  constexpr unsigned set_address = 5;
  constexpr unsigned set_configuration = 9;
  constexpr unsigned get_configuration = 8;
  UsbTransferRequest request{.sequence = static_cast<unsigned>(stage_), .output = {}};
  if (stage_ == Stage::address) {
    request.setup = {0, set_address, 1, 0, 0, 0, 0, 0};
  } else if (stage_ == Stage::configuration) {
    request.setup = {0, set_configuration, configuration_, 0, 0, 0, 0, 0};
  } else if (stage_ == Stage::readback) {
    request.address = input_bit;
    request.length = 1;
    request.setup = {input_bit, get_configuration, 0, 0, 0, 0, 1, 0};
    request.short_not_ok = true;
  } else {
    throw Fault("USB MIDI host has no pending control stage");
  }
  queue_.submit(std::move(request));
  submitted_ = true;
}

void UsbMidiHost::validate_configuration(const McuBus &bus) const {
  const auto selected = bus.usb_host_configuration();
  if (selected.configuration != configuration_ ||
      selected.interfaces != probe_.descriptors().interface_count() ||
      interface_ >= selected.interfaces || selected.alternatives.at(interface_) != 0) {
    throw Fault("USB MIDI descriptor/controller configuration mismatch");
  }
  for (const auto endpoint : {input_, output_}) {
    const auto actual = bus.usb_host_endpoint(endpoint.address);
    if (actual.maximum_packet != endpoint.maximum_packet || actual.type != bulk_type ||
        !actual.data1) {
      throw Fault("USB MIDI descriptor/controller endpoint mismatch or unknown DATA PID");
    }
  }
}

void UsbMidiHost::complete_control(McuBus &bus, const UsbTransferCompletion &completion) {
  if (completion.sequence != static_cast<unsigned>(stage_) ||
      completion.status != UsbTransferStatus::success) {
    throw Fault("USB MIDI enumeration control transfer did not complete successfully");
  }
  if (stage_ == Stage::address || stage_ == Stage::configuration) {
    if (completion.transferred != 0 || !completion.input.empty() || completion.input_direction) {
      throw Fault("USB MIDI enumeration received unexpected control data");
    }
    if (stage_ == Stage::address) {
      if (bus.usb_host_address() != 1) {
        throw Fault("USB SET_ADDRESS status did not commit address");
      }
      addressed_at_ = previous_;
      stage_ = Stage::configuration;
    } else {
      stage_ = Stage::readback;
    }
  } else if (stage_ == Stage::readback) {
    if (!completion.input_direction || completion.transferred != 1 ||
        completion.input != std::vector<std::uint8_t>{configuration_}) {
      throw Fault("USB GET_CONFIGURATION did not confirm the requested selection");
    }
    validate_configuration(bus);
    stage_ = Stage::ready;
  } else {
    throw Fault("Unexpected USB MIDI control completion");
  }
  submitted_ = false;
}

std::vector<UsbTransferCompletion>
UsbMidiHost::poll(McuBus &bus, const ExecutionContext context, const std::uint64_t ticks) {
  if (faulted_) { throw Fault("Faulted USB MIDI host requires a new session"); }
  try {
    if (!due(bus, ticks)) { return {}; }
    if (stage_ == Stage::descriptors) {
      if (probe_.poll(bus, context)) {
        select_endpoints();
        evidence_ = bus.usb_host_evidence();
        stage_ = Stage::address;
      }
      return {};
    }
    if (!ready()) {
      // USB2.0 9.2.6.3 gives the device two milliseconds after SET_ADDRESS status.
      if (stage_ == Stage::configuration && addressed_at_ &&
          ticks - *addressed_at_ < frame_ticks_ * 2) {
        return {};
      }
      if (!submitted_) { submit_control(); }
      const auto completions = queue_.poll(bus, context);
      if (!completions.empty()) { complete_control(bus, completions.front()); }
      return {};
    }
    if (!receiving_) {
      queue_.submit({.sequence = input_sequence,
                     .address = input_.address,
                     .length = input_.maximum_packet,
                     .output = {}});
      receiving_ = true;
    }
    auto completions = queue_.poll(bus, context);
    for (const auto &completion : completions) {
      if (completion.status != UsbTransferStatus::success ||
          completion.transferred % event_bytes != 0) {
        throw Fault("USB MIDI bulk transfer failed or split an event packet");
      }
      if (completion.sequence == input_sequence) { receiving_ = false; }
    }
    return completions;
  } catch (const Fault &fault) {
    faulted_ = true;
    throw Fault("USB MIDI host stage=" + std::to_string(static_cast<unsigned>(stage_)) + " tick=" +
                std::to_string(ticks) + " mcu-pc=" + hex(context.pc) + ": " + fault.what());
  }
}

bool UsbMidiHost::due(const McuBus &bus, const std::uint64_t ticks) {
  if (previous_ && ticks < *previous_) { throw Fault("USB MIDI host time moved backwards"); }
  previous_ = ticks;
  const auto evidence = bus.usb_host_evidence();
  if (!started_) {
    started_ = ticks;
    evidence_ = evidence;
  }
  if (evidence.link != evidence_.link ||
      (stage_ != Stage::descriptors && evidence.descriptions != evidence_.descriptions)) {
    throw Fault("USB MIDI host lost its link or endpoint descriptions");
  }
  if (!ready() && ticks - *started_ >= timeout_ticks_) {
    throw Fault("USB MIDI enumeration exceeded its device-time timeout");
  }
  const auto frame = ticks / frame_ticks_;
  if (frame_ == frame && !ready()) { return false; }
  frame_ = frame;
  return true;
}

std::uint32_t UsbMidiHost::send_packets(const std::span<const std::uint8_t> packets) {
  if (!ready() || packets.empty() || packets.size() % event_bytes != 0 ||
      packets.size() > maximum_usb_transfer ||
      sequence_ == std::numeric_limits<std::uint32_t>::max()) {
    throw Fault(
        "USB MIDI output requires a configured host, complete events and bounded transfers");
  }
  const auto sequence = sequence_;
  const auto reserved_input = receiving_ ? 0U : 1U;
  if (queue_.pending() >= maximum_usb_pending - reserved_input) {
    throw Fault("USB MIDI output queue is full");
  }
  queue_.submit({.sequence = sequence,
                 .address = output_.address,
                 .length = static_cast<std::uint32_t>(packets.size()),
                 .output = {packets.begin(), packets.end()}});
  ++sequence_;
  return sequence;
}
}
