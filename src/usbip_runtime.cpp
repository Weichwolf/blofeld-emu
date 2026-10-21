// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/usbip_runtime.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/usbip_export.hpp"
#include "blofeld/usbip_tcp.hpp"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {
UsbipRuntime::UsbipRuntime(UsbipRuntimeOptions options) : options_(std::move(options)) {
  if (options_.period_ticks == 0 || options_.probe_timeout_ticks == 0) {
    throw Fault("USB/IP runtime requires positive poll period and descriptor probe timeout");
  }
  server_.emplace(options_.tcp);
  port_ = server_->port();
}

void UsbipRuntime::retire() {
  server_.reset();
  publication_.reset();
  faulted_ = true;
}

void UsbipRuntime::validate_boundary(const UsbipRuntimeBoundary &boundary) {
  if (!boundary.owner) { throw Fault("USB/IP runtime requires a board owner identity"); }
  if (previous_ && (boundary.owner != previous_->owner ||
                    boundary.restore_revision != previous_->restore_revision ||
                    boundary.ticks < previous_->ticks)) {
    throw Fault("USB/IP board owner/restore/time changed; a new transport session is required");
  }
  previous_ = boundary;
}

void UsbipRuntime::validate_evidence(const McuBus &bus) const {
  const auto now = bus.usb_host_evidence();
  if (now.link != evidence_.link) {
    throw Fault("USB/IP link/reset evidence expired; board-owned recovery is required");
  }
  // After import the same host owns legitimate configuration/interface changes.
  // Its exclusive lease stays retired on close, so no stale metadata is re-used.
  if (!publication_ || publication_->available()) {
    if (now.descriptions != evidence_.descriptions) {
      throw Fault("USB/IP endpoint description evidence expired");
    }
    if (publication_ && bus.usb_host_configuration() != configuration_) {
      throw Fault("USB/IP active configuration evidence expired before import");
    }
  }
}

bool UsbipRuntime::due(const std::uint64_t ticks) {
  if (ticks < options_.after_ticks || ticks < next_) { return false; }
  if (ticks > std::numeric_limits<std::uint64_t>::max() - options_.period_ticks) {
    throw Fault("USB/IP poll timestamp exhausted");
  }
  next_ = ticks + options_.period_ticks;
  return true;
}

void UsbipRuntime::probe(McuBus &bus, const ExecutionContext context, const std::uint64_t ticks) {
  if (!server_) { throw Fault("USB/IP probe requires its active listener"); }
  if (!admitted_) {
    admitted_ = ticks;
    evidence_ = bus.usb_host_evidence();
  }
  if (publication_ || bus.usb_host_suspended()) { return; }
  if (probe_.poll(bus, context)) {
    configuration_ = bus.usb_host_configuration();
    publication_ =
        std::make_shared<UsbipExport>(probe_.descriptors(), options_.location, bus, context);
    server_->publish(publication_);
  }
}

std::vector<UsbipConnectionEvent> UsbipRuntime::poll(McuBus &bus,
                                                     const ExecutionContext context,
                                                     const UsbipRuntimeBoundary &boundary) {
  if (faulted_ || !server_) { throw Fault("Retired USB/IP runtime requires a new adapter"); }
  try {
    validate_boundary(boundary);
    // Check every board boundary, even between scheduled transport polls. A
    // reset pulse must not disappear because its final level returned to normal.
    if (admitted_) { validate_evidence(bus); }
    if (admitted_ && !publication_ && boundary.ticks - *admitted_ >= options_.probe_timeout_ticks) {
      throw Fault("USB/IP descriptor probe exceeded its reference tick budget");
    }
    if (!due(boundary.ticks)) { return {}; }
    probe(bus, context, boundary.ticks);
    auto events = server_->poll(bus, context);
    for (const auto &event : events) {
      if (event.requires_board_reset) {
        throw Fault("USB/IP imported connection=" + std::to_string(event.connection) +
                    " retired; board-owned recovery required: " + event.reason);
      }
    }
    return events;
  } catch (const Fault &fault) {
    retire();
    throw Fault("USB/IP runtime tick=" + std::to_string(boundary.ticks) +
                " mcu-pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) + ": " + fault.what());
  } catch (...) {
    retire();
    throw;
  }
}
}
