// SPDX-License-Identifier: GPL-3.0-only
#include "usb/usb_host_link.hpp"

#include "common/bus.hpp"
#include "mcu/mcu_bus.hpp"

#include <cstdint>
#include <limits>

namespace blofeld {
namespace {
constexpr unsigned debounce_frames = 100;
constexpr unsigned reset_frames = 100;
constexpr unsigned recovery_frames = 10;
}

UsbHostLink::UsbHostLink(const std::uint64_t frame_ticks) : frame_ticks_(frame_ticks) {
  if (frame_ticks == 0 || frame_ticks > std::numeric_limits<std::uint64_t>::max() / reset_frames) {
    throw Fault("USB host link requires a positive bounded millisecond frame period");
  }
}

bool UsbHostLink::poll(McuBus &bus, const std::uint64_t ticks) {
  if (previous_ && ticks < *previous_) { throw Fault("USB host link time moved backwards"); }
  previous_ = ticks;
  if (!bus.usb_host_connectable()) {
    if (stage_ == Stage::debounce) { stage_ = Stage::detached; }
    if (stage_ != Stage::detached) { throw Fault("USB device disconnected or disabled its link"); }
    return false;
  }
  if (stage_ == Stage::detached) {
    since_ = ticks;
    stage_ = Stage::debounce;
    return false;
  }
  // USB2.0 7.1.7.3 TATTDB: debounce restarts on disconnect. 7.1.7.5
  // TDRSTR requires at least 50 ms for a root port; this host holds 100 ms.
  if (stage_ == Stage::debounce) {
    if (ticks - since_ < frame_ticks_ * debounce_frames) { return false; }
    bus.observe_usb_decoder({.reset = true});
    since_ = ticks;
    stage_ = Stage::reset;
    return false;
  }
  if (stage_ == Stage::reset) {
    if (ticks - since_ < frame_ticks_ * reset_frames) { return false; }
    // S12USB20D6E2FV1 ENUMD is speed-handshake completion, not SET_CONFIGURATION.
    bus.observe_usb_decoder({.enumerated = true});
    since_ = ticks;
    frame_ = ticks / frame_ticks_;
    stage_ = Stage::recovery;
    return false;
  }
  const auto frame = ticks / frame_ticks_;
  if (frame > frame_) {
    if (frame - frame_ != 1) { throw Fault("USB host scheduler missed a device-time SOF frame"); }
    constexpr unsigned frame_mask = 0x7ff;
    bus.observe_usb_sof(static_cast<std::uint16_t>(frame & frame_mask));
    frame_ = frame;
  }
  // USB2.0 9.2.6.2 TRSTRCY: control transfers wait at least 10 ms after reset.
  if (stage_ == Stage::recovery && ticks - since_ >= frame_ticks_ * recovery_frames) {
    stage_ = Stage::connected;
  }
  return stage_ == Stage::connected;
}
}
