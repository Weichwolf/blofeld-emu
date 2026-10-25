// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/bus.hpp"
#include "usb/usb_host.hpp"
#include "usb/usbip.hpp"
#include "usb/usbip_export.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <vector>

namespace blofeld {
inline constexpr std::size_t usbip_input_chunk = 4096;
inline constexpr std::size_t maximum_usbip_output =
    maximum_usb_pending * (maximum_usb_transfer + usbip_transfer_header);
inline constexpr std::size_t maximum_usbip_replies = 4096;

struct UsbipInputProgress {
  // Frame assembly only: an accepted URB may still wait indefinitely for MIDI.
  bool frame_pending = false;
  bool completed_frame = false;
};

// Serialized, non-real-time connection state. No socket or CPU execution lives here.
// An imported export remains claimed after EOF/abort; re-publication requires
// separate board reset/state qualification, never automatic lease release.
class UsbipSession {
public:
  UsbipSession() = default;
  UsbipSession(const UsbipSession &) = delete;
  UsbipSession &operator=(const UsbipSession &) = delete;
  UsbipSession(UsbipSession &&) = delete;
  UsbipSession &operator=(UsbipSession &&) = delete;
  ~UsbipSession() = default;

  UsbipInputProgress receive(std::span<const std::uint8_t> bytes,
                             const std::shared_ptr<UsbipExport> &publication = {});
  void poll(McuBus &bus, ExecutionContext context);
  void end_input();
  void abort();
  [[nodiscard]] bool wants_input() const;
  [[nodiscard]] bool closed() const;

  [[nodiscard]] bool faulted() const { return state_ == State::faulted; }

  [[nodiscard]] bool imported() const { return imported_; }

  [[nodiscard]] std::size_t pending() const { return queue_.pending(); }

  [[nodiscard]] std::size_t queued_bytes() const { return output_bytes_; }

  [[nodiscard]] std::span<const std::uint8_t> output() const;
  void consume_output(std::size_t count);

private:
  enum class State : std::uint8_t { management, transfers, draining, closed, faulted };

  struct Output {
    std::vector<std::uint8_t> bytes;
    std::optional<std::uint32_t> sequence;
  };

  void manage(const std::shared_ptr<UsbipExport> &publication);
  void command(UsbipCommand value);
  void append(Output output);
  void finish_if_drained();
  void fail();
  State state_ = State::management;
  bool input_ended_ = false;
  bool imported_ = false;
  UsbipManagementDecoder management_;
  std::shared_ptr<UsbipExport> bound_;
  std::optional<UsbipDecoder> decoder_;
  UsbHostQueue queue_;
  std::set<std::uint32_t> live_;
  std::deque<Output> output_;
  std::size_t output_offset_ = 0;
  std::size_t output_bytes_ = 0;
};
}
