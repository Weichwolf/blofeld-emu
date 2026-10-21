// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/usbip_session.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

namespace blofeld {
void UsbipSession::append(Output output) {
  if (output.bytes.empty() || output.bytes.size() > maximum_usbip_output - output_bytes_ ||
      output_.size() == maximum_usbip_replies) {
    throw Fault("USB/IP outbound connection capacity exhausted");
  }
  const auto bytes = output.bytes.size();
  output_.push_back(std::move(output));
  output_bytes_ += bytes;
}

std::span<const std::uint8_t> UsbipSession::output() const {
  if (output_.empty()) { return {}; }
  return std::span(output_.front().bytes).subspan(output_offset_);
}

void UsbipSession::consume_output(const std::size_t count) {
  if (count == 0 || count > output().size()) { throw Fault("Invalid USB/IP output consumption"); }
  output_bytes_ -= count;
  output_offset_ += count;
  if (output_offset_ == output_.front().bytes.size()) {
    if (const auto sequence = output_.front().sequence) { live_.erase(*sequence); }
    output_.pop_front();
    output_offset_ = 0;
  }
  finish_if_drained();
}

void UsbipSession::finish_if_drained() {
  if (output_.empty() && (state_ == State::draining ||
                          (state_ == State::transfers && input_ended_ && queue_.pending() == 0))) {
    state_ = State::closed;
  }
}

void UsbipSession::end_input() {
  try {
    if (closed() || input_ended_) { throw Fault("Repeated/closed USB/IP input EOF"); }
    if (state_ == State::management) {
      management_.finish();
    } else if (state_ == State::transfers) {
      if (!decoder_) { throw Fault("USB/IP transfer EOF lacks its decoder"); }
      decoder_->finish();
    }
    input_ended_ = true;
    finish_if_drained();
  } catch (const Fault &) {
    fail();
    throw;
  }
}

void UsbipSession::abort() {
  queue_.reset();
  output_.clear();
  output_bytes_ = 0;
  output_offset_ = 0;
  live_.clear();
  input_ended_ = true;
  if (!faulted()) { state_ = State::closed; }
}

void UsbipSession::fail() {
  state_ = State::faulted;
  abort();
}
}
