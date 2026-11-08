// SPDX-License-Identifier: GPL-3.0-only
#include "io/memory_controller.hpp"

#include "storage/media_image.hpp"

#include <cstdint>

namespace blofeld::vi {
namespace {
constexpr std::uint32_t read_strobe = 0x100, write_strobe = 0x200;
constexpr std::uint32_t command = 0x400, address = 0x800, strobes = read_strobe | write_strobe;
constexpr std::uint8_t reset_command = 0xff, spare_command = 0x50;
constexpr unsigned main_half = 256, spare_start = 512;
}

void MemoryController::reset() noexcept {
  error_ = nullptr;
  phase_ = Phase::idle;
  controls_ = strobes;
  page_ = column_ = 0;
  addresses_ = 0;
  epoch_ = 0;
  pending_ = reset_pending_ = false;
  ready_ = true;
  output_.reset();
  buffer_ = {};
}

bool MemoryController::fail(const char *message) noexcept {
  error_ = message;
  return false;
}

bool MemoryController::latch(const Write access) noexcept {
  const auto controls = access.controls;
  const auto value = access.value;
  if ((controls & (command | address)) == (command | address)) {
    return fail("Media command/address overlap");
  }
  if ((controls & command) != 0) {
    output_.reset();
    buffer_ = {};
    if (value == reset_command) {
      ++epoch_;
      phase_ = Phase::idle;
      pending_ = reset_pending_ = true;
      ready_ = false;
      return true;
    }
    if (pending_) { return fail("Media command issued while an operation is busy"); }
    if (value != 0 && value != 1 && value != spare_command) {
      return fail("Media command is not implemented");
    }
    if (image_ == nullptr) { return fail("Media image is not connected"); }
    column_ = value == 1 ? main_half : 0;
    if (value == spare_command) { column_ = spare_start; }
    page_ = 0;
    addresses_ = 0;
    phase_ = Phase::address;
    ready_ = true;
    return true;
  }
  if ((controls & address) == 0 || phase_ != Phase::address || addresses_ >= 4) {
    return fail("Unexpected media address or program byte");
  }
  if (addresses_ == 0) {
    column_ += value;
    if (column_ >= MediaImage::page_bytes) { return fail("Media column exceeds page"); }
  } else {
    page_ |= std::uint32_t{value} << ((addresses_ - 1) * 8U);
  }
  ++addresses_;
  if (addresses_ == 4) {
    if (page_ >= MediaImage::pages) { return fail("Media row exceeds image geometry"); }
    pending_ = true;
    ++epoch_;
    ready_ = false;
  }
  return true;
}

bool MemoryController::write(const Write access) noexcept {
  if (error_ != nullptr) { return false; }
  auto controls = access.controls;
  // The firmware's neutral shared-port phase is inactive, not two strobes.
  if ((controls & strobes) == 0) { controls |= strobes; }
  const auto previous = controls_;
  if ((previous & write_strobe) == 0 && (controls & write_strobe) != 0) {
    if (!access.known) { return fail("Unknown media command/address byte"); }
    if (!latch({.controls = controls, .value = access.value, .known = access.known})) {
      return false;
    }
  }
  if ((previous & read_strobe) != 0 && (controls & read_strobe) == 0) {
    if ((controls & write_strobe) == 0 || !ready_ || phase_ != Phase::read) {
      return fail("Media read is not ready");
    }
    const auto offset = column_ * 2;
    constexpr std::uint8_t all_bits = 0xff;
    if (buffer_.empty() || buffer_[offset + 1] != all_bits) {
      return fail("Unknown media image byte");
    }
    output_ = buffer_[offset];
  }
  if ((previous & read_strobe) == 0 && (controls & read_strobe) != 0 && output_) {
    output_.reset();
    ++column_;
    if (column_ == MediaImage::page_bytes) { phase_ = Phase::end; }
  }
  controls_ = controls;
  return true;
}

bool MemoryController::complete() noexcept {
  if (error_ != nullptr || !pending_) { return false; }
  if (!reset_pending_) {
    if (image_ == nullptr) { return fail("Media image is not connected"); }
    buffer_ = image_->page(page_);
    if (buffer_.empty()) { return fail("Unknown media image page"); }
  }
  pending_ = false;
  ready_ = true;
  phase_ = reset_pending_ ? Phase::idle : Phase::read;
  reset_pending_ = false;
  return true;
}

bool MemoryController::ready(bool &value) const noexcept {
  if (error_ != nullptr || image_ == nullptr) { return false; }
  value = ready_;
  return true;
}
}
