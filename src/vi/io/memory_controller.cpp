// SPDX-License-Identifier: GPL-3.0-only
#include "io/memory_controller.hpp"

#include "storage/media_image.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::vi {
namespace {
constexpr std::uint32_t read_strobe = 0x100, write_strobe = 0x200;
constexpr std::uint32_t command_latch = 0x400, address_latch = 0x800;
constexpr std::uint32_t strobes = read_strobe | write_strobe;
constexpr std::uint8_t reset_command = 0xff, spare_command = 0x50;
constexpr std::uint8_t erase_command = 0x60, confirm_erase = 0xd0, status_command = 0x70;
constexpr std::uint8_t program_command = 0x80, confirm_program = 0x10;
constexpr unsigned main_half = 256, spare_start = 512;
// Nominal VM timings: erase/program rounded up from 2 ms/200 us at 11.2896 MHz.
constexpr unsigned read_ticks = 64, reset_ticks = 32, erase_ticks = 22580, program_ticks = 2258;
constexpr std::uint8_t writable_status = 0x80, ready_status = 0x40;
}

void MemoryController::interrupt_write() noexcept {
  if (image_ == nullptr) { return; }
  if (operation_ == Operation::erase) {
    image_->interrupt_erase(page_ / MediaImage::block_pages);
  } else if (operation_ == Operation::program) {
    image_->interrupt_program({.page = page_, .column = column_},
                              std::span(program_).first(program_size_));
  }
}

void MemoryController::reset() noexcept {
  interrupt_write();
  error_ = nullptr;
  failure_location_.reset();
  phase_ = Phase::idle;
  operation_ = Operation::none;
  controls_ = strobes;
  page_ = column_ = pointer_ = 0;
  addresses_ = program_size_ = 0;
  epoch_ = 0;
  status_mode_ = failed_ = false;
  output_.reset();
  buffer_ = {};
}

bool MemoryController::fail(const char *message,
                            const std::optional<MediaImage::Location> location) noexcept {
  error_ = message;
  failure_location_ = location;
  return false;
}

unsigned MemoryController::reference_ticks() const noexcept {
  if (operation_ == Operation::erase) { return erase_ticks; }
  if (operation_ == Operation::program) { return program_ticks; }
  return operation_ == Operation::reset ? reset_ticks : read_ticks;
}

std::uint8_t MemoryController::status() const noexcept {
  return static_cast<std::uint8_t>(writable_status | (!pending() ? ready_status : 0) |
                                   (failed_ ? 1 : 0));
}

std::optional<std::uint8_t> MemoryController::output() const noexcept {
  if (output_ && status_mode_) { return status(); }
  return output_;
}

bool MemoryController::command(const std::uint8_t value) noexcept {
  output_.reset();
  if (value == reset_command) {
    interrupt_write();
    if (pointer_ == main_half) { pointer_ = 0; }
    buffer_ = {};
    status_mode_ = failed_ = false;
    ++epoch_;
    phase_ = Phase::idle;
    operation_ = Operation::reset;
    return true;
  }
  if (image_ == nullptr) { return fail("Media image is not connected"); }
  if (value == status_command) {
    status_mode_ = true;
    return true;
  }
  if (pending()) { return fail("Media command issued while an operation is busy"); }
  status_mode_ = false;
  buffer_ = {};
  if (value == confirm_erase) { return confirm(Operation::erase); }
  if (value == confirm_program) { return confirm(Operation::program); }
  if (value == program_command) {
    column_ = pointer_;
    phase_ = Phase::program_address;
  } else if (value == erase_command) {
    column_ = 0;
    phase_ = Phase::erase_address;
  } else {
    if (value != 0 && value != 1 && value != spare_command) {
      return fail("Media command is not implemented");
    }
    pointer_ = value == 1 ? main_half : 0;
    if (value == spare_command) { pointer_ = spare_start; }
    column_ = pointer_;
    phase_ = Phase::address;
  }
  page_ = 0;
  addresses_ = program_size_ = 0;
  return true;
}

bool MemoryController::confirm(const Operation operation) noexcept {
  if (operation == Operation::erase && phase_ != Phase::erase_confirm) {
    return fail("Media erase confirm has no complete row");
  }
  if (operation == Operation::program && (phase_ != Phase::program_data || program_size_ == 0)) {
    return fail("Media program confirm has no complete address and data");
  }
  if (pointer_ == main_half) { pointer_ = 0; }
  operation_ = operation;
  failed_ = false;
  ++epoch_;
  return true;
}

bool MemoryController::address(const std::uint8_t value) noexcept {
  const auto erasing = phase_ == Phase::erase_address;
  const auto programming = phase_ == Phase::program_address;
  if ((!erasing && !programming && phase_ != Phase::address) || addresses_ >= (erasing ? 3U : 4U)) {
    return fail("Unexpected media address byte");
  }
  if (!erasing && addresses_ == 0) {
    constexpr unsigned spare_mask = 15;
    column_ += pointer_ == spare_start ? value & spare_mask : value;
    if (column_ >= MediaImage::page_bytes) { return fail("Media column exceeds page"); }
  } else {
    page_ |= std::uint32_t{value} << ((addresses_ - (erasing ? 0U : 1U)) * 8U);
  }
  ++addresses_;
  if (addresses_ == (erasing ? 3U : 4U)) {
    if (page_ >= MediaImage::pages) { return fail("Media row exceeds image geometry"); }
    if (erasing) {
      phase_ = Phase::erase_confirm;
    } else if (programming) {
      phase_ = Phase::program_data;
    } else {
      if (pointer_ == main_half) { pointer_ = 0; }
      operation_ = Operation::read;
      ++epoch_;
    }
  }
  return true;
}

bool MemoryController::latch(const Write access) noexcept {
  if ((access.controls & (command_latch | address_latch)) == (command_latch | address_latch)) {
    return fail("Media command/address overlap");
  }
  if ((access.controls & command_latch) != 0) { return command(access.value); }
  if ((access.controls & address_latch) != 0) { return address(access.value); }
  if (pending() || status_mode_ || phase_ != Phase::program_data) {
    return fail("Media program data has no completed address phase");
  }
  if (program_size_ >= MediaImage::page_bytes - column_) {
    return fail("Media program data exceeds page");
  }
  program_[program_size_++] = access.value;
  return true;
}

bool MemoryController::read_output() noexcept {
  output_.reset();
  if (status_mode_) {
    output_ = status();
    return true;
  }
  if (pending() || phase_ != Phase::read) {
    // A shared-port strobe without readable data has an undefined output. The
    // actual data-register read must reject that unknown value; this control
    // write neither consumes a byte nor manufactures a device acknowledgement.
    return true;
  }
  const auto offset = column_ * 2;
  constexpr std::uint8_t all_bits = 0xff;
  if (buffer_.empty() || buffer_[offset + 1] != all_bits) {
    return fail("Unknown media image byte", MediaImage::Location{.page = page_, .column = column_});
  }
  output_ = buffer_[offset];
  return true;
}

bool MemoryController::write(const Write access) noexcept {
  if (error_ != nullptr) { return false; }
  auto controls = access.controls;
  // The firmware's neutral shared-port phase is inactive, not two strobes.
  if ((controls & strobes) == 0) { controls |= strobes; }
  const auto previous = controls_;
  // Shared control setup also occurs while Port F is an input. Only an actual
  // data producer can submit a command/address/payload transfer; an unknown
  // byte from an enabled producer must still fail.
  if ((previous & write_strobe) == 0 && (controls & write_strobe) != 0 && access.driven) {
    if (!access.known) { return fail("Unknown media command/address/data byte"); }
    if (!latch({.controls = controls, .value = access.value, .known = access.known})) {
      return false;
    }
  }
  if ((previous & read_strobe) != 0 && (controls & read_strobe) == 0) {
    if ((controls & write_strobe) == 0) { return fail("Media read/write overlap"); }
    if (!read_output()) { return false; }
  }
  if ((previous & read_strobe) == 0 && (controls & read_strobe) != 0 && output_) {
    // Advance once, retaining the sampled byte for the firmware's post-release
    // register read. The next read strobe or command replaces that latch.
    if (!status_mode_ && ++column_ == MediaImage::page_bytes) { phase_ = Phase::end; }
  }
  controls_ = controls;
  return true;
}

bool MemoryController::complete() {
  if (error_ != nullptr || !pending()) { return false; }
  if (operation_ == Operation::read) {
    if (image_ == nullptr) { return fail("Media image is not connected"); }
    buffer_ = image_->page(page_);
    if (buffer_.empty()) {
      return fail("Unknown media image page",
                  MediaImage::Location{.page = page_, .column = column_});
    }
    phase_ = Phase::read;
  } else if (operation_ == Operation::erase || operation_ == Operation::program) {
    const auto result = operation_ == Operation::erase
                            ? image_->erase(page_ / MediaImage::block_pages)
                            : image_->program({.page = page_, .column = column_},
                                              std::span(program_).first(program_size_));
    if (!result) {
      return fail("Unqualified media protection, validity or program history/limit",
                  MediaImage::Location{.page = page_, .column = column_});
    }
    failed_ = !*result;
    phase_ = Phase::idle;
  } else {
    failed_ = false;
    phase_ = Phase::idle;
  }
  operation_ = Operation::none;
  return true;
}

bool MemoryController::ready(bool &value) const noexcept {
  if (error_ != nullptr || image_ == nullptr) { return false; }
  value = !pending();
  return true;
}
}
