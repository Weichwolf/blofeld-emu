// SPDX-License-Identifier: GPL-3.0-only
#include "storage/nand.hpp"

#include "common/bus.hpp"
#include "storage/nand_array.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace blofeld {
namespace {
constexpr std::uint8_t second_half = 1;
constexpr std::uint8_t spare = 0x50;
constexpr std::uint8_t reset_command = 0xff;
constexpr std::uint8_t read_status = 0x70;
constexpr std::uint8_t program_command = 0x80;
constexpr std::uint8_t program_confirm = 0x10;
constexpr std::uint8_t erase_command = 0x60;
constexpr std::uint8_t erase_confirm = 0xd0;
constexpr unsigned half_bytes = 256;
constexpr unsigned address_bytes = 4;
constexpr unsigned byte_bits = 8;
}

[[noreturn]] void Nand::fail(const std::string &operation, const std::string &reason) const {
  throw Fault("NAND " + operation + " page=" + hex(page_, 6) + " column=" + hex(column_, 3) +
              " command=" + hex(command_, 2) + ": " + reason);
}

NandPending Nand::pending() const {
  switch (phase_) {
    case Phase::read_pending: return NandPending::read;
    case Phase::reset_pending: return NandPending::reset;
    case Phase::program_pending: return NandPending::program;
    case Phase::erase_pending: return NandPending::erase;
    case Phase::cold:
    case Phase::idle:
    case Phase::read_address:
    case Phase::data:
    case Phase::end:
    case Phase::program_address:
    case Phase::program_data:
    case Phase::erase_address: return NandPending::none;
    default: fail("pending observation", "invalid internal phase");
  }
}

void Nand::check_observation() const {
  if (pending() == NandPending::program || pending() == NandPending::erase) {
    fail("contents/block observation", "write operation is active");
  }
}

void Nand::observe_power_ready() {
  if (phase_ != Phase::cold) { fail("power-ready observation", "not a cold device"); }
  phase_ = Phase::read_address;
  pointer_ = 0;
  ready_ = true;
}

void Nand::observe_bytes(const unsigned page,
                         const unsigned column,
                         const std::span<const std::uint8_t> bytes) {
  check_observation();
  try {
    contents_.observe_bytes({.page = page, .column = column}, bytes);
  } catch (const Fault &fault) { fail("contents observation", fault.what()); }
}

void Nand::observe_cell(const NandLocation location, const NandByte cell) {
  check_observation();
  try {
    contents_.observe_cell(location, cell);
  } catch (const Fault &fault) { fail("cell observation", fault.what()); }
}

void Nand::observe_block(const unsigned block, const NandBlock state) {
  check_observation();
  try {
    contents_.observe_block(block, state);
  } catch (const Fault &fault) { fail("block observation", fault.what()); }
}

NandByte Nand::cell(const NandLocation location) const {
  return contents_.cell(location);
}

void Nand::begin_read(const std::uint8_t value) {
  if (phase_ == Phase::program_address || phase_ == Phase::program_data ||
      phase_ == Phase::erase_address) {
    fail("read setup", "unqualified replacement of a staged write sequence");
  }
  unsigned base = 0;
  if (value == spare) { base = main_bytes; }
  if (value == second_half) { base = half_bytes; }
  pointer_ = base;
  address_cycles_ = 0;
  status_mode_ = false;
  phase_ = Phase::read_address;
}

void Nand::command(const std::uint8_t value) {
  if (value == reset_command) {
    abort_active();
    phase_ = Phase::reset_pending;
    ready_ = false;
    command_ = value;
    address_cycles_ = 0;
    status_mode_ = false;
    outcome_.reset();
    reset_status_ = false;
    pointer_.reset();
    program_ = {};
    return;
  }
  if (value == read_status && phase_ != Phase::cold && phase_ != Phase::end &&
      phase_ != Phase::program_address && phase_ != Phase::program_data &&
      phase_ != Phase::erase_address) {
    status_mode_ = true;
    command_ = value;
    return;
  }
  if (ready_ != true || phase_ == Phase::end) {
    fail("command", "unqualified readiness/post-page state; value=" + hex(value, 2));
  }
  switch (value) {
    case 0:
    case second_half:
    case spare: begin_read(value); break;
    case program_command: begin_program(); break;
    case program_confirm: confirm_program(); break;
    case erase_command: begin_erase(); break;
    case erase_confirm: confirm_erase(); break;
    default: fail("command", "unsupported value=" + hex(value, 2));
  }
  command_ = value;
}

void Nand::address_page(const std::uint8_t value) {
  if (address_cycles_ == 0) {
    constexpr unsigned spare_columns = page_bytes - main_bytes;
    if (!pointer_) { fail("address", "unknown area pointer"); }
    if (*pointer_ == main_bytes && value >= spare_columns) {
      fail("address", "unqualified upper spare-column bits");
    }
    column_ = *pointer_ + value;
    page_ = 0;
  } else {
    const auto shift = (address_cycles_ - 1) * byte_bits;
    const auto page = page_ | (static_cast<unsigned>(value) << shift);
    if (page >= pages) { fail("address", "row exceeds device: page=" + hex(page, 6)); }
    page_ = page;
  }
  ++address_cycles_;
  if (address_cycles_ == address_bytes) {
    if (phase_ == Phase::program_address) {
      program_.start = column_;
      phase_ = Phase::program_data;
    } else {
      phase_ = Phase::read_pending;
      ready_ = false;
      outcome_.reset();
      reset_status_ = false;
    }
  }
}

void Nand::address(const std::uint8_t value) {
  if (!status_mode_ && address_cycles_ == address_bytes &&
      (phase_ == Phase::read_pending || (phase_ == Phase::program_data && program_.size == 0))) {
    return;
  }
  if (phase_ == Phase::erase_address && ready_ == true) {
    address_erase(value);
    return;
  }
  if ((phase_ != Phase::read_address && phase_ != Phase::program_address) || ready_ != true ||
      status_mode_) {
    fail("address", "no qualified read command/address phase");
  }
  address_page(value);
}

void Nand::observe_completion() {
  if (phase_ == Phase::reset_pending) {
    phase_ = Phase::idle;
    address_cycles_ = 0;
    page_ = 0;
    column_ = 0;
    reset_status_ = wp_high_ == true;
    if (reset_status_) { outcome_ = NandOutcome::pass; }
  } else if (phase_ == Phase::read_pending) {
    phase_ = Phase::data;
  } else {
    fail("completion observation", "no pending reset/read; writes require an observed outcome");
  }
  ready_ = true;
}

NandByte Nand::status() const {
  constexpr unsigned ready_bit = 0x40;
  constexpr unsigned wp_bit = 0x80;
  NandByte result;
  constexpr unsigned reset_unused = 0x3f;
  if (reset_status_) { result.known = reset_unused; }
  if (ready_.has_value()) {
    result.known |= ready_bit;
    if (*ready_) { result.value |= ready_bit; }
  }
  if (wp_high_.has_value()) {
    result.known |= wp_bit;
    if (*wp_high_) { result.value |= wp_bit; }
  }
  if (outcome_) {
    result.known |= 1;
    if (*outcome_ == NandOutcome::fail) { result.value |= 1; }
  }
  return result;
}

std::uint8_t Nand::read(const std::uint8_t mask) {
  if (mask == 0) { fail("read", "empty read mask"); }
  if (status_mode_) {
    const auto sample = status();
    if ((mask & ~sample.known) != 0) {
      fail("status read", "unknown status bits mask=" + hex(mask, 2));
    }
    return static_cast<std::uint8_t>(sample.value & mask);
  }
  if (phase_ != Phase::data || ready_ != true) { fail("read", "no completed page transfer"); }
  const auto sample = contents_.cell({.page = page_, .column = column_});
  if ((mask & ~sample.known) != 0) { fail("read", "unknown contents"); }
  const auto value = static_cast<std::uint8_t>(sample.value & mask);
  ++column_;
  if (column_ == page_bytes) {
    phase_ = Phase::end;
    ready_.reset();
  }
  return value;
}
}
