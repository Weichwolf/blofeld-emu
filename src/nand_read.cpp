// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/nand_read.hpp"

#include "blofeld/bus.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace blofeld {
namespace {
constexpr std::uint8_t second_half = 1;
constexpr std::uint8_t spare = 0x50;
constexpr std::uint8_t reset_command = 0xff;
constexpr unsigned half_bytes = 256;
constexpr unsigned address_bytes = 4;
constexpr unsigned byte_bits = 8;
}

[[noreturn]] void NandRead::fail(const std::string &operation, const std::string &reason) const {
  throw Fault("NAND " + operation + " page=" + hex(page_, 6) + " column=" + hex(column_, 3) +
              " command=" + hex(command_, 2) + ": " + reason);
}

void NandRead::observe_power_ready() {
  if (phase_ != Phase::cold) { fail("power-ready observation", "not a cold device"); }
  // The documented power-on pointer is Read1; ready is a separate lab observation.
  phase_ = Phase::address;
  ready_ = true;
}

void NandRead::observe_bytes(const unsigned page,
                             const unsigned column,
                             const std::span<const std::uint8_t> bytes) {
  if (page >= pages || column > page_bytes || bytes.size() > page_bytes - column) {
    fail("contents observation",
         "range exceeds page: page=" + hex(page, 6) + " column=" + hex(column, 3));
  }
  if (bytes.empty()) { return; }
  auto &target = contents_[page];
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    const auto position = column + index;
    target.bytes.at(position) = bytes[index];
    target.known.at(position) = true;
  }
}

void NandRead::command(const std::uint8_t value) {
  if (value == reset_command) {
    phase_ = Phase::reset_pending;
    ready_ = false;
    command_ = value;
    address_cycles_ = 0;
    return;
  }
  if (ready_ != true || phase_ == Phase::end) {
    fail("command", "unqualified readiness/post-page state; value=" + hex(value, 2));
  }
  if (value != 0 && value != second_half && value != spare) {
    fail("command", "unsupported value=" + hex(value, 2));
  }
  command_ = value;
  address_cycles_ = 0;
  phase_ = Phase::address;
}

void NandRead::address(const std::uint8_t value) {
  // The array-organization description explicitly ignores surplus address cycles.
  if (address_cycles_ == address_bytes && phase_ == Phase::read_pending) { return; }
  if (phase_ != Phase::address || ready_ != true) {
    fail("address", "no qualified read command/address phase");
  }
  if (address_cycles_ == 0) {
    constexpr unsigned spare_columns = page_bytes - main_bytes;
    if (command_ == spare && value >= spare_columns) {
      fail("address", "unqualified upper spare-column bits");
    }
    unsigned base = 0;
    if (command_ == spare) { base = main_bytes; }
    if (command_ == second_half) { base = half_bytes; }
    column_ = base + value;
    page_ = 0;
  } else {
    const auto shift = (address_cycles_ - 1) * byte_bits;
    const auto page = page_ | (static_cast<unsigned>(value) << shift);
    if (page >= pages) { fail("address", "row exceeds device: page=" + hex(page, 6)); }
    page_ = page;
  }
  ++address_cycles_;
  if (address_cycles_ == address_bytes) {
    phase_ = Phase::read_pending;
    ready_ = false;
  }
}

void NandRead::observe_completion() {
  if (phase_ == Phase::reset_pending) {
    phase_ = Phase::idle;
    address_cycles_ = 0;
    page_ = 0;
    column_ = 0;
  } else if (phase_ == Phase::read_pending) {
    phase_ = Phase::data;
  } else {
    fail("completion observation", "no pending reset/read");
  }
  ready_ = true;
}

std::uint8_t NandRead::read() {
  if (phase_ != Phase::data || ready_ != true) { fail("read", "no completed page transfer"); }
  const auto found = contents_.find(page_);
  if (found == contents_.end() || !found->second.known.at(column_)) {
    fail("read", "unknown contents");
  }
  const auto value = found->second.bytes.at(column_);
  ++column_;
  if (column_ == page_bytes) {
    // TSOP sequential-row entry and CE abort timing need a separate qualified model.
    phase_ = Phase::end;
    ready_.reset();
  }
  return value;
}
}
