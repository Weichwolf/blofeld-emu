// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_cop.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08_system_options.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned enabled = 0x80, long_timeout = 0x40;
constexpr unsigned short_periods = 8192, long_periods = 262144;

unsigned timeout(const unsigned options) {
  return (options & long_timeout) != 0 ? long_periods : short_periods;
}
}

Hcs08Cop::Hcs08Cop(const Hcs08CopContract contract) {
  if (contract != Hcs08CopContract::candidate_gt8a_run_bus_edges) {
    fail("configure", status, "requires candidate GT8A run-mode bus-edge contract");
  }
}

void Hcs08Cop::context(const ExecutionContext value) {
  context_ = value;
}

void Hcs08Cop::reset(const std::optional<std::uint8_t> value) {
  if (value && (*value & 1U) != 0) { fail("reset", status, "SRS bit0 is unimplemented"); }
  options_.reset();
  reset_status_ = value;
  elapsed_ = 0;
  reset_requested_ = false;
}

std::uint64_t Hcs08Cop::advance_bus(const std::uint64_t periods) {
  healthy("clock", status);
  if ((options_.read() & enabled) == 0) { return periods; }
  const auto remaining = timeout(options_.read()) - elapsed_;
  const auto consumed = std::min<std::uint64_t>(periods, remaining);
  elapsed_ += static_cast<std::uint32_t>(consumed);
  reset_requested_ = elapsed_ == timeout(options_.read());
  return consumed;
}

Hcs08CopState Hcs08Cop::state() const {
  return {.reset_status = reset_status_,
          .elapsed = elapsed_,
          .options = options_.read(),
          .options_written = options_.written(),
          .reset_requested = reset_requested_};
}

std::uint8_t Hcs08Cop::read(const std::uint16_t address) const {
  healthy("read", address);
  if (address == Hcs08SystemOptions::address) { return options_.read(); }
  if (address != status) { fail("read", address, "unsupported register"); }
  if (!reset_status_) { fail("read", address, "unknown reset sources"); }
  return *reset_status_;
}

void Hcs08Cop::write(const ByteWrite access) {
  healthy("write", access.address);
  if (access.address == status) {
    elapsed_ = 0;
    return;
  }
  if (access.address != Hcs08SystemOptions::address) {
    fail("write", access.address, "unsupported register");
  }
  auto candidate = options_;
  candidate.write(access, context_);
  if ((candidate.read() & enabled) != 0 && elapsed_ >= timeout(candidate.read())) {
    fail("write", access.address, "shortened expired timeout requires qualified counter history");
  }
  options_ = candidate;
}

void Hcs08Cop::healthy(const std::string &operation, const unsigned address) const {
  if (reset_requested_) {
    fail(operation, address, "pending COP reset requires coordinated reset");
  }
}

[[noreturn]] void Hcs08Cop::fail(const std::string &operation,
                                 const unsigned address,
                                 const std::string &reason) const {
  throw Fault("HCS08 COP " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
