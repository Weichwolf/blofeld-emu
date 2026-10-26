// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_icg_status.hpp"

#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
namespace {
constexpr unsigned status1 = 0x4a, status2 = 0x4b;
constexpr unsigned mode_shift = 6, mode_mask = 0xc0, reference = 0x20;
constexpr unsigned lock = 8, external = 2, stable = 1, reserved = 0xfe;

void add(Hcs08KnownByte &byte, const std::optional<bool> value, const unsigned mask) {
  if (!value) { return; }
  byte.known |= static_cast<std::uint8_t>(mask);
  if (*value) { byte.value |= static_cast<std::uint8_t>(mask); }
}
}

Hcs08IcgStatus::Hcs08IcgStatus(const Hcs08IcgStatusContract contract) {
  if (contract != Hcs08IcgStatusContract::candidate_gt8a_observed_bits) {
    fail("configure", status1, "requires explicit candidate observed-status contract");
  }
}

void Hcs08IcgStatus::context(const ExecutionContext value) {
  context_ = value;
  events_.context(value);
}

void Hcs08IcgStatus::reset() {
  events_.reset();
  point_ = {}; // A reset sample is not a continuously valid live observation.
}

void Hcs08IcgStatus::observe(const Hcs08IcgStatusPoint point) {
  if (point.mode && *point.mode > 3) {
    fail("observe", status1, "CLKST requires a raw two-bit synchronized field");
  }
  if (point.mode && (*point.mode == 0 || *point.mode == 2) && point.locked == true) {
    fail("observe", status1, "LOCK cannot be set in self-clocked or bypassed mode");
  }
  point_ = point;
}

void Hcs08IcgStatus::control2(const std::uint8_t value) {
  events_.control2(value);
}

void Hcs08IcgStatus::signal(const Hcs08IcgFault event) {
  events_.signal(event);
}

Hcs08KnownByte Hcs08IcgStatus::bits(const std::uint16_t address) const {
  if (address == status2) {
    Hcs08KnownByte result{.known = reserved};
    add(result, point_.dco_stable, stable);
    return result;
  }
  if (address != status1) { fail("read", address, "unsupported status register"); }
  // Inspect a value copy; a refused strict read must not arm the live clear.
  auto copy = events_;
  const auto flags = copy.read_status_flags();
  Hcs08KnownByte result{.value = static_cast<std::uint8_t>(flags.value),
                        .known = static_cast<std::uint8_t>(flags.known)};
  if (point_.mode) {
    result.value |= static_cast<std::uint8_t>(*point_.mode << mode_shift);
    result.known |= mode_mask;
  }
  add(result, point_.reference_crystal, reference);
  const auto locked = point_.mode && (*point_.mode == 0 || *point_.mode == 2)
                          ? std::optional<bool>{false}
                          : point_.locked;
  add(result, locked, lock);
  add(result, point_.external_stable, external);
  return result;
}

Hcs08KnownByte Hcs08IcgStatus::read_known(const std::uint16_t address) {
  const auto result = bits(address);
  if (address == status1) { static_cast<void>(events_.read_status_flags()); }
  return result;
}

std::uint8_t Hcs08IcgStatus::read(const std::uint16_t address) {
  const auto result = bits(address);
  if (result.known != Hcs08KnownByte::all) {
    fail("read", address, "unknown live bits known=" + hex(result.known));
  }
  return read_known(address).value;
}

void Hcs08IcgStatus::write(const ByteWrite access, const std::uint8_t known) {
  if (access.address == status2) { return; } // Every bit is read-only.
  if (access.address != status1) { fail("write", access.address, "unsupported status register"); }
  if ((known & stable) == 0) { fail("write", status1, "unknown ICGIF clear request"); }
  events_.write_status(access.value); // Other data bits are ignored by hardware.
}

Hcs08IcgStatusState Hcs08IcgStatus::state() const {
  return {.point = point_, .events = events_.state()};
}

Hcs08IcgFaultAction Hcs08IcgStatus::action() const {
  return events_.action();
}

[[noreturn]] void Hcs08IcgStatus::fail(const std::string &operation,
                                       const unsigned address,
                                       const std::string &reason) const {
  throw Fault("HCS08 ICG status " + operation + " address=" + hex(address) +
              " pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
