// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08_icg.hpp"

#include "common/bus.hpp"

#include <cstdint>
#include <string>

namespace blofeld {
namespace hi = hcs08_icg;

namespace {
constexpr unsigned reset_trim = 0x80, mode_mask = 0x18;
constexpr unsigned fixed_after_first_write = 0xe0, control1_mutable = 0x1e;
constexpr unsigned frequency_fields = 0x77, control1_implemented = 0xfe;
constexpr unsigned external_request = 0x10, crystal_request = 0x20, refs_unresolved = 0xdf;
}

Hcs08Icg::Hcs08Icg(const Hcs08IcgContract contract, const Hcs08IcgReset cause) {
  if (contract != Hcs08IcgContract::candidate_gt8a_no_clock_events) {
    fail("configure",
         hi::control1,
         "requires the explicit candidate GT8A no-clock-event register contract");
  }
  reset(cause);
}

void Hcs08Icg::context(const ExecutionContext value) {
  context_ = value;
}

void Hcs08Icg::reset(const Hcs08IcgReset cause) {
  if (cause != Hcs08IcgReset::power_on && cause != Hcs08IcgReset::warm) {
    fail("reset", hi::control1, "unqualified reset cause");
  }
  const auto retained = state_.trim;
  state_ = {};
  state_.trim = retained;
  if (cause == Hcs08IcgReset::power_on) { state_.trim = reset_trim; }
}

[[noreturn]] void Hcs08Icg::unqualified_clock_event() {
  state_.reference_history_valid = false;
  state_.control1_known = 0;
  state_.filter.reset();
  fail("clock event",
       hi::control1,
       "unmodeled automatic mode/IRQ/reset evolution invalidates reference history");
}

void Hcs08Icg::require_history(const std::string &operation, const unsigned address) const {
  if (!state_.reference_history_valid) {
    fail(operation, address, "requires reset after an unqualified clock event");
  }
}

Hcs08IcgState Hcs08Icg::state() const {
  return state_;
}

std::uint8_t Hcs08Icg::read(const std::uint16_t address) const {
  require_history("read", address);
  switch (address) {
    case hi::control1:
      if (state_.control1_known != hi::all_known) {
        fail("read", address, "REFS readback after first internal-mode request is unqualified");
      }
      return state_.control1_request;
    case hi::control2: return state_.control2_request;
    case hi::trim:
      if (!state_.trim) { fail("read", address, "unknown retained trim"); }
      return *state_.trim;
    case hi::status1:
    case hi::status2:
      fail(
          "read", address, "live clock/status evolution is unqualified; reset flags are not ready");
    case hi::filter_upper:
    case hi::filter_lower: fail("read", address, "filter latch/synchronization is unqualified");
    default: fail("read", address, "unsupported register");
  }
}

void Hcs08Icg::write_control1(const std::uint8_t value) {
  if (state_.control1_written && (value & mode_mask) != (state_.control1_request & mode_mask)) {
    fail(
        "write", hi::control1, "changed CLKS requires qualified synchronization and pin ownership");
  }
  const auto frozen = state_.control1_request & fixed_after_first_write;
  const auto requested =
      state_.control1_written ? frozen | (value & control1_mutable) : value & control1_implemented;
  if (!state_.control1_written && (requested & external_request) == 0 &&
      (requested & crystal_request) != 0) {
    // GT9.4.10's REFS=0 lock note does not establish readback for GT9.3.1's
    // REFS=1 request with first CLKS=0X. Preserve the request, not an invented bit.
    state_.control1_known = refs_unresolved;
  }
  state_.control1_request = static_cast<std::uint8_t>(requested);
  state_.control1_written = true;
  if ((requested & mode_mask) != 0) { state_.filter.reset(); }
}

void Hcs08Icg::write_control2(const std::uint8_t value) {
  if (state_.control2_written &&
      (value & frequency_fields) != (state_.control2_request & frequency_fields)) {
    fail("write", hi::control2, "changed MFD/RFD requires qualified cross-domain synchronization");
  }
  state_.control2_request = value;
  state_.control2_written = true;
}

void Hcs08Icg::write(const ByteWrite access) {
  require_history("write", access.address);
  switch (access.address) {
    case hi::control1: write_control1(access.value); return;
    case hi::control2: write_control2(access.value); return;
    case hi::trim: state_.trim = access.value; return;
    case hi::status1:
      if ((access.value & 1U) != 0) {
        fail("write", access.address, "ICGIF clear sequence requires live read/event history");
      }
      return; // ICGIF zero has no effect; other status bits are read-only (GT9.3.3).
    case hi::status2: return; // Read-only (GT9.3.4); this does not establish DCOS.
    case hi::filter_upper:
    case hi::filter_lower:
      fail("write", access.address, "filter latch/synchronization is unqualified");
    default: fail("write", access.address, "unsupported register");
  }
}

[[noreturn]] void Hcs08Icg::fail(const std::string &operation,
                                 const unsigned address,
                                 const std::string &reason) const {
  throw Fault("HCS08 ICG " + operation + " address=" + hex(address) + " pc=" + hex(context_.pc) +
              " ccr=" + hex(context_.ccr) + " agent=CPU: " + reason);
}
} // namespace blofeld
