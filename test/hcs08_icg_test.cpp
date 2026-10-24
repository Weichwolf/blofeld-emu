// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08_icg.hpp"
#include "hcs08_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace hi = hcs08_icg;
constexpr auto chip = Hcs08IcgContract::candidate_gt8a_no_clock_events;
constexpr auto warm = Hcs08IcgReset::warm, por = Hcs08IcgReset::power_on;
constexpr unsigned first_control = 0x28, second_control = 0x30, por_trim = 0x80;
constexpr unsigned own_pc = 0xe1a3, own_ccr = 0x79;
constexpr unsigned reset_control = 0x44, reset_filter = 0xc0, mutable_control = 0x1e;

Hcs08Icg configured(const Hcs08IcgReset cause) {
  Hcs08Icg icg(chip, cause);
  icg.context({.pc = own_pc, .ccr = own_ccr});
  return icg;
}

void reset_knowledge() {
  require(rejected([&] {
            const Hcs08Icg icg(Hcs08IcgContract::unqualified, por);
          }).contains("explicit candidate"),
          "An unidentified part cannot silently select an ICG register map");
  auto icg = configured(warm);
  require(icg.state() == Hcs08IcgState{} && icg.read(hi::control1) == reset_control &&
              icg.read(hi::control2) == 0 && icg.state().filter == reset_filter &&
              rejected([&] { static_cast<void>(icg.read(hi::trim)); }).contains("unknown retained"),
          "Warm entry restores control/filter but does not invent an earlier live trim");
  icg.reset(warm);
  require(!icg.state().trim, "Unknown trim stays unknown across repeated warm resets");
  const auto before = icg.state();
  require(rejected([&] { icg.reset(Hcs08IcgReset::unqualified); }).contains("reset cause") &&
              icg.state() == before,
          "An unqualified reset cause fails before changing register state");
  for (unsigned value = 0; value < byte_values; ++value) {
    icg.write({.address = hi::trim, .value = static_cast<std::uint8_t>(value)});
    icg.write({.address = hi::control1, .value = first_control});
    icg.write({.address = hi::control2, .value = second_control});
    require(icg.state().trim == value && !icg.state().filter,
            "An FLL request invalidates filter knowledge without supplying a frequency");
    icg.reset(warm);
    require(icg.state() == Hcs08IcgState{.trim = static_cast<std::uint8_t>(value)} &&
                icg.read(hi::trim) == value,
            "All256 trim values survive warm reset while control/filter/write-once state resets");
    icg.reset(por);
    require(icg.state() == Hcs08IcgState{.trim = por_trim} && icg.read(hi::trim) == por_trim,
            "Power-on reset alone restores documented80, without reading nonvolatile FFBE");
  }
}

void control_requests() {
  constexpr unsigned upper_fields = 0xe0;
  constexpr unsigned writable_low = 6;
  constexpr unsigned frequency_bits = 0x77;
  constexpr unsigned refs_test = 0x30;
  constexpr unsigned internal_refs_one = 0x20;
  constexpr unsigned refs_unknown = 0xdf;
  for (unsigned value = 0; value < byte_values; ++value) {
    auto icg = configured(por);
    icg.write({.address = hi::control1, .value = static_cast<std::uint8_t>(value)});
    const bool ambiguous_refs = (value & refs_test) == internal_refs_one;
    require(icg.state().control1_request == value - (value % 2) && icg.state().control1_written &&
                icg.state().control1_known == (ambiguous_refs ? refs_unknown : byte_mask) &&
                icg.state().control2_request == 0 && icg.state().trim == por_trim,
            "All first ICGC1 stores retain request fields and distinguish ambiguous REFS readback");
    if (ambiguous_refs) {
      require(
          rejected([&] { static_cast<void>(icg.read(hi::control1)); }).contains("REFS readback"),
          "A request bit cannot silently become an unproved hardware readback");
    } else {
      require(icg.read(hi::control1) == value - (value % 2),
              "Unambiguous first control readback retains read-zero bit0");
    }
    const unsigned modes = (value / 8) % 4;
    require(icg.state().filter.has_value() == (modes == 0),
            "Only an unchanged open-loop request retains the reset filter knowledge");
    const auto again = static_cast<std::uint8_t>(value ^ (upper_fields | writable_low));
    icg.write({.address = hi::control1, .value = again});
    require(icg.state().control1_request == ((value & upper_fields) | (again & mutable_control)) &&
                icg.state().control1_known == (ambiguous_refs ? refs_unknown : byte_mask),
            "HGO/RANGE/REFS are write-once; identical CLKS permits independent OSCSTEN/LOCD");
    const auto before = icg.state();
    constexpr unsigned other_mode = 8;
    require(rejected([&] {
              icg.write({.address = hi::control1,
                         .value = static_cast<std::uint8_t>(again ^ other_mode)});
            }).contains("synchronization and pin ownership") &&
                icg.state() == before,
            "Different CLKS cannot silently claim cross-domain completion or an external pin");
    icg.write({.address = hi::control2, .value = static_cast<std::uint8_t>(value)});
    require(icg.read(hi::control2) == value && icg.state().control2_written,
            "Every first MFD/RFD request is recorded without claiming DCO/LOCK evolution");
    constexpr unsigned reset_enables = 0x88;
    icg.write({.address = hi::control2, .value = static_cast<std::uint8_t>(value ^ reset_enables)});
    require(icg.read(hi::control2) == (value ^ reset_enables),
            "LOLRE/LOCRE remain writable when requested frequency fields are unchanged");
    const auto stable = icg.state();
    for (unsigned bit = 1; bit <= por_trim; bit *= 2) {
      if ((bit & frequency_bits) == 0) { continue; }
      require(rejected([&] {
                icg.write({.address = hi::control2,
                           .value = static_cast<std::uint8_t>((value ^ reset_enables) ^ bit)});
              }).contains("cross-domain synchronization") &&
                  icg.state() == stable,
              "Every changed MFD/RFD bit is refused atomically before a clock owner exists");
    }
  }
}

void strict_transactions() {
  auto icg = configured(por);
  const auto before = icg.state();
  constexpr std::array<std::uint16_t, 5> unresolved{
      hi::status1, hi::status2, hi::filter_upper, hi::filter_lower, hi::trim + 1};
  for (const auto address : unresolved) {
    const auto error = rejected([&] { static_cast<void>(icg.read(address)); });
    require(error.contains("HCS08 ICG read") && error.contains("address=" + hex(address)) &&
                error.contains("pc=0xe1a3") && error.contains("ccr=0x0079") &&
                error.contains("agent=CPU") && icg.state() == before,
            "Live status/filter and unsupported reads fail with full context and unchanged state");
  }
  for (const auto address :
       std::array<std::uint16_t, 3>{hi::filter_upper, hi::filter_lower, hi::status1}) {
    const auto *const expected = address == hi::status1 ? "ICGIF clear sequence" : "filter latch";
    require(rejected([&] { icg.write({.address = address, .value = 1}); }).contains(expected),
            "Unmodeled filter latch and interrupt-clear transactions are never successful stubs");
    require(icg.state() == before, "Refused writes leave every modeled register unchanged");
  }
  constexpr unsigned no_clear = 0xfe;
  icg.write({.address = hi::status1, .value = no_clear});
  icg.write({.address = hi::status2, .value = byte_mask});
  require(icg.state() == before &&
              rejected([&] { static_cast<void>(icg.read(hi::status2)); }).contains("live clock"),
          "Documented ineffective status writes cannot establish a stable DCO or ready flags");
  require(rejected([&] {
            icg.write({.address = hi::trim + 1, .value = 0});
          }).contains("unsupported register") &&
              icg.state() == before,
          "A reserved address is not a read/write register bank");
}

void clock_event_quarantine() {
  auto icg = configured(por);
  constexpr unsigned stored_trim = 0x5a;
  icg.write({.address = hi::trim, .value = stored_trim});
  icg.write({.address = hi::control1, .value = first_control});
  icg.write({.address = hi::control2, .value = second_control});
  require(rejected([&] { icg.unqualified_clock_event(); }).contains("automatic mode/IRQ/reset") &&
              !icg.state().reference_history_valid && icg.state().control1_known == 0 &&
              !icg.state().filter && icg.state().trim == stored_trim,
          "An unmodeled clock event invalidates live knowledge instead of reusing old requests");
  const auto failed = icg.state();
  for (unsigned address = hi::control1; address <= hi::trim; ++address) {
    const auto reg = static_cast<std::uint16_t>(address);
    require(
        rejected([&] { static_cast<void>(icg.read(reg)); }).contains("requires reset") &&
            rejected([&] { icg.write({.address = reg, .value = 0}); }).contains("requires reset") &&
            icg.state() == failed,
        "Every actual ICG transaction stays quarantined after unknown automatic changes");
  }
  require(rejected([&] { icg.reset(Hcs08IcgReset::unqualified); }).contains("reset cause") &&
              icg.state() == failed,
          "An invented reset cause cannot reopen access to an unqualified history");
  icg.reset(warm);
  require(icg.state() == Hcs08IcgState{.trim = stored_trim} && icg.read(hi::trim) == stored_trim,
          "A real explicit warm register reset recovers the reference state, without a clock rate");
}
} // namespace

int main() {
  try {
    reset_knowledge();
    control_requests();
    strict_transactions();
    clock_event_quarantine();
    std::cout << "HCS08 ICG reset/trim,256 control requests, write-once fields and strict live "
                 "clock boundaries passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
