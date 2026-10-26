// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "keyboard/hcs08_icg_events.hpp"
#include "keyboard/hcs08_icg_status.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr auto contract = Hcs08IcgStatusContract::candidate_gt8a_observed_bits;
constexpr unsigned first_status = 0x4a, second_status = 0x4b, own_pc = 0xe1b0, own_ccr = 0x69;
constexpr unsigned all_bytes = 256, all_known = 255, event_mask = 0x15, second_reserved = 0xfe;
constexpr unsigned mode_step = 64, ref_bit = 32, lock_bit = 8, ext_bit = 2;
constexpr unsigned complete_point = 0x6a, expected_observations = 351, expected_contradictions = 54;
constexpr unsigned expected_ack_cases = 3072;
constexpr std::array<std::optional<bool>, 3> facts{std::nullopt, false, true};
unsigned observations = 0, contradictions = 0, ack_cases = 0;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Expected strict status refusal");
}

Hcs08IcgStatus configured() {
  Hcs08IcgStatus status(contract);
  status.context({.pc = own_pc, .ccr = own_ccr});
  return status;
}

Hcs08IcgStatusPoint complete() {
  return {.mode = 1,
          .reference_crystal = true,
          .locked = true,
          .external_stable = true,
          .dco_stable = true};
}

void observation(const std::optional<std::uint8_t> mode,
                 const std::optional<bool> reference,
                 const std::optional<bool> locked,
                 const std::optional<bool> external,
                 const std::optional<bool> dco) {
  auto status = configured();
  const Hcs08IcgStatusPoint point{.mode = mode,
                                  .reference_crystal = reference,
                                  .locked = locked,
                                  .external_stable = external,
                                  .dco_stable = dco};
  if (mode && (*mode == 0 || *mode == 2) && locked == true) {
    const auto before = status.state();
    require(rejected([&] { status.observe(point); }).contains("LOCK cannot") &&
                status.state() == before,
            "Impossible LOCK observation rejects atomically");
    ++contradictions;
    return;
  }
  status.observe(point);
  unsigned value = mode ? *mode * mode_step : 0;
  unsigned known = mode ? event_mask | (3 * mode_step) : event_mask;
  if (reference) {
    known |= ref_bit;
    if (*reference) { value |= ref_bit; }
  }
  if (locked || (mode && (*mode == 0 || *mode == 2))) {
    known |= lock_bit;
    if (locked == true) { value |= lock_bit; }
  }
  if (external) {
    known |= ext_bit;
    if (*external) { value |= ext_bit; }
  }
  const auto first = status.read_known(first_status);
  const auto second = status.read_known(second_status);
  require(first.value == value && first.known == known && second.value == (dco == true ? 1U : 0U) &&
              second.known == (dco ? all_known : second_reserved),
          "Independent literal raw-field and optional-knowledge expectations");
  const auto before = status.state();
  if (known == all_known) {
    require(status.read(first_status) == value, "Complete observed byte admits ordinary read");
  } else {
    require(
        rejected([&] { static_cast<void>(status.read(first_status)); }).contains("unknown live"),
        "Incomplete status never becomes an ordinary byte");
  }
  require(status.state() == before, "No pending interrupt means read/strict refusal has no effect");
  ++observations;
}

void observations_matrix() {
  for (unsigned raw = 0; raw <= 4; ++raw) {
    const auto mode = raw == 4 ? std::optional<std::uint8_t>{}
                               : std::optional<std::uint8_t>{static_cast<std::uint8_t>(raw)};
    for (const auto reference : facts) {
      for (const auto locked : facts) {
        for (const auto external : facts) {
          for (const auto dco : facts) { observation(mode, reference, locked, external, dco); }
        }
      }
    }
  }
  require(observations == expected_observations && contradictions == expected_contradictions,
          "Every explicit unknown/false/true status combination was classified");
}

void acknowledgements() {
  struct Routing {
    unsigned control;
    std::array<unsigned, 3> before, after;
  };

  constexpr std::array<Routing, 4> routes{
      {{.control = 0, .before = {17, 5, 21}, .after = {0, 0, 0}},
       {.control = 8, .before = {17, 4, 21}, .after = {0, 4, 4}},
       {.control = 128, .before = {16, 5, 21}, .after = {16, 0, 16}},
       {.control = 136, .before = {16, 4, 20}, .after = {16, 4, 20}}}};
  constexpr std::array events{Hcs08IcgFault::loss_of_lock,
                              Hcs08IcgFault::loss_of_clock,
                              Hcs08IcgFault::loss_of_clock_and_lock};
  for (const auto &route : routes) {
    for (unsigned n = 0; n < events.size(); ++n) {
      for (unsigned value = 0; value < all_bytes; ++value) {
        auto status = configured();
        status.control2(static_cast<std::uint8_t>(route.control));
        status.signal(events[n]);
        const auto pending = status.state();
        require(
            rejected([&] { static_cast<void>(status.read(first_status)); }).contains("unknown") &&
                status.state() == pending,
            "Refused incomplete ordinary read cannot arm the clear sequence");
        status.write({.address = first_status, .value = 1});
        require(status.state() == pending, "Write without a real read cannot acknowledge");
        const auto first = status.read_known(first_status);
        require(first.value == route.before[n] && first.known == event_mask,
                "Real masked read retains owned pending flags and arms any interrupt");
        auto saved = status;
        status.write({.address = first_status, .value = static_cast<std::uint8_t>(value)}, 1);
        saved.write({.address = first_status, .value = static_cast<std::uint8_t>(value)}, 1);
        require(saved.state() == status.state(), "Copied armed sequence replays independently");
        status.observe(complete());
        const unsigned flags = value % 2 == 0 ? route.before[n] : route.after[n];
        require(status.read(first_status) == (complete_point | flags) &&
                    status.read(second_status) == 1,
                "Literal fault routing/ack result composes with complete actual clock facts");
        ++ack_cases;
      }
    }
  }
  require(ack_cases == expected_ack_cases, "All data bytes and fault/reset routes were covered");
}

void invalid_and_reset() {
  require(rejected([] {
            const Hcs08IcgStatus bad(Hcs08IcgStatusContract::unqualified);
          }).contains("explicit candidate"),
          "No default observed-status contract");
  auto status = configured();
  status.observe(complete());
  status.signal(Hcs08IcgFault::loss_of_lock);
  static_cast<void>(status.read_known(first_status));
  const auto armed = status.state();
  const auto invalid = rejected([&] { status.observe({.mode = 4}); });
  require(invalid.contains("address=0x004a") && invalid.contains("pc=0xe1b0") &&
              invalid.contains("ccr=0x0069") && invalid.contains("agent=CPU") &&
              status.state() == armed,
          "Invalid synchronized field rejects with complete context before mutation");
  require(rejected([&] {
            status.write({.address = first_status, .value = 1}, 0);
          }).contains("unknown ICGIF") &&
              status.state() == armed,
          "Unknown writable clear bit cannot acknowledge");
  status.write({.address = second_status, .value = all_known}, 0);
  require(status.state() == armed, "Every ICGS2 data bit is ignored without consuming knowledge");
  require(rejected([&] {
            static_cast<void>(status.read_known(second_status + 1));
          }).contains("unsupported") &&
              rejected([&] {
                status.write({.address = second_status + 1, .value = 0});
              }).contains("unsupported") &&
              status.state() == armed,
          "Neighboring unsupported transactions remain visible and atomic");
  status.signal(Hcs08IcgFault::loss_of_clock);
  status.write({.address = first_status, .value = 1});
  require(status.read(first_status) == (complete_point | event_mask),
          "A new interrupt invalidates an older read-clear sequence");
  status.reset();
  require(status.state() == Hcs08IcgStatusState{} && status.action() == Hcs08IcgFaultAction::none,
          "Reset clears latch history and invalidates live clock observations");
  const auto first = status.read_known(first_status);
  const auto second = status.read_known(second_status);
  require(first.value == 0 && first.known == event_mask && second.value == 0 &&
              second.known == second_reserved,
          "A reset constructor never presents unknown clock flags as known ready or stable");
}
}

int main() {
  try {
    observations_matrix();
    acknowledgements();
    invalid_and_reset();
    std::cout << "ICG status: " << observations << " observed points, " << contradictions
              << " contradictions, " << ack_cases
              << " masked read/ack cases. No live clock generation.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
