// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08_tpm.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
constexpr auto contract = Hcs08TpmContract::candidate_gt8a_run_bus_edges;
constexpr unsigned values = 256, cycle = 65536, flag = 0x80, bus = 8, irq = 0x40;
constexpr unsigned disabled_periods = 999, unsupported_sources = 0x30, control_mask = 0x7f;
constexpr std::array<unsigned, 4> unsupported_registers{0x33, 0x34, 0x35, 0x2f};
constexpr ExecutionContext context{.pc = 0xe1ca, .ccr = 0x69};

void divisors_and_overflow() {
  constexpr std::array<unsigned, 8> divisors{1, 2, 4, 8, 16, 32, 64, 128};
  for (unsigned code = 0; code < divisors.size(); ++code) {
    Hcs08Tpm timer(contract, 0);
    const auto divisor = divisors[code];
    timer.write({.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(bus | code)});
    timer.advance_bus((static_cast<std::uint64_t>(cycle) * divisor) - 1);
    require(timer.state().counter == cycle - 1 && !timer.state().overflow &&
                timer.state().prescaler_phase == divisor - 1,
            "Every primary prescale tap reaches FFFF one input edge before its first overflow");
    timer.advance_bus(1);
    require(timer.state().counter == 0 && timer.state().prescaler_phase == 0 &&
                timer.state().overflow && !timer.interrupt_requested(),
            "The exact next edge wraps to zero and sets TOF without inventing an enabled IRQ");
    timer.write({.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(bus | code)});
    require(timer.state().overflow, "Writing zero without reading TOF cannot acknowledge it");
    require(timer.read(Hcs08Tpm::status) == (flag | bus | code),
            "TOF read arms its clear sequence");
    timer.advance_bus(static_cast<std::uint64_t>(cycle) * divisor);
    timer.write({.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(bus | code)});
    require(timer.state().overflow,
            "A second overflow invalidates the earlier read/clear sequence");
    static_cast<void>(timer.read(Hcs08Tpm::status));
    timer.write(
        {.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(flag | bus | code)});
    require(timer.state().overflow, "Writing one never clears TOF");
    timer.write(
        {.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(irq | bus | code)});
    require(!timer.state().overflow && !timer.interrupt_requested(),
            "Read-then-zero clears the flag");
    timer.advance_bus((static_cast<std::uint64_t>(cycle) * divisor) + divisor);
    require(timer.state().counter == 1 && timer.interrupt_requested(),
            "A subsequent overflow drives the explicit TOF/TOIE request, independently of CPU I");
    const auto copy = timer;
    timer.advance_bus(divisor);
    require(copy.state().counter == 1 && timer.state().counter == 2,
            "Copies preserve their own counter, flag and divider histories");
  }
  Hcs08Tpm timer(contract, 0);
  timer.write({.address = Hcs08Tpm::status, .value = bus});
  timer.advance_bus(std::numeric_limits<std::uint64_t>::max());
  require(timer.state().counter == cycle - 1 && timer.state().overflow,
          "Maximal batched input never overflows host addition or drops the final count");
}

void coherent_reads() {
  constexpr unsigned first = 0x12ff;
  constexpr unsigned old_high = 0x12;
  constexpr unsigned next_high = 0x13;
  constexpr unsigned low_mask = 0xff;
  for (const bool high_first : {false, true}) {
    Hcs08Tpm timer(contract, 0);
    timer.write({.address = Hcs08Tpm::status, .value = bus});
    timer.advance_bus(first);
    const auto a = high_first ? Hcs08Tpm::count_high : Hcs08Tpm::count_low;
    const auto b = high_first ? Hcs08Tpm::count_low : Hcs08Tpm::count_high;
    require(timer.read(a) == (high_first ? old_high : low_mask), "First byte latches both bytes");
    timer.advance_bus(1);
    const auto cut = timer;
    require(timer.read(a) == (high_first ? old_high : low_mask) &&
                timer.read(b) == (high_first ? low_mask : old_high) && !timer.state().read_latch,
            "Repeated same-byte reads retain the old coherent pair across12FF->1300");
    require(timer.read(Hcs08Tpm::count_high) == next_high && timer.read(Hcs08Tpm::count_low) == 0,
            "Only reading both old bytes admits the current counter into a new latch");
    auto fresh = cut;
    require(fresh.read(b) == (high_first ? low_mask : old_high) && !fresh.state().read_latch,
            "A fresh owner resumes a partially consumed coherent word");
    static_cast<void>(timer.read(a));
    timer.advance_bus(2);
    timer.write({.address = Hcs08Tpm::status, .value = bus});
    require(!timer.state().read_latch && timer.read(Hcs08Tpm::count_low) == 2,
            "Any SC write resets read coherency without resetting the counter");
    timer.write({.address = b, .value = low_mask});
    require(timer.state().counter == 0 && !timer.state().read_latch &&
                !timer.state().prescaler_phase,
            "Any counter-byte write clears the whole counter and latch without guessing phase");
  }
}

void phase_and_strictness() {
  require(rejected([] {
            const Hcs08Tpm invalid(Hcs08TpmContract::unqualified);
          }).contains("run-mode bus-edge contract"),
          "The component cannot silently assume a clock/device contract");
  Hcs08Tpm timer(contract);
  timer.context(context);
  require(timer.state() == Hcs08TpmState{},
          "Reset disables timer and leaves divider phase unobserved");
  timer.advance_bus(disabled_periods);
  timer.write({.address = Hcs08Tpm::status, .value = bus | 7});
  const auto unknown = timer.state();
  require(rejected([&] { timer.advance_bus(1); }).contains("unknown prescaler phase") &&
              timer.state() == unknown,
          "Unknown phase cannot produce an invented first counter value");
  timer.write({.address = Hcs08Tpm::status, .value = 0});
  for (unsigned phase = 0; phase < 128; ++phase) {
    timer.observe_bus_phase(phase);
    timer.write({.address = Hcs08Tpm::status, .value = bus | 7});
    timer.advance_bus(128 - phase);
    require(timer.state().counter == phase + 1 && timer.state().prescaler_phase == 0,
            "All128 declared initial phases reach their next divider carry at the expected edge");
    timer.write({.address = Hcs08Tpm::status, .value = 0});
  }
  timer.reset(0);
  require(timer.state() == Hcs08TpmState{.prescaler_phase = 0},
          "Explicit reset clears all live history");
  timer.write({.address = Hcs08Tpm::status, .value = bus});
  const auto running = timer.state();
  for (unsigned divisor = 1; divisor < 8; ++divisor) {
    require(rejected([&] {
              timer.write(
                  {.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(bus | divisor)});
            }).contains("divider history") &&
                timer.state() == running,
            "Every changed live divider refuses before control/coherency/phase mutation");
  }
  require(rejected([&] { timer.observe_bus_phase(0); }).contains("disabled timer") &&
              timer.state() == running,
          "A live observation cannot overwrite divider history");
  timer.reset(0);
  for (unsigned value = 0; value < values; ++value) {
    Hcs08Tpm candidate(contract, 0);
    candidate.context(context);
    const auto before = candidate.state();
    if ((value & unsupported_sources) == 0) {
      candidate.write({.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(value)});
      require(candidate.read(Hcs08Tpm::status) == (value & control_mask),
              "TOF is read-only except its clear sequence");
    } else {
      const auto error = rejected([&] {
        candidate.write({.address = Hcs08Tpm::status, .value = static_cast<std::uint8_t>(value)});
      });
      require(error.contains("write address=0x0030 pc=0xe1ca ccr=0x0069") &&
                  candidate.state() == before,
              "Unsupported sources/PWM reject before all state changes");
    }
  }
  const auto before = timer.state();
  require(rejected([&] { timer.reset(128); }).contains("divider phase") && timer.state() == before,
          "Invalid phase reset fails atomically");
  for (const unsigned address : unsupported_registers) {
    require(rejected([&] {
              timer.write({.address = static_cast<std::uint16_t>(address)});
            }).contains("unimplemented") &&
                timer.state() == before,
            "Unimplemented modulo/channel writes retain complete state");
  }
}
} // namespace

int main() {
  try {
    divisors_and_overflow();
    coherent_reads();
    phase_and_strictness();
    std::cout << "Qualified TPM bus edges preserve count, coherency and overflow history.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
