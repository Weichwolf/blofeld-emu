// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_icg.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace hi = hcs08_icg;
constexpr unsigned own_trim = 0x5a, control1 = 0x28, control2 = 0x30;
constexpr unsigned own_hx = 0x1234, own_sp = 0x47c, own_ccr = 0x79;
constexpr unsigned retired_setup = 6, failing_pc = entry + 12;
constexpr unsigned reset_control = 0x44, reset_filter = 0xc0;
constexpr unsigned unresolved_refs = 0xdf;
constexpr auto chip = Hcs08IcgContract::candidate_gt8a_no_clock_events;
constexpr auto warm = Hcs08IcgReset::warm;

// Own immediate/direct ISA probe, not extracted manufacturer program bytes.
constexpr std::array<std::uint8_t, 14> code{
    0xa6,
    own_trim,
    0xb7,
    hi::trim, // LDA #5A; STA ICGTRM.
    0xa6,
    control1,
    0xb7,
    hi::control1, // LDA #28; STA ICGC1.
    0xa6,
    control2,
    0xb7,
    hi::control2, // LDA #30; STA ICGC2.
    0xb6,
    hi::status1}; // LDA ICGS1 must not return fake LOCK/DCOS.

class ClockMemory final : public Memory {
public:
  Hcs08Icg icg{chip, warm};

  void begin_instruction(const ExecutionContext value) override {
    Memory::begin_instruction(value);
    icg.context(value);
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (address < hi::control1 || address > hi::trim) { return Memory::read(address); }
    const auto value = icg.read(address);
    accesses.push_back({.address = address, .value = value, .write = false});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (address < hi::control1 || address > hi::trim) {
      Memory::write(address, value);
      return;
    }
    icg.write({.address = address, .value = value});
    accesses.push_back({.address = address, .value = value, .write = true});
    ++generation;
  }
};

void expected(const ClockMemory &memory, const Hcs08 &cpu, const unsigned steps) {
  constexpr std::array<unsigned, 6> accumulators{
      own_trim, own_trim, control1, control1, control2, control2};
  require(cpu.state() == Hcs08State{.pc = static_cast<std::uint16_t>(entry + (steps * 2)),
                                    .hx = own_hx,
                                    .sp = own_sp,
                                    .a = static_cast<std::uint8_t>(accumulators.at(steps - 1)),
                                    .ccr = own_ccr} &&
              memory.retired == steps && !cpu.faulted(),
          "Every retired instruction has independent complete register and CCR expectations");
  const Hcs08IcgState clock{
      .control1_request = static_cast<std::uint8_t>(steps >= 4 ? control1 : reset_control),
      .control1_known = static_cast<std::uint8_t>(steps >= 4 ? unresolved_refs : byte_mask),
      .control2_request = static_cast<std::uint8_t>(steps >= 6 ? control2 : 0),
      .trim = steps >= 2 ? std::optional<std::uint8_t>(own_trim) : std::nullopt,
      .filter = steps >= 4 ? std::nullopt : std::optional<std::uint16_t>(reset_filter),
      .control1_written = steps >= 4,
      .control2_written = steps >= 6};
  require(memory.icg.state() == clock,
          "Only real native stores establish trim/control requests; no clock rate is supplied");
}

void run_suffix(ClockMemory &memory, Hcs08 &cpu, const unsigned start) {
  for (unsigned n = start; n < retired_setup; ++n) {
    cpu.step();
    expected(memory, cpu, n + 1);
  }
  const auto before = memory.icg.state();
  const auto failure = rejected([&] { cpu.step(); });
  require(failure.contains("HCS08 ICG read address=0x004a pc=0x800c ccr=0x0079") &&
              failure.contains("live clock/status evolution is unqualified") && cpu.faulted() &&
              memory.retired == retired_setup && memory.icg.state() == before,
          "Actual native status load stops before retirement without manufacturing a ready clock");
  require(rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
              memory.icg.state() == before,
          "A faulted CPU cannot execute through the missing clock owner");
}

void native_and_restores() {
  struct Saved {
    ClockMemory memory;
    Hcs08Checkpoint cpu;
    Hcs08IcgState icg;
    std::vector<Access> accesses;
    unsigned steps;
  };

  ClockMemory memory;
  memory.install(entry, code);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry, .hx = own_hx, .sp = own_sp, .ccr = own_ccr});
  constexpr std::array<unsigned, 2> cuts{2, 4};
  std::vector<Saved> checkpoints;
  checkpoints.reserve(cuts.size());
  for (unsigned n = 0; n < retired_setup; ++n) {
    cpu.step();
    expected(memory, cpu, n + 1);
    if (n + 1 != cuts[0] && n + 1 != cuts[1]) { continue; }
    checkpoints.push_back({.memory = memory,
                           .cpu = cpu.checkpoint(),
                           .icg = memory.icg.state(),
                           .accesses = memory.accesses,
                           .steps = n + 1});
  }
  run_suffix(memory, cpu, retired_setup);
  for (const auto &saved : checkpoints) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      auto restored_memory = saved.memory;
      Hcs08 restored_cpu(restored_memory, BLOFELD_HCS08_SLA);
      restored_cpu.restore_checkpoint(saved.cpu);
      expected(restored_memory, restored_cpu, saved.steps);
      run_suffix(restored_memory, restored_cpu, saved.steps);
      require(saved.memory.retired == saved.steps && memory.retired == retired_setup &&
                  saved.memory.icg.state() == saved.icg && saved.memory.accesses == saved.accesses,
              "Fresh CPU/ICG/memory continuations preserve both distinct source histories");
    }
  }
  unsigned stores = 0;
  constexpr std::array<std::uint16_t, 3> addresses{hi::trim, hi::control1, hi::control2};
  constexpr std::array<unsigned, 3> values{own_trim, control1, control2};
  for (const auto access : memory.accesses) {
    if (!access.write) { continue; }
    require(stores < addresses.size() && access.address == addresses[stores] &&
                access.value == values[stores],
            "Independent actual-write order/value oracle covers every ICG store");
    ++stores;
  }
  require(stores == addresses.size() && memory.context.pc == failing_pc,
          "Three native stores precede the genuine unqualified status transaction");
  memory.icg.reset(warm);
  require(memory.icg.state() == Hcs08IcgState{.trim = own_trim},
          "Warm ICG reset preserves the actual CPU-written trim and resets control/filter state");
}
} // namespace

int main() {
  try {
    native_and_restores();
    std::cout << "HCS08 ICG: own native six-instruction setup, four fresh joint continuations "
                 "and five strict live-status failures passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
