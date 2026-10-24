// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_run_clock.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "blofeld/private_file.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_flash_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using blofeld::test::hcs08::rejected;
using blofeld::test::hcs08::require;
namespace hp = blofeld::test::hcs08_programmer;
namespace cmds = blofeld::test::hcs08_bdc;
namespace nvm = blofeld::test::hcs08_nvm;
namespace hf = hcs08_flash;
constexpr auto chip = Hcs08FlashContract::candidate_gt8a;
constexpr auto run_contract = Hcs08RunContract::reference_instruction_retirement;
constexpr unsigned byte_mask = 255, vector = 0xfffe, trim_address = 0xffbe;
constexpr unsigned upload_bytes = 164, reset_known = 0x68, load_known = 0xee;
constexpr unsigned running_status = 0x88, active_status = 0xc8, command_count = 171;
constexpr unsigned strict_pc = 0xb2, final_period = 26, prefix_periods = 22, middle_period = 24;
constexpr unsigned loaded_hx = 0x78, loaded_sp = 0x77, written_divider = 0x1e;
constexpr unsigned reset_status = 0xc0, options = 0xc2, final_ccr = 0x6c;
constexpr unsigned warm_pc = 0x1234, warm_hx = 0x3322, warm_sp = 0x4567, warm_a = 0xa5;
constexpr unsigned second_pc = 0x5678, second_hx = 0x44bb, second_sp = 0x2456, second_a = 0x5a;
constexpr unsigned protected_vectors = 0xfc, stack = 0x47c;

struct Seed {
  Hcs08State warm;
  std::uint8_t protection;
  std::uint8_t trim;
};

constexpr std::array<Seed, 2> seeds{
    {{.warm = {.pc = warm_pc, .hx = warm_hx, .sp = warm_sp, .a = warm_a, .ccr = byte_mask},
      .protection = byte_mask,
      .trim = warm_a},
     {.warm = {.pc = second_pc,
               .hx = second_hx,
               .sp = second_sp,
               .a = second_a,
               .ccr = Hcs08State::fixed_ccr},
      .protection = protected_vectors,
      .trim = second_a}}};

struct Saved {
  hp::Memory memory;
  Hcs08Checkpoint cpu;
  Hcs08BdcCheckpoint bdc;
  Hcs08RunCheckpoint clock;
};

struct Target {
  hp::Memory memory;
  Hcs08 cpu{memory, BLOFELD_HCS08_SLA};
  cmds::Memory mapping{memory};

  explicit Target(const Seed seed) : memory(Hcs08IdleLaunch::unqualified, 5) {
    auto image = nvm::initial();
    image.data[hf::nv_protection - hf::first] = seed.protection;
    image.data[trim_address - hf::first] = seed.trim;
    image.known.reset(vector - hf::first);
    image.known.reset(vector + 1 - hf::first);
    memory.flash = Hcs08Flash(image, chip); // No observed NVM operating frequency.
    memory.bytes.fill(hp::data_canary);
    memory.known.reset(); // Actual BDC writes must establish every uploaded RAM byte.
    cpu.set_state(seed.warm);
    cpu.reset(Hcs08ResetEntry::active_background);
  }

  explicit Target(const Saved &saved) : memory(saved.memory) { cpu.restore_checkpoint(saved.cpu); }
};

struct Machine {
  Target target;
  Hcs08Bdc bdc{target.cpu, target.mapping, chip, Hcs08BdcReset::bkgd_held_low};
  Hcs08RunClock clock{target.memory, bdc, run_contract};

  explicit Machine(const Seed seed) : target(seed) {}

  explicit Machine(const Saved &saved) : target(saved) {
    bdc.restore_checkpoint(saved.bdc);
    clock.restore_checkpoint(saved.clock);
  }

  [[nodiscard]] Saved checkpoint() const {
    return {.memory = target.memory,
            .cpu = target.cpu.checkpoint(),
            .bdc = bdc.checkpoint(),
            .clock = clock.checkpoint()};
  }
};

void registers(std::ostream &trace, const Hcs08State state) {
  trace << " pc=" << state.pc << " pc-known=" << state.pc_known << " hx=" << state.hx
        << " sp=" << state.sp << " a=" << static_cast<unsigned>(state.a)
        << " ccr=" << static_cast<unsigned>(state.ccr)
        << " known=" << static_cast<unsigned>(state.ccr_known);
}

void ram(const hp::Memory &memory, const std::span<const std::uint8_t> program) {
  for (unsigned address = 0; address < memory.bytes.size(); ++address) {
    const bool uploaded = address >= hp::ram_first && address < hp::algorithm + hp::algorithm_bytes;
    unsigned expected = hp::data_canary;
    if (uploaded) { expected = address < hp::algorithm ? 0U : program[address - hp::algorithm]; }
    if (memory.known[address] != uploaded || memory.bytes[address] != expected) {
      throw Fault("Original reset-entry RAM ownership mismatch address=" + hex(address));
    }
  }
}

std::uint8_t status(Hcs08Bdc &bdc) {
  constexpr std::array<std::uint8_t, 1> read{0xe4};
  cmds::command(bdc, read);
  return bdc.read_reply();
}

void prepare(Machine &machine,
             const Seed seed,
             const std::span<const std::uint8_t> program,
             std::ostream &trace) {
  auto &cpu = machine.target.cpu;
  auto &memory = machine.target.memory;
  const auto after_reset = cpu.state();
  require(
      after_reset == Hcs08State{.pc = 0,
                                .hx = static_cast<std::uint16_t>(seed.warm.hx & byte_mask),
                                .sp = byte_mask,
                                .a = seed.warm.a,
                                .ccr_known = reset_known,
                                .ccr = reset_known,
                                .pc_known = false} &&
          memory.known.none() && memory.accesses.empty() && !memory.resetting,
      "Actual active reset precedes vector/RAM access and retains only documented warm knowledge");
  trace << "RESET";
  registers(trace, after_reset);
  trace << " status=" << static_cast<unsigned>(machine.bdc.status())
        << " ram-known=" << memory.known.count() << '\n';
  machine.clock.advance();
  constexpr std::array<std::uint8_t, 1> background{0x90};
  cmds::command(machine.bdc, background);
  require(status(machine.bdc) == active_status && memory.retired == 0 &&
              machine.clock.state().executing_periods == 0,
          "Real BACKGROUND/status commands preserve genuinely held reset CPU");
  const auto trim = cmds::read(machine.bdc, trim_address);
  require(trim == seed.trim, "Actual BDC trim read retains supplied pre-update calibration input");
  trace << "TRIM value=" << static_cast<unsigned>(trim)
        << " commands=" << machine.bdc.state().serviced << '\n';
  for (unsigned n = 0; n < upload_bytes; ++n) {
    const auto byte =
        n < hp::algorithm - hp::ram_first ? 0U : program[n - (hp::algorithm - hp::ram_first)];
    cmds::write(machine.bdc,
                {.address = static_cast<std::uint16_t>(hp::ram_first + n),
                 .value = static_cast<std::uint8_t>(byte)});
  }
  ram(memory, program);
  require(cpu.state() == after_reset,
          "Every actual BDC RAM write preserves reset PC/CCR knowledge");
  trace << "UPLOAD bytes=" << upload_bytes << " known=" << memory.known.count()
        << " commands=" << machine.bdc.state().serviced << " pc-known=" << cpu.state().pc_known
        << '\n';
  constexpr std::array<std::uint8_t, 3> pc{0x4b, 0, hp::algorithm};
  constexpr std::array<std::uint8_t, 3> sp{0x4f, 4, 0x7c};
  constexpr std::array<std::uint8_t, 1> go{0x08};
  cmds::command(machine.bdc, pc);
  cmds::command(machine.bdc, sp);
  cmds::command(machine.bdc, go);
  require(
      status(machine.bdc) == running_status && cpu.state().pc_known &&
          cpu.state().ccr_known == reset_known && cpu.state().sp == stack &&
          machine.bdc.state().serviced == command_count,
      "Actual WRITE_PC/WRITE_SP/GO starts original RAM code without WRITE_CCR or guessed flags");
  trace << "START";
  registers(trace, cpu.state());
  trace << " commands=" << machine.bdc.state().serviced << " status=" << running_status << '\n';
  memory.accesses.clear();
}

void prefix(Machine &machine, const Seed seed, std::ostream &trace) {
  constexpr std::array<unsigned, 8> pcs{0xa2, 0xa3, 0xa5, 0xa8, 0xaa, 0xad, 0xaf, strict_pc};
  constexpr std::array<unsigned, 8> periods{2, 4, 6, 10, 12, 16, 18, prefix_periods};
  constexpr std::array<unsigned, 6> loaded{
      written_divider, written_divider, 2, 2, byte_mask, byte_mask};
  auto &memory = machine.target.memory;
  for (unsigned n = 0; n < pcs.size(); ++n) {
    while (memory.retired == n) {
      require(machine.clock.state().executing_periods < periods[n],
              "Independent native prefix timing bound");
      machine.clock.advance();
    }
    const auto expected_a = n < 2 ? seed.warm.a : loaded[n - 2];
    const Hcs08State expected{.pc = static_cast<std::uint16_t>(pcs[n]),
                              .hx = loaded_hx,
                              .sp = static_cast<std::uint16_t>(n == 0 ? stack : loaded_sp),
                              .a = static_cast<std::uint8_t>(expected_a),
                              .ccr_known = load_known,
                              .ccr = static_cast<std::uint8_t>(n < 6 ? reset_known : final_ccr)};
    require(machine.target.cpu.state() == expected && memory.retired == n + 1 &&
                machine.clock.state().retired_instructions == n + 1 &&
                machine.clock.state().executing_periods == periods[n],
            "Each original instruction has an independent full register/CCR/period expectation");
    trace << "STEP n=" << memory.retired << " periods=" << machine.clock.state().executing_periods;
    registers(trace, machine.target.cpu.state());
    trace << '\n';
  }
  using blofeld::test::hcs08::Access;
  require(
      memory.accesses ==
              std::vector<Access>{{.address = hf::divider, .value = written_divider, .write = true},
                                  {.address = hf::options, .value = 2, .write = true},
                                  {.address = hf::protection, .value = byte_mask, .write = true}} &&
          memory.flash.read(hf::divider) == (hf::empty | written_divider) &&
          memory.flash.read(hf::options) == options &&
          memory.flash.read(hf::status) == reset_status &&
          memory.flash.read(hf::protection) == seed.protection && !memory.options.written(),
      "Only actual FCDIV loads; original CPU FOPT/FPROT attempts remain ignored; SOPT is not yet "
      "written");
  for (const auto &access : memory.accesses) {
    trace << "ACCESS address=" << access.address << " value=" << access.value
          << " write=" << access.write << '\n';
  }
}

std::string
fault(Machine &machine, const Hcs08FlashImage &image, const std::span<const std::uint8_t> program) {
  auto &memory = machine.target.memory;
  const auto diagnostic = rejected([&] {
    while (true) {
      require(machine.clock.state().executing_periods < final_period,
              "Actual FSTAT attempt remains bounded");
      machine.clock.advance();
    }
  });
  require(diagnostic.contains("pc=0x00b2") && diagnostic.contains("address=0x1825") &&
              diagnostic.contains("unqualified idle FCBEF") && machine.clock.faulted() &&
              machine.bdc.faulted() && machine.target.cpu.faulted() && memory.retired == 8 &&
              machine.clock.state().retired_instructions == 8 &&
              machine.clock.state().executing_periods == final_period &&
              machine.clock.state().offered_periods == final_period + 1 &&
              machine.clock.state().remaining_periods == 0 && !machine.bdc.state().active &&
              memory.transfers == 0 && memory.completions == 0 && memory.array_writes == 0 &&
              memory.command_writes == 0 && memory.status_writes == 0 && memory.sopt_writes == 0 &&
              !memory.transfer_due && !memory.complete_due && memory.flash.image() == image &&
              !memory.flash.state().active &&
              memory.flash.state().buffer == Hcs08FlashBuffer::empty,
          "Actual default FSTAT refusal quarantines all owners without an NVM write, timing guess "
          "or retirement");
  require(machine.target.cpu.state() == Hcs08State{.pc = strict_pc,
                                                   .hx = loaded_hx,
                                                   .sp = loaded_sp,
                                                   .a = byte_mask,
                                                   .ccr_known = load_known,
                                                   .ccr = final_ccr},
          "Fault keeps actual native state and unknown half-carry/carry flags");
  ram(memory, program);
  const auto before = machine.clock.state();
  require(rejected([&] { machine.clock.advance(); }).contains("coordinated recovery") &&
              rejected([&] {
                static_cast<void>(machine.bdc.checkpoint());
              }).contains("coordinated recovery") &&
              machine.clock.state() == before,
          "Repeated requests cannot resume a quarantined original instruction");
  std::ostringstream trace;
  trace << "ERROR " << diagnostic << '\n'
        << "FAULT n=" << memory.retired << " periods=" << machine.clock.state().executing_periods
        << " offered=" << machine.clock.state().offered_periods;
  registers(trace, machine.target.cpu.state());
  trace << " commands=" << machine.bdc.state().serviced << " transfers=" << memory.transfers
        << " completions=" << memory.completions << '\n';
  return trace.str();
}

void run(const unsigned index,
         const Seed seed,
         const std::span<const std::uint8_t> program,
         std::ostream &trace) {
  Machine machine(seed);
  const auto image = machine.target.memory.flash.image();
  trace << "CASE index=" << index << " a=" << static_cast<unsigned>(seed.warm.a)
        << " hx=" << seed.warm.hx << " sp=" << seed.warm.sp
        << " protection=" << static_cast<unsigned>(seed.protection)
        << " trim=" << static_cast<unsigned>(seed.trim) << '\n';
  prepare(machine, seed, program, trace);
  prefix(machine, seed, trace);
  const auto boundary = machine.checkpoint();
  for (unsigned n = 0; n < 2; ++n) { machine.clock.advance(); }
  require(machine.clock.state().executing_periods == middle_period &&
              machine.clock.state().remaining_periods == 2 && machine.target.memory.retired == 8 &&
              machine.target.memory.flash.image() == image,
          "Middle-budget snapshot precedes any effects of the actual FSTAT store");
  const auto middle = machine.checkpoint();
  trace << "PENDING periods=" << machine.clock.state().executing_periods
        << " remaining=" << machine.clock.state().remaining_periods
        << " retired=" << machine.target.memory.retired << '\n';
  const auto suffix = fault(machine, image, program);
  trace << "ATTEMPT index=0\n" << suffix;
  for (unsigned cut = 0; cut < 2; ++cut) {
    for (unsigned replay = 0; replay < 2; ++replay) {
      const auto &saved = cut == 0 ? boundary : middle;
      Machine restored(saved);
      const auto observed = fault(restored, image, program);
      require(observed == suffix && restored.target.cpu.state() == machine.target.cpu.state() &&
                  restored.clock.state() == machine.clock.state() &&
                  restored.bdc.state() == machine.bdc.state() &&
                  restored.target.memory.bytes == machine.target.memory.bytes &&
                  restored.target.memory.known == machine.target.memory.known &&
                  restored.target.memory.flash.state() == machine.target.memory.flash.state(),
              "Every fresh joint CPU/memory/BDC/clock restore reproduces complete fault suffix and "
              "endpoint");
      trace << "ATTEMPT index=" << 1 + (cut * 2) + replay << '\n' << observed;
      trace << "REPLAY cut=" << cut << " index=" << replay << " identical=1\n";
    }
  }
  trace << "CASE_END index=" << index << " retired=" << machine.target.memory.retired
        << " commands=" << machine.bdc.state().serviced << '\n';
}
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected pinned original programmer and private trace path");
    const auto program = hp::read_input(arguments[1], hp::algorithm_bytes);
    std::ostringstream trace;
    for (unsigned index = 0; index < seeds.size(); ++index) {
      run(index, seeds[index], program, trace);
    }
    trace << "END cases=2 instructions=16 faults=10 replays=8\n";
    const auto text = trace.str();
    write_private_file(arguments[2],
                       {reinterpret_cast<const std::uint8_t *>(text.data()), text.size()});
    std::cout << "Original HCS08 reset entry preserves partial CCR and default FSTAT refusal.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
