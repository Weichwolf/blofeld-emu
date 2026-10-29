// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned target_value = 0x123456;
constexpr unsigned ram = 0x100;
constexpr unsigned ram_target = 0x140;
constexpr unsigned other_target = 0x160;
constexpr unsigned rom_words = 128;
constexpr unsigned rom_target = boot + 80;
constexpr unsigned first_retirement = 7;
constexpr unsigned target_retirement = 11;
constexpr unsigned period = 45;

DspFetchRunInputs contract() {
  auto result = fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

void prepare(DspMachine &machine, const std::vector<std::uint32_t> &words) {
  machine.bus().install_rom({.start = boot, .words = words});
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

const DspPipelineCycle &stages(const DspFetchCycleResult &cycle) {
  if (!cycle.pipeline) { throw Fault("Expected actual automatic pipeline state"); }
  return *cycle.pipeline;
}

DspFetchCycleResult tick(DspFetchRun &run, DspMachine &machine, std::ostream &trace) {
  const auto cycle = run.advance_cycle(machine, fetch_clock::input());
  require(cycle.period_ticks == period && cycle.completion_ticks == cycle.completed_cycles * period,
          "Every actual pipeline period matches the independent reset PLL clock");
  trace << "cycle=" << cycle.completed_cycles << " pc=" << hex(machine.cpu().state().pc, 6);
  if (cycle.issued_address) { trace << " F1=" << hex(*cycle.issued_address, 6); }
  if (cycle.bus.fetch) {
    const auto &fetched = *cycle.bus.fetch;
    require(fetched.used != 0, "Actual F2 completion retains its real program word");
    const auto word = fetched.words.at(fetched.used - 1);
    trace << " F2=" << hex(word.address, 6) << ':' << (word.data ? hex(*word.data, 6) : "unknown");
  }
  const auto &phase = stages(cycle);
  if (phase.retirement_published) {
    if (!phase.execute2) { throw Fault("Actual retirement lost its E2 word"); }
    trace << " E2=" << hex(phase.execute2->address, 6);
  }
  trace << '\n';
  return cycle;
}

void state(const DspMachine &machine, const DspDebugState &expected) {
  require(machine.cpu().debug_state() == expected,
          "Full native CPU/retained state differs from independent literal JMP/NOP result at " +
              hex(expected.cpu.pc, 6));
}

void ram_nops(DspMachine &machine, const unsigned start) {
  constexpr unsigned count = 32;
  for (unsigned i = 0; i < count; ++i) { machine.bus().write({.address = start + i, .value = 0}); }
}

enum class Target : std::uint8_t { rom, ram, sequential, self };

void one_case(const unsigned prefix, const Target kind, std::ostream &trace) {
  DspMachine machine;
  const unsigned jump = boot + prefix;
  unsigned target = rom_target;
  if (kind == Target::ram) {
    target = ram;
    ram_nops(machine, target);
  }
  if (kind == Target::sequential) { target = jump + 2; }
  if (kind == Target::self) { target = jump; }
  std::vector<std::uint32_t> words(rom_words, 0);
  words.at(prefix) = absolute_jump;
  words.at(prefix + 1) = target;
  prepare(machine, words);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  const bool self = kind == Target::self;
  const auto last = prefix + (self ? 15U : target_retirement);
  unsigned jumps = 0;
  unsigned retired = 0;
  while (run.completed_cycles() < last) {
    const auto cycle = tick(run, machine, trace);
    const auto &phase = stages(cycle);
    if (phase.ag1 && phase.ag1->address == jump) {
      require(phase.ag1_extension == DspProgramWord{.address = jump + 1, .value = target},
              "Absolute AG1 must retain the adjacent genuinely fetched extension");
    }
    if (phase.execute1 && phase.execute1->address == jump) {
      require(phase.e1_extension == DspProgramWord{.address = jump + 1, .value = target} &&
                  phase.execution_captured,
              "Native E1 consumes the same actual opcode/extension pair");
    }
    if (!phase.retirement_published) { continue; }
    if (!phase.execute2) { throw Fault("Expected actual native E2"); }
    require(phase.execute2->address == expected.cpu.pc,
            "Every native retirement has its literal PC");
    if (phase.execute2->address == jump) {
      require(cycle.completed_cycles == prefix + first_retirement + (jumps * 4),
              "Long absolute JMP retains the independently expected four-period spacing");
      expected.cpu.pc = target;
      expected.native_cycles += 4;
      ++jumps;
    } else {
      ++expected.cpu.pc;
      ++expected.native_cycles;
    }
    ++expected.native_instructions;
    ++retired;
    state(machine, expected);
  }
  require(jumps == (self ? 3U : 1U) && retired == prefix + (self ? 3U : 2U),
          "Cold,filled,sequential,self and full24-bit target paths retain exact native counts");
}

void snapshots(std::ostream &trace) {
  for (unsigned cut = 1; cut <= target_retirement; ++cut) {
    DspMachine machine;
    std::vector<std::uint32_t> words(rom_words, 0);
    words[0] = absolute_jump;
    words[1] = rom_target;
    prepare(machine, words);
    auto expected = machine.cpu().debug_state();
    expected.cpu.pc = rom_target + 1;
    expected.native_cycles += 5;
    expected.native_instructions += 2;
    DspFetchRun run(contract(), machine);
    while (run.completed_cycles() < cut) { static_cast<void>(tick(run, machine, trace)); }
    const auto saved = run.checkpoint(machine);
    for (unsigned replay = 0; replay < 2; ++replay) {
      run.restore(saved, machine);
      while (run.completed_cycles() < target_retirement) {
        static_cast<void>(tick(run, machine, trace));
      }
      state(machine, expected);
    }
  }
}

void actual_words(const bool replace_target, std::ostream &trace) {
  DspMachine machine;
  std::vector<std::uint32_t> words(rom_words, 0);
  words[0] = absolute_jump;
  words[1] = ram;
  ram_nops(machine, ram);
  ram_nops(machine, ram_target);
  ram_nops(machine, other_target);
  machine.bus().write({.address = ram, .value = absolute_jump});
  machine.bus().write({.address = ram + 1, .value = ram_target});
  prepare(machine, words);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  constexpr unsigned after_extension_f2 = 7;
  constexpr unsigned after_target_f1 = 9;
  constexpr unsigned final_retirement = 15;
  while (run.completed_cycles() < after_extension_f2) {
    static_cast<void>(tick(run, machine, trace));
  }
  machine.bus().write({.address = ram + 1, .value = other_target});
  while (run.completed_cycles() < after_target_f1) { static_cast<void>(tick(run, machine, trace)); }
  if (replace_target) {
    machine.bus().write({.address = ram_target, .value = immediate_x0});
    machine.bus().write({.address = ram_target + 1, .value = target_value});
  }
  while (run.completed_cycles() < final_retirement) {
    static_cast<void>(tick(run, machine, trace));
  }
  expected.cpu.pc = ram_target + (replace_target ? 2U : 1U);
  expected.native_instructions += 3;
  expected.native_cycles += replace_target ? 10U : 9U;
  if (replace_target) { expected.cpu.x = target_value; }
  state(machine, expected);
  require(
      machine.bus().inspect_program(ram + 1) == other_target,
      "Native execution used the actual old F2 extension despite different current RAM contents");
}

void refused_pair(const unsigned variant) {
  DspBus bus;
  bus.write({.address = ram, .value = absolute_jump});
  bus.write({.address = ram + 1, .value = ram_target});
  DspProgramFetch fetch;
  fetch.start(ram, reset_omr);
  for (unsigned i = 0; i < 2; ++i) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto opcode = fetch.queued(0);
  const auto extension = fetch.queued(1);
  if (!opcode || !extension) { throw Fault("Own branch pair needs two genuine F2 reads"); }
  const DspBranchDecision valid{.word = opcode->word,
                                .extension = extension->word,
                                .omr = reset_omr,
                                .next_pc = ram_target,
                                .taken = true};
  auto invalid = valid;
  auto identity = opcode->occurrence;
  if (variant == 0) {
    invalid.extension.reset();
  } else if (variant == 1) {
    invalid.extension->value ^= 1U;
  } else if (variant == 2) {
    ++invalid.extension->address;
  } else if (variant == 3) {
    invalid.word.value ^= 1U;
  } else if (variant == 4) {
    ++identity;
  } else if (variant == 5) {
    ++invalid.next_pc;
  } else {
    invalid.taken = false;
  }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch(invalid, identity);
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("P:") && message.contains("pc=") && message.contains("omr=") &&
                fetch.view() == before,
            "A false absolute pair/target/identity leaves the queue unchanged");
    refused = true;
  }
  require(refused, "An invalid absolute branch forecast must not mutate actual fetch ownership");
  fetch.predict_branch(valid, opcode->occurrence);
  require(fetch.view().next_address == ram_target && fetch.view().used == 2,
          "A corrected forecast still owns both genuine reads after each refused request");
}

void missing_extension() {
  DspBus bus;
  bus.write({.address = ram, .value = absolute_jump});
  DspProgramFetch fetch;
  fetch.start(ram, reset_omr);
  fetch.issue();
  fetch.capture(bus);
  const auto word = fetch.queued(0);
  if (!word) { throw Fault("Expected actual opcode F2"); }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch({.word = word->word,
                          .extension = DspProgramWord{.address = ram + 1, .value = ram_target},
                          .omr = reset_omr,
                          .next_pc = ram_target,
                          .taken = true},
                         word->occurrence);
  } catch (const Fault &) { refused = true; }
  require(refused && fetch.view() == before,
          "A supplied but unfetched extension can never become a native branch response");
}

void unknown_target(std::ostream &trace) {
  DspMachine machine;
  std::vector<std::uint32_t> words(rom_words, 0);
  constexpr unsigned unmapped = 0x1000;
  words[0] = absolute_jump;
  words[1] = unmapped;
  prepare(machine, words);
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  bool refused = false;
  try {
    constexpr unsigned bound = 16;
    for (unsigned i = 0; i < bound; ++i) { static_cast<void>(tick(run, machine, trace)); }
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("Unmapped DSP memory read at P:0x001000") && run.faulted(),
            "Unknown absolute target remains a real contextual memory failure: " + message);
    refused = true;
  }
  constexpr std::uint64_t completed_prefix = 5;
  require(refused && run.completed_cycles() == completed_prefix &&
              run.elapsed_ticks() == completed_prefix * period,
          "Only the genuine five-period prefix completes before rejected target F2");
  state(machine, before);
}

void cold_owners(std::ostream &trace) {
  std::array<std::exception_ptr, 2> errors{};
  std::array<std::string, 2> rows{};
  const auto worker = [&](const unsigned index) {
    try {
      std::ostringstream local;
      one_case(0, Target::self, local);
      actual_words(true, local);
      rows[index] = local.str();
    } catch (...) { errors[index] = std::current_exception(); }
  };
  std::jthread first(worker, 0U);
  std::jthread second(worker, 1U);
  first.join();
  second.join();
  for (const auto &error : errors) {
    if (error) { std::rethrow_exception(error); }
  }
  trace << rows[0] << rows[1];
}

void extension_target(std::ostream &trace) {
  constexpr unsigned branch = 0xb8;
  constexpr unsigned operand_address = branch + 1; // Also the independent ANDI #0,CCR encoding.
  DspMachine machine;
  std::vector<std::uint32_t> rom(rom_words, 0);
  rom[0] = absolute_jump;
  rom[1] = branch;
  ram_nops(machine, branch);
  machine.bus().write({.address = branch, .value = absolute_jump});
  machine.bus().write({.address = operand_address, .value = operand_address});
  prepare(machine, rom);
  auto expected = machine.cpu().debug_state();
  DspFetchRun run(contract(), machine);
  constexpr std::array periods{7U, 11U, 15U, 18U};
  constexpr std::array next_pc{branch, operand_address, operand_address + 1, operand_address + 2};
  constexpr std::array costs{4U, 4U, 3U, 1U};
  unsigned retired = 0;
  unsigned captures = 0;
  while (run.completed_cycles() < periods.back()) {
    const auto cycle = tick(run, machine, trace);
    if (cycle.bus.fetch &&
        cycle.bus.fetch->words.at(cycle.bus.fetch->used - 1).address == operand_address) {
      ++captures;
    }
    if (!stages(cycle).retirement_published) { continue; }
    require(cycle.completed_cycles == periods.at(retired),
            "Own-extension target follows literal JMP/ANDI/NOP retirement periods");
    expected.cpu.pc = next_pc.at(retired);
    expected.native_cycles += costs.at(retired);
    ++expected.native_instructions;
    if (retired == 2) {
      constexpr unsigned ccr_mask = 0xff;
      expected.cpu.sr &= ~ccr_mask;
    }
    state(machine, expected);
    ++retired;
  }
  require(retired == periods.size() && captures == 2,
          "A jump into its own extension executes a fresh opcode F2 and preserves literal state");
}

void active_loop_refusal(std::ostream &trace) {
  DspMachine machine;
  DspProgram code;
  code.emit("do #2," + number(boot + 8));
  code.word(absolute_jump);
  code.word(boot + 6);
  constexpr unsigned padding = 32;
  for (unsigned i = 0; i < padding; ++i) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  bool refused = false;
  try {
    constexpr unsigned bound = 32;
    for (unsigned i = 0; i < bound; ++i) { static_cast<void>(tick(run, machine, trace)); }
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("Unqualified absolute JMP/active DO read P:") &&
                message.contains("pc=") && message.contains("omr=") && run.faulted(),
            "A genuine active native DO must expose the unqualified control overlap: " + message);
    refused = true;
  }
  require(refused && machine.cpu().state().lc == 2,
          "The real native DO count survives the refused absolute-branch forecast");
}

void run(std::ostream &trace) {
  cold_owners(trace);
  for (const unsigned prefix : {0U, 1U, 7U, 16U}) {
    for (const auto target : {Target::rom, Target::ram, Target::sequential, Target::self}) {
      one_case(prefix, target, trace);
    }
  }
  snapshots(trace);
  actual_words(false, trace);
  actual_words(true, trace);
  for (unsigned variant = 0; variant < 7; ++variant) { refused_pair(variant); }
  missing_extension();
  unknown_target(trace);
  extension_target(trace);
  active_loop_refusal(trace);
}
}

int main(const int argc, const char *const *const argv) {
  try {
    blofeld::test::require(argc == 2 || argc == 3,
                           "Expected own diagnostic trace and optional alias scope");
    std::ofstream trace{std::filesystem::path(argv[1])};
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    if (argc == 3) {
      const std::string_view scope(argv[2]);
      if (scope == "alias") {
        extension_target(trace);
      } else if (scope == "loop-refusal") {
        active_loop_refusal(trace);
      } else {
        throw blofeld::Fault("Unknown own test scope");
      }
    } else {
      run(trace);
    }
    std::cout << "Native absolute JMP/F2 ownership and independent state checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
