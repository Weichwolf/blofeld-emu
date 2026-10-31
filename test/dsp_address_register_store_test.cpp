// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_address_register_store.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100, data = 0x400, edge = 0x4ff, second_data = 0x500;
constexpr unsigned ordinary_omr = 0x300, word_mask = 0xffffff, byte_mask = 0xff;
constexpr unsigned modulo_256 = 0x80ff, ring_size = 256, canary = 0x654321;
constexpr unsigned x0_value = 0x123456, jump = 0x0af080, illegal_word = 7;
constexpr unsigned r_seed = 0x55aa00, final_clock = 7;
constexpr std::uint64_t period_ticks = 45;
constexpr std::array modes{0U, 1U, 2U, 3U, 4U, 5U, 7U};
constexpr std::array values{0U, 1U, 0x7fffffU, 0x800000U, 0xabcdefU, word_mask};
constexpr std::array scales{0U, 0x400U, 0x800U, 0xc00U};
constexpr std::array flags{0x55U, 0xaaU};

struct Case {
  unsigned source = 0, reg = 2, mode = 3, address = data, value = second_data;
  unsigned scale = 0, flags = 0;
  bool y = false, modulo = false, younger = false;
};

// FM Appendix A/Table A-1: indexed/predecrement adds one owned E1 hold.
unsigned latency(const Case sample) {
  return sample.mode == 5 || sample.mode == 7 ? 1U : 0U;
}

unsigned end_clock(const Case sample) {
  return final_clock + latency(sample) + (sample.younger ? 1U : 0U);
}

unsigned source_value(const Case sample) {
  return sample.source == sample.reg ? sample.address : sample.value;
}

unsigned add(const unsigned address, const int delta, const bool modulo) {
  if (!modulo) { return static_cast<unsigned>(static_cast<int>(address) + delta); }
  const auto low = static_cast<unsigned>(
      (static_cast<int>(address & byte_mask) + delta + static_cast<int>(ring_size)) %
      static_cast<int>(ring_size));
  return (address & ~byte_mask) | low;
}

int delta(const Case sample) {
  constexpr std::array offsets{-3, 3, -1, 1, 0, 3, 0, -1};
  return offsets.at(sample.mode);
}

unsigned pointer(const Case sample) {
  return sample.mode == 5 ? sample.address : add(sample.address, delta(sample), sample.modulo);
}

unsigned address(const Case sample) {
  return sample.mode == 5 || sample.mode == 7 ? add(sample.address, delta(sample), sample.modulo)
                                              : sample.address;
}

unsigned younger_address(const Case sample) {
  return sample.source == sample.reg ? pointer(sample) : sample.value;
}

unsigned younger_pointer(const Case sample) {
  return add(younger_address(sample), 1, sample.modulo && sample.source == sample.reg);
}

unsigned opcode(const Case sample) {
  // FM13-118/122: independent source16..23,space,EA,W,MMM and RRR fields.
  constexpr unsigned raw_r_base = 0x604000;
  constexpr unsigned source_shift = 16;
  constexpr unsigned y_bit = 0x80000;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned reg_shift = 8;
  return raw_r_base | (sample.source << source_shift) | (sample.y ? y_bit : 0U) |
         (sample.mode << mode_shift) | (sample.reg << reg_shift);
}

unsigned younger_opcode(const Case sample) {
  constexpr unsigned x0_postincrement = 0x445800;
  constexpr unsigned y_bit = 0x80000;
  return x0_postincrement | (sample.source << 8U) | (sample.y ? 0U : y_bit);
}

std::string operand(const Case sample) {
  const auto r = "r" + std::to_string(sample.reg);
  const auto n = "n" + std::to_string(sample.reg);
  auto base = '(' + r + ')';
  switch (sample.mode) {
    case 0: return base + '-' + n;
    case 1: return base + '+' + n;
    case 2: return base + '-';
    case 3: return base + '+';
    case 4: return base;
    case 5: return '(' + r + '+' + n + ')';
    case 7: return '-' + base;
    default: throw Fault("Undefined own R-source store address mode");
  }
}

void encoding(const Case sample) {
  const auto text =
      "move r" + std::to_string(sample.source) + (sample.y ? ",y:" : ",x:") + operand(sample);
  require(test::checkpoint::assemble(text.c_str()) == std::vector<std::uint32_t>{opcode(sample)},
          "Independent raw R-source and address fields match the native assembler");
  if (sample.younger) {
    const auto text2 = "move x0," + std::string(sample.y ? "x:" : "y:") + "(r" +
                       std::to_string(sample.source) + ")+";
    require(test::checkpoint::assemble(text2.c_str()) ==
                std::vector<std::uint32_t>{younger_opcode(sample)},
            "Independent younger X0 store encoding");
  }
}

DspFetchRunInputs contract() {
  auto result = test::fetch_clock::contract();
  result.execution = DspReferenceExecution::staged_register_boundaries;
  result.pipeline = DspFetchPipeline::automatic_memory;
  return result;
}

std::array<unsigned, 6> cells(const Case sample) {
  const unsigned other = sample.younger ? younger_address(sample) : second_data;
  return {address(sample) - 1, address(sample), address(sample) + 1, other - 1, other, other + 1};
}

DspDebugState prepare(DspMachine &machine, const Case sample) {
  test::DspProgram code;
  code.immediate("omr", ordinary_omr);
  code.immediate("sz", 0);
  for (unsigned i = 0; i < 8; ++i) {
    unsigned value = r_seed + i;
    if (i == sample.source) { value = sample.value; }
    if (i == sample.reg) { value = sample.address; }
    code.immediate("r" + std::to_string(i), value);
    code.immediate("n" + std::to_string(i), 3);
    code.immediate("m" + std::to_string(i),
                   sample.modulo && i == sample.reg ? modulo_256 : word_mask);
  }
  code.immediate("x0", x0_value);
  code.immediate("y0", second_data);
  code.immediate("sr", test::masked_status | sample.scale | sample.flags);
  for (unsigned i = 0; i < 5; ++i) { code.word(0); }
  code.word(jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned pc = entry; pc < entry + 16; ++pc) { machine.bus().write({.address = pc}); }
  machine.bus().write({.address = entry, .value = opcode(sample)});
  if (sample.younger) {
    machine.bus().write({.address = entry + 1, .value = younger_opcode(sample)});
  }
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells(sample)) {
      machine.bus().write({.space = space, .address = cell, .value = canary});
    }
  }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == entry && before.cpu.omr == ordinary_omr &&
              before.cpu.r.at(sample.source) == source_value(sample) &&
              before.cpu.r.at(sample.reg) == sample.address &&
              before.cpu.sr == (test::masked_status | sample.scale | sample.flags),
          "Real setup supplies literal R-source,AGU and CCR inputs");
  static_cast<void>(machine.begin_reference_instruction(contract().execution));
  return before;
}

DspDebugState expected(const DspDebugState &before, const Case sample, const unsigned clock) {
  auto result = before;
  if (clock >= 5) { result.cpu.r.at(sample.reg) = pointer(sample); }
  if (sample.younger && clock >= 6 + latency(sample)) {
    result.cpu.r.at(sample.source) = younger_pointer(sample);
  }
  if (clock >= 7 + latency(sample)) {
    ++result.native_instructions;
    result.native_cycles += 1 + latency(sample);
    ++result.cpu.pc;
  }
  if (sample.younger && clock >= 8 + latency(sample)) {
    ++result.native_instructions;
    ++result.native_cycles;
    ++result.cpu.pc;
  }
  return result;
}

void memory(DspMachine &machine, const Case sample, const unsigned clock) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells(sample)) {
      unsigned value = canary;
      if (clock >= 6 + latency(sample) && cell == address(sample) &&
          (space == DspSpace::y) == sample.y) {
        value = source_value(sample);
      }
      if (sample.younger && clock >= 7 + latency(sample) && cell == younger_address(sample) &&
          (space == DspSpace::y) != sample.y) {
        value = x0_value;
      }
      require(machine.bus().read({.space = space, .address = cell}) == value,
              "Only actual old R-source and younger X0 cells change; neighbors retain canaries");
    }
  }
}

void step(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("R-source store lost its actual pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  if (clock == 3) {
    require(phase.decode_accepted &&
                phase.decode == DspProgramWord{.address = entry, .value = opcode(sample)},
            "The actual F2 R-source occurrence owns Decode");
    machine.bus().write({.address = entry, .value = illegal_word});
  }
  if (sample.younger && clock == 4) {
    machine.bus().write({.address = entry + 1, .value = illegal_word});
  }
  const bool first = clock == 6 + latency(sample);
  const bool second = sample.younger && clock == 7 + latency(sample);
  const unsigned fetches = clock == 1 || (latency(sample) != 0 && clock == 7) ? 0U : 1U;
  require(phase.memory.has_value() == (first || second) && !phase.memory_source_stall &&
              phase.agu_dependencies == 0 &&
              cycle.bus.core_access_count == fetches + (first || second ? 1U : 0U),
          "Exactly one real E1 write and genuine F2 per period; no invented holds");
  if (first) {
    if (!phase.memory) { throw Fault("R-source E1 lost its actual memory effect"); }
    const auto &effect = *phase.memory;
    if (effect.data.value != source_value(sample)) {
      std::cerr << "R_SOURCE clock=" << clock << " source=" << sample.source
                << " reg=" << sample.reg << " mode=" << sample.mode << " younger=" << sample.younger
                << " modulo=" << sample.modulo
                << " stored=" << (effect.data.value ? hex(*effect.data.value, 6) : "absent")
                << " expected=" << hex(source_value(sample), 6) << '\n';
    }
    require(effect.address.kind == DspAddressStageKind::register_agu &&
                effect.address.update.before == sample.address &&
                effect.address.update.value == pointer(sample) &&
                effect.data.kind == DspNativeAccessKind::write &&
                effect.data.target.address == address(sample) &&
                effect.data.target.space == (sample.y ? DspSpace::y : DspSpace::x) &&
                effect.data.value == source_value(sample),
            "Native E1 stores the old raw R source beside genuine AG1/AG2 updates");
  }
  require(machine.cpu().debug_state() == expected(before, sample, clock),
          "Complete native CPU,CCR,stack and counters retain actual AG2/E1/E2 order");
  require(run.elapsed_ticks() == clock * period_ticks, "The complete period ledger remains exact");
  memory(machine, sample, clock);
}

unsigned windows = 0, restores = 0, clones = 0;

void finish(DspFetchRun &run, DspMachine &machine, const DspDebugState &before, const Case sample) {
  const auto end = end_clock(sample);
  while (run.completed_cycles() < end) { step(run, machine, before, sample); }
}

void one(const Case sample) {
  encoding(sample);
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(contract(), machine);
  std::vector<DspFetchCheckpoint> cuts;
  const bool snapshots = sample.source == 0 && (sample.reg == 0 || sample.reg == 2) &&
                         sample.mode == 3 && sample.scale == 0 && sample.flags == flags.at(0) &&
                         !sample.y && sample.younger && !sample.modulo;
  while (run.completed_cycles() < end_clock(sample)) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    step(run, machine, before, sample);
  }
  for (const auto &cut : cuts) {
    run.restore(cut, machine);
    finish(run, machine, before, sample);
    ++restores;
    DspMachine fresh;
    DspFetchRun clone(cut, fresh);
    finish(clone, fresh, before, sample);
    ++clones;
  }
  ++windows;
}

void base_variants(Case sample) {
  for (const auto scale : scales) {
    for (const auto pattern : flags) {
      sample.scale = scale;
      sample.flags = pattern;
      one(sample);
    }
  }
}

void base() {
  for (unsigned source = 0; source < 8; ++source) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto mode : modes) {
        for (const bool y : {false, true}) {
          base_variants({.source = source,
                         .reg = reg,
                         .mode = mode,
                         .value = values.at((source + reg + mode) % values.size()),
                         .y = y});
        }
      }
    }
  }
  for (unsigned source = 0; source < 8; ++source) {
    for (const auto mode : modes) {
      for (const bool y : {false, true}) {
        for (const auto value : values) {
          one({.source = source, .reg = (source + 1) & 7U, .mode = mode, .value = value, .y = y});
        }
      }
    }
  }
}

void younger() {
  for (unsigned source = 0; source < 8; ++source) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto mode : modes) {
        for (const bool y : {false, true}) {
          for (const auto pattern : flags) {
            one({.source = source,
                 .reg = reg,
                 .mode = mode,
                 .flags = pattern,
                 .y = y,
                 .younger = true});
          }
        }
      }
    }
  }
}

void wraps() {
  for (unsigned source = 0; source < 8; ++source) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto mode : modes) {
        for (const bool y : {false, true}) {
          for (const auto first : {data, edge}) {
            for (const bool next : {false, true}) {
              one({.source = source,
                   .reg = reg,
                   .mode = mode,
                   .address = first,
                   .flags = flags.at(1),
                   .y = y,
                   .modulo = true,
                   .younger = next});
            }
          }
        }
      }
    }
  }
}

struct Producer {
  Case sample;
  unsigned gap = 0;
};

unsigned producer_wait(const Producer input) {
  return input.sample.source == input.sample.reg && input.gap < 3 ? 3 - input.gap : 0U;
}

unsigned producer_end(const Producer input) {
  return 8 + input.gap + producer_wait(input);
}

void producer_memory(DspMachine &machine,
                     const Case sample,
                     const unsigned target,
                     const bool committed) {
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto cell : cells(sample)) {
      const bool written = committed && cell == target && (space == DspSpace::y) == sample.y;
      require(machine.bus().read({.space = space, .address = cell}) ==
                  (written ? second_data : canary),
              "Producer/consumer windows retain every neighbor and opposite-space cell");
    }
  }
}

void producer_step(DspFetchRun &run,
                   DspMachine &machine,
                   const DspDebugState &before,
                   const Producer input) {
  const auto cycle = run.advance_cycle(machine, test::fetch_clock::input());
  if (!cycle.pipeline) { throw Fault("R-source producer lost its actual pipeline"); }
  const auto &phase = *cycle.pipeline;
  const auto clock = static_cast<unsigned>(cycle.completed_cycles);
  const auto sample = input.sample;
  const unsigned body = entry + 1 + input.gap;
  const unsigned decoded = 4 + input.gap + producer_wait(input);
  const bool held = clock >= 4 + input.gap && clock < decoded;
  const bool stored = clock == decoded + 3;
  if (clock == 3 || clock == decoded) {
    const unsigned pc = clock == 3 ? entry : body;
    require(
        phase.decode_accepted && phase.decode && phase.decode->address == pc,
        "Captured producer and consumer Decode periods match independent gap/interlock positions");
    machine.bus().write({.address = pc, .value = illegal_word});
  }
  auto wanted = before;
  if (clock >= 6) { wanted.cpu.r.at(sample.source) = second_data; }
  const bool alias = sample.source == sample.reg;
  const unsigned target = alias ? second_data : data;
  if (clock >= decoded + 2) { wanted.cpu.r.at(sample.reg) = target + 1; }
  const unsigned leading = clock >= 7 ? std::min(clock - 6, 1 + input.gap) : 0U;
  wanted.cpu.pc += leading;
  wanted.native_instructions += leading;
  wanted.native_cycles += leading;
  if (clock >= producer_end(input)) {
    ++wanted.cpu.pc;
    ++wanted.native_instructions;
    ++wanted.native_cycles;
  }
  require(machine.cpu().debug_state() == wanted && run.elapsed_ticks() == clock * period_ticks,
          "MOVER E1 publication and genuine address interlocks precede raw source sampling");
  require(phase.memory.has_value() == stored && !phase.memory_source_stall &&
              (phase.agu_dependencies != 0) == held && phase.transfer.has_value() == (clock == 6) &&
              cycle.bus.core_access_count == (clock == 1 || held ? 0U : 1U) + (stored ? 1U : 0U),
          "Only real address-set dependencies hold Decode/F1/F2; ordinary R data sources need no "
          "false hold");
  if (stored) {
    if (!phase.memory) { throw Fault("R producer consumer lost its actual E1 store"); }
    require(phase.memory->data.value == second_data &&
                phase.memory->data.target.address == target &&
                phase.memory->data.target.space == (sample.y ? DspSpace::y : DspSpace::x),
            "E1 observes the preceding real Y0-to-R move,not an early AG1 source latch");
  }
  producer_memory(machine, sample, target, clock >= decoded + 3);
}

void producer_one(const Producer input) {
  const auto sample = input.sample;
  encoding(sample);
  constexpr unsigned y0_to_r0 = 0x20d000;
  const auto writer = y0_to_r0 | (sample.source << 8U);
  const auto text = "move y0,r" + std::to_string(sample.source);
  require(test::checkpoint::assemble(text.c_str()) == std::vector<std::uint32_t>{writer},
          "Independent Y0-to-R producer field encoding");
  DspMachine machine;
  const auto before = prepare(machine, sample);
  for (unsigned pc = entry; pc < entry + 16; ++pc) { machine.bus().write({.address = pc}); }
  machine.bus().write({.address = entry, .value = writer});
  machine.bus().write({.address = entry + 1 + input.gap, .value = opcode(sample)});
  DspFetchRun run(contract(), machine);
  std::vector<DspFetchCheckpoint> cuts;
  const bool snapshots =
      sample.source == 0 && input.gap == 0 && !sample.y && sample.flags == flags.at(0);
  const auto finish = [&](DspFetchRun &owner, DspMachine &target) {
    while (owner.completed_cycles() < producer_end(input)) {
      producer_step(owner, target, before, input);
    }
  };
  while (run.completed_cycles() < producer_end(input)) {
    if (snapshots) { cuts.push_back(run.checkpoint(machine)); }
    producer_step(run, machine, before, input);
  }
  for (const auto &cut : cuts) {
    run.restore(cut, machine);
    finish(run, machine);
    ++restores;
    DspMachine fresh;
    DspFetchRun clone(cut, fresh);
    finish(clone, fresh);
    ++clones;
  }
  ++windows;
}

void producers() {
  for (unsigned source = 0; source < 8; ++source) {
    for (const bool alias : {false, true}) {
      for (const bool y : {false, true}) {
        for (unsigned gap = 0; gap < 4; ++gap) {
          for (const auto pattern : flags) {
            producer_one({.sample = {.source = source,
                                     .reg = alias ? source : (source + 1) & 7U,
                                     .value = 0x800000,
                                     .flags = pattern,
                                     .y = y},
                          .gap = gap});
          }
        }
      }
    }
  }
  std::cout << "R_SOURCE producers=256 passed\n";
}

void guards() {
  for (unsigned source = 0; source < 8; ++source) {
    const Case sample{.source = source};
    const auto word = opcode(sample);
    require(address_register_scalar_store(word) && !data_offset_scalar_store(word) &&
                !fetch_repeat_body(word),
            "R-source qualification does not widen the separate REP source families");
    constexpr unsigned transfer_y1_a = 0x71;
    require(!decode_memory_stage(word | transfer_y1_a),
            "Parallel ALU R-source forms remain separate");
  }
  std::cout << "R_SOURCE guards=16 passed\n";
}
}

int main() {
  try {
    std::cout << "R_SOURCE begin\n" << std::flush;
    base();
    std::cout << "R_SOURCE base=" << windows << " passed\n" << std::flush;
    younger();
    wraps();
    producers();
    guards();
    std::cout << "R_SOURCE windows=" << windows << " restores=" << restores << " clones=" << clones
              << " passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "R_SOURCE windows=" << windows << ": " << error.what() << '\n';
    return 1;
  }
}
