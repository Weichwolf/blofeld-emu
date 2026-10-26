// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_pipeline_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned register_count = 8;
constexpr unsigned guard_words = 4;
constexpr unsigned maximum_clocks = 100;
constexpr unsigned source_seed = 0x123456;
constexpr unsigned destination_seed = 0x654320;
constexpr unsigned memory_address = 0x40;
constexpr unsigned memory_value = 0x7abcde;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned immediate_r0 = 0x60f400;
constexpr unsigned register_move = 0x200000;
constexpr unsigned register_code_r0 = 16;
constexpr unsigned no_producer = 0;

enum class Producer : std::uint8_t { none, literal, memory, postincrement };

struct Case {
  Producer producer = Producer::none;
  unsigned source = 0;
  unsigned destination = 1;
  unsigned gap = 0;
};

bool memory(const Case sample) {
  return sample.producer == Producer::memory || sample.producer == Producer::postincrement;
}

struct Program {
  std::vector<std::string> instructions;
  unsigned pc = pipeline::body;
  unsigned producer_pc = no_producer;
  unsigned copy_pc = 0;
  unsigned source_instructions = 0;

  void append(const std::string &instruction, const unsigned words = 1) {
    instructions.push_back(instruction);
    pc += words;
  }

  void seed(const unsigned index, const unsigned value) {
    append("move #>" + number(value) + ",r" + std::to_string(index), 2);
  }
};

Program prepare(DspMachine &machine, const Case sample) {
  Program code;
  code.seed(sample.source, source_seed + sample.source);
  code.seed(sample.destination, destination_seed + sample.destination);
  if (memory(sample)) {
    code.seed(2, memory_address);
    code.append("move #>$ffffff,m2", 2);
  }
  for (unsigned index = 0; index < guard_words + 1; ++index) { code.append("nop"); }
  if (sample.producer != Producer::none) { code.producer_pc = code.pc; }
  if (sample.producer == Producer::literal) { code.seed(0, memory_value); }
  if (sample.producer == Producer::memory) { code.append("move x:(r2),r0"); }
  if (sample.producer == Producer::postincrement) { code.append("move x:(r2)+,r0"); }
  for (unsigned index = 0; index < sample.gap; ++index) { code.append("nop"); }
  code.copy_pc = code.pc;
  code.source_instructions = static_cast<unsigned>(code.instructions.size());
  code.append("move r" + std::to_string(sample.source) + ",r" + std::to_string(sample.destination));
  std::vector<const char *> pointers;
  pointers.reserve(code.instructions.size());
  for (const auto &text : code.instructions) { pointers.push_back(text.c_str()); }
  pipeline::prepare(machine, pointers);
  if (memory(sample)) {
    machine.bus().write({.space = DspSpace::x, .address = memory_address, .value = memory_value});
  }
  const auto opcode = register_move | ((register_code_r0 + sample.source) << 13) |
                      ((register_code_r0 + sample.destination) << 8);
  require(machine.bus().inspect_program(code.copy_pc) == opcode &&
              machine.bus().inspect_program(pipeline::body) ==
                  (immediate_r0 | (sample.source << 16)),
          "Installed R transfer and seed agree with independently pinned instruction fields");
  return code;
}

DspDebugState expected_inputs(const DspDebugState &before, const Case sample) {
  auto expected = before;
  expected.cpu.r.at(sample.source) = source_seed + sample.source;
  expected.cpu.r.at(sample.destination) = destination_seed + sample.destination;
  if (memory(sample)) {
    expected.cpu.r.at(2) = memory_address;
    expected.cpu.m.at(2) = word_mask;
  }
  if (sample.producer != Producer::none) { expected.cpu.r.front() = memory_value; }
  if (sample.producer == Producer::postincrement) { expected.cpu.r.at(2) = memory_address + 1; }
  return expected;
}

void check_cpu(const DspDebugState &actual, const DspDebugState &expected) {
  require(
      actual.cpu == expected.cpu && actual.sc == expected.sc && actual.ep == expected.ep &&
          actual.sz == expected.sz && actual.stack_high == expected.stack_high &&
          actual.stack_low == expected.stack_low &&
          actual.native_instructions == expected.native_instructions,
      "Native R copy preserves all independently expected CPU/frame fields and instruction count");
}

void check_memory(const DspFetchCycleResult &cycle,
                  const DspMachine &machine,
                  const Case sample,
                  const Program &code) {
  const auto &phase = pipeline::stages(cycle);
  if (!memory(sample)) { return; }
  if (phase.ag2 && phase.ag2->address == code.producer_pc) {
    const auto pointer = memory_address + (sample.producer == Producer::postincrement ? 1U : 0U);
    require(phase.ag2_address && phase.ag2_address->published &&
                phase.ag2_address->update.address == memory_address &&
                phase.ag2_address->update.value == pointer &&
                machine.cpu().state().r.at(2) == pointer,
            "Actual memory producer publishes its independently expected AG2 pointer once");
  }
  if (phase.execute1 && phase.execute1->address == code.producer_pc) {
    require(phase.memory && phase.memory->data.target.space == DspSpace::x &&
                phase.memory->data.target.address == memory_address &&
                phase.memory->data.value == memory_value && phase.execution_captured,
            "The R source producer really reads its own declared X RAM at native E1");
  }
}

void run_case(const Case sample) {
  DspMachine machine;
  const auto code = prepare(machine, sample);
  const auto before = machine.cpu().debug_state();
  const auto inputs = expected_inputs(before, sample);
  auto expected = inputs;
  expected.cpu.r.at(sample.destination) = inputs.cpu.r.at(sample.source);
  expected.cpu.pc = code.copy_pc;
  expected.native_instructions += code.source_instructions;
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  bool actual_e1 = false;
  bool actual_e2 = false;
  for (unsigned clock = 1; clock < maximum_clocks; ++clock) {
    const auto cycle = run.advance_cycle(machine, fetch_clock::input());
    const auto &phase = pipeline::stages(cycle);
    require(!phase.memory_source_stall && phase.agu_dependencies == 0,
            "A stable explicit R data source introduces no undocumented address-generation delay");
    check_memory(cycle, machine, sample, code);
    if (phase.execute1 && phase.execute1->address == code.copy_pc) {
      const DspMoveFields fields{.r = static_cast<std::uint8_t>(1U << sample.destination)};
      require(clock == 6 + code.copy_pc - pipeline::body && phase.execution_captured &&
                  phase.transfer && phase.transfer->fields == fields && phase.e1_operands &&
                  phase.e1_operands->r.at(sample.source) == inputs.cpu.r.at(sample.source) &&
                  phase.candidate_inputs &&
                  phase.candidate_inputs->r.at(sample.source) == inputs.cpu.r.at(sample.source),
              "Actual native R transfer E1 reads the correct producer value and publishes only Rd");
      check_cpu(machine.cpu().debug_state(), expected);
      actual_e1 = true;
    }
    if (phase.execute2 && phase.execute2->address == code.copy_pc) {
      require(actual_e1 && clock == 7 + code.copy_pc - pipeline::body && phase.retirement_published,
              "R copy retires its actual PC on E2 one period after E1");
      ++expected.cpu.pc;
      ++expected.native_instructions;
      check_cpu(machine.cpu().debug_state(), expected);
      actual_e2 = true;
      break;
    }
  }
  require(actual_e1 && actual_e2, "Expected real E1 and E2 R transfer phases");
}

struct DataDestination {
  std::string name;
  DspMoveFields fields;
  DspState expected;
};

DataDestination destination(const DspState &before,
                            const std::string &name,
                            const unsigned index,
                            const unsigned value) {
  DataDestination result{.name = name, .fields = {}, .expected = before};
  constexpr std::uint64_t word = 0xffffff;
  constexpr std::uint64_t extension = 0xff;
  const auto lane = static_cast<std::uint8_t>(1U << index);
  if (name.starts_with('n')) {
    result.fields.n = lane;
    result.expected.n.at(index) = value;
  } else if (name.starts_with('m')) {
    result.fields.m = lane;
    result.expected.m.at(index) = value;
  } else if (name.starts_with('x') || name.starts_with('y')) {
    const auto shift = index * word_bits;
    auto &data = name.starts_with('x') ? result.expected.x : result.expected.y;
    auto &fields = name.starts_with('x') ? result.fields.x : result.fields.y;
    fields = lane;
    data = (data & ~(word << shift)) | (static_cast<std::uint64_t>(value) << shift);
  } else {
    auto &data = name.starts_with('a') ? result.expected.a : result.expected.b;
    auto &fields = name.starts_with('a') ? result.fields.a : result.fields.b;
    fields = lane;
    const auto shift = index * word_bits;
    const auto mask = index == 2 ? extension : word;
    data = (data & ~(mask << shift)) | ((static_cast<std::uint64_t>(value) & mask) << shift);
    if (name.size() == 1) {
      fields = 7;
      data = static_cast<std::uint64_t>(value) << word_bits;
    }
  }
  return result;
}

void data_destination(const unsigned source, const std::string &name, const unsigned index) {
  Program code;
  const auto value = source_seed + source;
  code.seed(source, value);
  for (unsigned guard = 0; guard < guard_words + 1; ++guard) { code.append("nop"); }
  code.copy_pc = code.pc;
  code.source_instructions = static_cast<unsigned>(code.instructions.size());
  code.append("move r" + std::to_string(source) + "," + name);
  std::vector<const char *> pointers;
  pointers.reserve(code.instructions.size());
  for (const auto &text : code.instructions) { pointers.push_back(text.c_str()); }
  DspMachine machine;
  pipeline::prepare(machine, pointers);
  const auto before = machine.cpu().debug_state();
  auto seeded = before;
  seeded.cpu.r.at(source) = value;
  const auto target = destination(seeded.cpu, name, index, value);
  auto expected = seeded;
  expected.cpu = target.expected;
  expected.cpu.pc = code.copy_pc;
  expected.native_instructions += code.source_instructions;
  auto contract = pipeline::contract();
  contract.pipeline = DspFetchPipeline::automatic_memory;
  DspFetchRun run(contract, machine);
  pipeline::clocks(run, machine, 5 + code.copy_pc - pipeline::body);
  const auto e1 = run.advance_cycle(machine, fetch_clock::input());
  const auto &phase = pipeline::stages(e1);
  require(phase.execute1 && phase.execute1->address == code.copy_pc && phase.execution_captured &&
              phase.transfer && phase.transfer->fields == target.fields && phase.candidate_inputs &&
              phase.candidate_inputs->r.at(source) == value,
          "R data E1 uses the literal source and independently expected destination field mask");
  check_cpu(machine.cpu().debug_state(), expected);
  const auto e2 = run.advance_cycle(machine, fetch_clock::input());
  require(pipeline::stages(e2).retirement_published, "R data destination retires the actual E2");
  ++expected.cpu.pc;
  ++expected.native_instructions;
  check_cpu(machine.cpu().debug_state(), expected);
}

void data_destinations() {
  for (unsigned source = 0; source < register_count; ++source) {
    for (const char group : std::array{'n', 'm'}) {
      for (unsigned index = 0; index < register_count; ++index) {
        data_destination(source, std::string(1, group) + std::to_string(index), index);
      }
    }
    for (const char group : std::array{'x', 'y'}) {
      for (unsigned index = 0; index < 2; ++index) {
        data_destination(source, std::string(1, group) + std::to_string(index), index);
      }
    }
    for (const char group : std::array{'a', 'b'}) {
      for (unsigned index = 0; index < 3; ++index) {
        data_destination(source, std::string(1, group) + std::to_string(index), index);
      }
      data_destination(source, std::string(1, group), 0);
    }
  }
}

}

int main(const int argc, const char *const *const argv) {
  try {
    if (argc == 2 && std::string_view(argv[1]) == "fields") {
      data_destinations();
      std::cout << "Native R sources publish all independently expected X/Y/A/B/N/M fields\n";
      return 0;
    }
    require(argc == 1, "Unknown R transfer test argument");
    for (unsigned source = 0; source < register_count; ++source) {
      for (unsigned destination = 0; destination < register_count; ++destination) {
        run_case({.source = source, .destination = destination});
      }
    }
    for (const auto producer :
         std::array{Producer::literal, Producer::memory, Producer::postincrement}) {
      for (unsigned gap = 0; gap < guard_words; ++gap) {
        run_case({.producer = producer,
                  .source = producer == Producer::postincrement ? 2U : 0U,
                  .gap = gap});
      }
    }
    data_destinations();
    std::cout << "Native R data transfer, real memory producer and AG2 forwarding cases passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
