// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned nops = 32, loop_instructions = nops + 1, loop_cycles = nops + 3;
constexpr unsigned short_jump = 0x0c0000, missing_program = 0x300;
constexpr unsigned load_x = 0x44f000, store_x = 0x447000;
constexpr unsigned datum_address = 0x80, datum = 0x01ffff, bus_control = 0xfffffb;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned port_f_control = 0xfffff7;

void require(const bool condition, const std::string &reason) {
  if (!condition) { throw Fault(reason); }
}

void program(DspBus &bus, const std::span<const std::uint32_t> words) {
  for (unsigned index = 0; index < words.size(); ++index) {
    bus.write({.address = index, .value = words[index]});
  }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) { return error.what(); }
  throw Fault("Unsupported actual board access was accepted");
}

void native_cache_and_refusal() {
  DspBus bus;
  std::array<std::uint32_t, loop_instructions> words{};
  words.back() = short_jump;
  program(bus, words);
  OwnDspIo io(bus);
  dsp56300::Core cpu(io);
  const auto bounded =
      cpu.run(dsp56300::RunBudget{.instructions = loop_instructions, .cycles = loop_cycles});
  const auto &first = bounded.execution;
  const auto native = cpu.jit_statistics();
  require(cpu.backend() == dsp56300::Backend::jit && first.dispatches == loop_instructions &&
              first.fault.kind == dsp56300::FaultKind::none && cpu.registers().pc == 0 &&
              first.progress.instructions == loop_instructions &&
              first.progress.estimated_cycles == loop_cycles && native.compiled_blocks > 0 &&
              native.native_alu_instructions == loop_instructions &&
              native.helper_instructions == 0 && bounded.cycles == loop_cycles &&
              bounded.stop == dsp56300::RunStop::instruction_budget && bounded.next &&
              bounded.next->address == 0 && bounded.next->opcode == 0 && !io.failure(),
          "The actual board adapter executes bounded native NOP/JMP code with no ALU helper");
  const auto revision = bus.program_revision();
  bus.write({.space = DspSpace::x, .address = datum_address, .value = datum});
  const auto second = cpu.run(loop_instructions);
  require(second.dispatches == loop_instructions &&
              second.fault.kind == dsp56300::FaultKind::none &&
              second.progress.instructions == std::uint64_t{2} * loop_instructions &&
              second.progress.estimated_cycles == std::uint64_t{2} * loop_cycles &&
              cpu.registers().pc == 0 && bus.program_revision() == revision &&
              cpu.jit_statistics().compiled_blocks == native.compiled_blocks,
          "Independent data writes retain actual cached native code and exact base costs");
  bus.write({.value = short_jump | missing_program});
  const auto changed = cpu.run(1);
  require(changed.dispatches == 1 && changed.fault.kind == dsp56300::FaultKind::none &&
              cpu.registers().pc == missing_program && bus.program_revision() > revision &&
              cpu.jit_statistics().compiled_blocks > native.compiled_blocks,
          "A real program write invalidates stale code before the next native dispatch");
  const auto refused = cpu.run(1);
  const auto &recorded_failure = io.failure();
  require(refused.dispatches == 0 && refused.fault.kind == dsp56300::FaultKind::memory_read &&
              refused.fault.space == dsp56300::Space::program &&
              refused.fault.address == missing_program && refused.fault.pc == missing_program &&
              recorded_failure && recorded_failure->operation == OwnDspIo::Operation::read &&
              recorded_failure->address == missing_program &&
              recorded_failure->pc == missing_program && recorded_failure->cause != nullptr,
          "Missing board code retains the real refused read and precise execution context");
  std::cout << "ACTUAL_NATIVE blocks=" << native.compiled_blocks
            << " alu=" << native.native_alu_instructions
            << " helpers=" << native.helper_instructions
            << " data-cache-reuse=1 code-invalidation=1 unknown-refusal=1\n";
}

void real_peripheral_transfer() {
  DspBus bus;
  constexpr std::array words{load_x, datum_address, store_x, bus_control};
  program(bus, words);
  bus.write({.space = DspSpace::x, .address = datum_address, .value = datum});
  const auto revision = bus.program_revision();
  require(bus.read({.space = DspSpace::x, .address = bus_control}) != datum &&
              rejected([&] {
                static_cast<void>(bus.read({.space = DspSpace::y, .address = port_f_control}));
              }).contains("requires BCR"),
          "Before a real BCR write the documented Port-F BIU gate rejects register access");
  OwnDspIo io(bus);
  dsp56300::Core cpu(io);
  const auto boundary = cpu.run(dsp56300::RunBudget{.instructions = 10, .cycles = 10});
  require(boundary.execution.dispatches == 1 && boundary.cycles == 2 &&
              boundary.stop == dsp56300::RunStop::external_effect && boundary.pc == 2 &&
              boundary.next && boundary.next->opcode == store_x &&
              boundary.next->extension == bus_control && boundary.next_cycles == 2 &&
              cpu.registers().pc == 2 && cpu.registers().x == datum &&
              cpu.jit_statistics().native_move_instructions == 1 && !io.failure(),
          "The native RAM load retires before the whole real peripheral-store owner boundary");
  const auto result = cpu.run(1);
  require(result.dispatches == 1 && result.fault.kind == dsp56300::FaultKind::none &&
              result.progress.instructions == 2 && result.progress.estimated_cycles == 4 &&
              cpu.registers().pc == 4 && (cpu.registers().x & word_mask) == datum &&
              bus.read({.space = DspSpace::y, .address = port_f_control}) == 0 &&
              bus.program_revision() == revision && !io.failure(),
          "The native RAM load and genuine peripheral callback move the actual word into BCR");
  const auto stats = cpu.jit_statistics();
  std::cout << "ACTUAL_IO blocks=" << stats.compiled_blocks
            << " alu=" << stats.native_alu_instructions << " helpers=" << stats.helper_instructions
            << " peripheral-effect=1 cycles=4\n";
  bus.write({.address = 4, .value = load_x});
  bus.write({.address = 5, .value = bus_control});
  const auto refused = cpu.run(1);
  const auto &recorded_failure = io.failure();
  std::string reason;
  if (recorded_failure && recorded_failure->cause != nullptr) {
    try {
      std::rethrow_exception(recorded_failure->cause);
    } catch (const Fault &error) { reason = error.what(); }
  }
  const dsp56300::Fault expected{.kind = dsp56300::FaultKind::io_read,
                                 .space = dsp56300::Space::x,
                                 .address = bus_control,
                                 .pc = 4};
  require(refused.dispatches == 0 && refused.fault == expected && recorded_failure &&
              recorded_failure->operation == OwnDspIo::Operation::read &&
              recorded_failure->address == bus_control && recorded_failure->pc == 4 &&
              reason.contains("BIU bus-master state"),
          "Actual native BCR reads still refuse unobserved BIU status with the true cause");
  std::cout << "ACTUAL_BIU unknown-status-refusal=1 context=1\n";
}

void staged_peripheral_transfer() {
  DspBus bus;
  constexpr std::array words{load_x, datum_address, store_x, bus_control, load_x, bus_control};
  program(bus, words);
  bus.write({.space = DspSpace::x, .address = datum_address, .value = datum});
  OwnDspIo io(bus);
  dsp56300::Core cpu(io);
  const auto advance = [&cpu] {
    auto step = cpu.begin_fetch(dsp56300::InstructionPublication::pcu_updates);
    for (unsigned effect = 0; effect < 32 && step.pending && !step.retired; ++effect) {
      auto observation = *step.pending;
      observation.effect.value ^= word_mask;
      step = cpu.advance_instruction(observation.ticket);
    }
    return step;
  };
  auto wanted = cpu.registers();
  require(advance().retired && !io.failure(),
          "The actual staged board RAM load retires through its owned read and publication");
  wanted.pc = 2;
  wanted.x = datum;
  require(cpu.registers() == wanted &&
              cpu.progress() == dsp56300::Progress{.instructions = 1, .estimated_cycles = 2},
          "Tampered caller observations cannot replace the independently expected held result");
  require(advance().retired && !io.failure(),
          "The stored owned effect writes BCR through the genuine board adapter");
  wanted.pc = 4;
  require(cpu.registers() == wanted &&
              cpu.progress() == dsp56300::Progress{.instructions = 2, .estimated_cycles = 4} &&
              bus.read({.space = DspSpace::y, .address = port_f_control}) == 0,
          "Owned publication preserves full registers, exact base costs and the actual BCR gate");
  const auto refused = advance();
  const dsp56300::Fault expected{.kind = dsp56300::FaultKind::memory_read,
                                 .space = dsp56300::Space::x,
                                 .address = bus_control,
                                 .pc = 4};
  const auto &failure = io.failure();
  require(!refused.retired && refused.fault == expected && !refused.pending && failure &&
              failure->operation == OwnDspIo::Operation::read && failure->pc == 4 &&
              failure->address == bus_control && failure->cause && cpu.registers() == wanted &&
              cpu.progress() == dsp56300::Progress{.instructions = 2, .estimated_cycles = 4},
          "An actual unknown BIU status refuses with context and preserves the retired prefix");
  std::cout << "ACTUAL_STAGED_IO caller-copy-isolation=1 peripheral-write=1 "
               "unknown-refusal=1 retained-prefix=1\n";
}

}

int main() {
  try {
    native_cache_and_refusal();
    real_peripheral_transfer();
    staged_peripheral_transfer();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
