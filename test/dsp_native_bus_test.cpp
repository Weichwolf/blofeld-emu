// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr unsigned nops = 32, loop_instructions = nops + 1, loop_cycles = nops + 3;
constexpr auto jump = isa::opcode<isa::Opcode::jmp_absolute_short>().front();
constexpr auto store_x = isa::opcode<isa::Opcode::move_x_effective>().front() | 0x3000;
constexpr auto load_x = store_x | 0x8000;
constexpr unsigned missing_program = 0x300, datum_address = 0x80, datum = 0x01ffff;
constexpr unsigned bus_control = 0xfffffb, port_f_control = 0xfffff7;
unsigned checks = 0;

void require(const bool condition, const std::string_view reason) {
  ++checks;
  if (!condition) { throw blofeld::Fault(std::string(reason)); }
}

void program(DspBus &bus, const std::span<const Word> words) {
  for (unsigned index = 0; index < words.size(); ++index) {
    bus.write({.address = index, .value = words[index]});
  }
}

std::string rejected(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Unsupported actual board access was accepted");
}

void native_loop_and_refusal() {
  DspBus bus;
  std::array<Word, loop_instructions> words{};
  words.fill(isa::nop.front());
  words.back() = jump;
  program(bus, words);
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  const auto first = cpu.run(loop_instructions);
  require(first.retired == loop_instructions && first.fault.kind == FaultKind::none &&
              cpu.registers().pc == 0 &&
              first.progress ==
                  Progress{.instructions = loop_instructions, .estimated_cycles = loop_cycles} &&
              !io.failure(),
          "The real board adapter executes native NOP/JMP with exact retired state and base costs");
  const auto revision = bus.program_revision();
  bus.write({.space = DspSpace::x, .address = datum_address, .value = datum});
  const auto second = cpu.run(loop_instructions);
  require(second.retired == loop_instructions && second.fault.kind == FaultKind::none &&
              second.progress == Progress{.instructions = std::uint64_t{2} * loop_instructions,
                                          .estimated_cycles = std::uint64_t{2} * loop_cycles} &&
              cpu.registers().pc == 0 && bus.program_revision() == revision && !io.failure(),
          "Ordinary data writes preserve execution and do not mutate the program view");
  bus.write({.value = jump | missing_program});
  const auto changed = cpu.run(1);
  require(changed.retired == 1 && changed.fault.kind == FaultKind::none &&
              cpu.registers().pc == missing_program && bus.program_revision() > revision,
          "An explicit program transition updates the actual program view");
  const auto refused = cpu.run(1);
  const auto &failure = io.failure();
  require(refused.retired == 0 && refused.fault.kind == FaultKind::memory_read &&
              refused.fault.space == Space::program && refused.fault.address == missing_program &&
              refused.fault.pc == missing_program && failure &&
              failure->operation == DspIo::Operation::read && failure->address == missing_program &&
              failure->pc == missing_program && failure->cause != nullptr,
          "Missing code retains its refused original read and precise execution context");
}

void real_peripheral_transfer() {
  DspBus bus;
  constexpr std::array words{load_x, datum_address, store_x, bus_control, load_x, bus_control};
  program(bus, words);
  bus.write({.space = DspSpace::x, .address = datum_address, .value = datum});
  const auto revision = bus.program_revision();
  require(bus.read({.space = DspSpace::x, .address = bus_control}) != datum &&
              rejected([&] {
                static_cast<void>(bus.read({.space = DspSpace::y, .address = port_f_control}));
              }).contains("requires BCR"),
          "Before the actual BCR write the documented Port-F gate rejects register access");
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  auto expected = cpu.registers();
  expected.pc = 4;
  expected.x = datum;
  const auto result = cpu.run(2);
  require(result.retired == 2 && result.fault.kind == FaultKind::none &&
              result.progress == Progress{.instructions = 2, .estimated_cycles = 4} &&
              cpu.registers() == expected &&
              bus.read({.space = DspSpace::y, .address = port_f_control}) == 0 &&
              bus.program_revision() == revision && !io.failure(),
          "Native RAM load and actual peripheral store enable the real BCR gate exactly once");
  const auto refused = cpu.run(1);
  const auto &failure = io.failure();
  std::string reason;
  if (failure && failure->cause != nullptr) {
    try {
      std::rethrow_exception(failure->cause);
    } catch (const blofeld::Fault &error) { reason = error.what(); }
  }
  const dsp56300::Fault expected_fault{
      .kind = FaultKind::io_read, .space = Space::x, .address = bus_control, .pc = 4};
  require(refused.retired == 0 && refused.fault == expected_fault && failure &&
              failure->operation == DspIo::Operation::read && failure->address == bus_control &&
              failure->pc == 4 && reason.contains("BIU bus-master state"),
          "Unobserved BCR status fails with the original cause instead of an invented response");
}
}

int main() {
  try {
    native_loop_and_refusal();
    real_peripheral_transfer();
    std::cout << checks << " native board bus checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
