// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "isa.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr Word low_x_end = 0x3000, low_y_end = 0x2000, rom_base = 0x4000, high_base = 0xc000;
constexpr Word default_x_end = 0x12000, default_y_end = 0x16000;
constexpr Word source = 0x80, destination = 0xc080, datum = 0x9654a1, changed = 0x2579b3;
constexpr Word switched_tail = 0x10000;
constexpr Word absolute_address = 0x3000, read_direction = 0x8000;
constexpr auto write_x = isa::opcode<isa::Opcode::move_x_effective>().front() | absolute_address;
constexpr auto write_y = isa::opcode<isa::Opcode::move_y_effective>().front() | absolute_address;
constexpr auto read_x = write_x | read_direction, read_y = write_y | read_direction;
constexpr Word memory_switch = 0x80, memory_switch_shift = 21;
constexpr std::uint64_t initial_x = 0x112233aabbcc;
constexpr unsigned word_bits = 24;
constexpr std::array banks{DspSpace::x, DspSpace::y};
constexpr std::array<Word, 4> switched_y_ends{0xe000, 0x10000, 0x14000, 0x16000};
std::size_t checks = 0;

void require(const bool value, const char *message) {
  ++checks;
  if (!value) { throw blofeld::Fault(message); }
}

std::string refused(const auto &action) {
  try {
    action();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Unsupported actual data access was accepted");
}

Space core_space(const DspSpace space) {
  return space == DspSpace::x ? Space::x : Space::y;
}

Word read_opcode(const DspSpace space) {
  return space == DspSpace::x ? read_x : read_y;
}

Word write_opcode(const DspSpace space) {
  return space == DspSpace::x ? write_x : write_y;
}

void code(DspBus &bus, const std::span<const Word> words) {
  for (unsigned index = 0; index < words.size(); ++index) {
    bus.write({.address = index, .value = words[index]});
  }
}

Registers initial(const Word pc = 0) {
  Registers result;
  result.pc = pc;
  result.m.fill(word_mask);
  result.x = initial_x;
  return result;
}

Registers loaded(const Word value) {
  auto result = initial(2);
  result.x = (initial_x & (std::uint64_t{word_mask} << word_bits)) | value;
  return result;
}

void shape(DspBus &bus, const Word x_end, const Word y_end) {
  for (const auto space : banks) {
    const auto windows = bus.data_windows(space);
    const auto low = space == DspSpace::x ? low_x_end : low_y_end;
    const auto end = space == DspSpace::x ? x_end : y_end;
    require(windows[0].address == 0 && windows[0].words.size() == low && windows[0].writable &&
                windows[1].address == rom_base && windows[1].words.size() == high_base - rom_base &&
                !windows[1].writable && windows[2].address == high_base &&
                windows[2].words.size() == end - high_base && windows[2].writable,
            "Independent low/ROM/high intervals and permissions match the current stable map");
    for (const auto &window : windows) {
      require(window.words.size() == window.known.size() && !window.words.empty(),
              "Every real storage word has its matching actual knowledge byte");
    }
    const auto revision = bus.program_revision();
    for (const auto slot : {0U, 2U}) {
      const auto &window = windows[slot];
      const auto last = static_cast<Word>(window.words.size() - 1);
      bus.write({.space = space, .address = window.address, .value = datum});
      bus.write({.space = space, .address = window.address + last, .value = changed});
      require(window.words.front() == datum && window.known.front() == 1 &&
                  window.words.back() == changed && window.known.back() == 1 &&
                  bus.program_revision() == revision,
              "Both ends of each data RAM span expose actual bus storage without code mutation");
    }
    constexpr std::array rom{datum};
    bus.install_rom({.space = space, .start = rom_base, .words = rom});
    require(windows[1].words.front() == datum && windows[1].known.front() == 1 &&
                windows[1].known[1] == 0 && bus.program_revision() == revision,
            "A partial genuine ROM install changes its own bytes, preserving unknown neighbors");
  }
}

void layouts() {
  for (unsigned mode = 0; mode < switched_y_ends.size(); ++mode) {
    DspBus bus;
    shape(bus, default_x_end, default_y_end);
    const auto revision = bus.program_revision();
    bus.observe_omr(memory_switch | (mode << memory_switch_shift));
    require(bus.program_revision() > revision, "Switch admission changes the actual view revision");
    shape(bus, switched_tail, switched_y_ends[mode]);
    bus.advance_instruction_cycles(2);
    shape(bus, switched_tail, switched_y_ends[mode]);
    bus.advance_instruction_cycles(1);
    shape(bus, switched_tail, switched_y_ends[mode]);
    bus.observe_omr(0);
    shape(bus, switched_tail, switched_y_ends[mode]);
    bus.advance_instruction_cycles(3);
    shape(bus, default_x_end, default_y_end);
    bus.reset();
    shape(bus, default_x_end, default_y_end);
  }
  DspBus bus;
  require(refused([&] { static_cast<void>(bus.data_windows(DspSpace::p)); }).contains("X or Y"),
          "A program-space grant request is explicitly rejected");
}

void native_known_bytes(const DspSpace space, const Word address) {
  DspBus bus;
  const std::array program{read_opcode(space), address};
  code(bus, program);
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  cpu.set_registers(initial());
  require(io.memory_windows(Space::program).empty(), "The adapter never grants program windows");
  const auto unknown = cpu.run(1);
  const auto &unknown_failure = io.failure();
  require(unknown.retired == 0 && unknown.fault.kind == FaultKind::memory_read &&
              unknown.fault.space == core_space(space) && unknown.fault.address == address &&
              unknown.fault.pc == 0 && unknown_failure && unknown_failure->cause,
          "An unknown real storage word fails with its original owner cause and exact context");
  const auto revision = bus.program_revision();
  const bool rom = address >= rom_base && address < high_base;
  const auto initialize = [&](const Word value) {
    if (rom) {
      const std::array words{value};
      bus.install_rom({.space = space, .start = address, .words = words});
    } else {
      bus.write({.space = space, .address = address, .value = value});
    }
  };
  initialize(datum);
  cpu.reset();
  io.clear_failure();
  cpu.set_registers(initial());
  const auto result = cpu.run(1);
  require(result.retired == 1 && result.fault.kind == FaultKind::none &&
              cpu.registers() == loaded(datum) &&
              result.progress == Progress{.instructions = 1, .estimated_cycles = 2} &&
              !io.failure(),
          "Explicit recovery and initialization admit the real load with exact full registers");
  initialize(changed);
  cpu.set_registers(initial());
  const auto second = cpu.run(1);
  require(second.retired == 1 && second.fault.kind == FaultKind::none &&
              cpu.registers() == loaded(changed) &&
              second.progress == Progress{.instructions = 2, .estimated_cycles = 4} &&
              bus.program_revision() == revision && !io.failure(),
          "Retained native code reads subsequent owner writes from the actual data storage");
}

void independent_storage(const DspSpace space) {
  DspBus bus;
  const std::array program{read_opcode(space), source, write_opcode(space), destination};
  code(bus, program);
  bus.write({.space = space, .address = source, .value = datum});
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  cpu.set_registers(initial());
  auto expected = loaded(datum);
  expected.pc = 4;
  const auto stored = cpu.run(2);
  const auto offset = destination - high_base;
  require(stored.retired == 2 && stored.fault.kind == FaultKind::none &&
              cpu.registers() == expected && bus.data_windows(space)[2].known[offset] == 1 &&
              bus.read({.space = space, .address = destination}) == datum && !io.failure(),
          "Native load/store initializes the actual high-bank word and knowledge byte");
  DspBus second_bus;
  code(second_bus, program);
  second_bus.write({.space = space, .address = source, .value = changed});
  require(second_bus.data_windows(space)[2].known[offset] == 0 &&
              second_bus.data_windows(space)[0].words.data() !=
                  bus.data_windows(space)[0].words.data(),
          "A fresh device owns distinct storage without inheriting initialized destinations");
  DspIo second_io(second_bus);
  Core second_cpu(Memory::external(), &second_io);
  second_cpu.set_registers(initial());
  const auto second = second_cpu.run(2);
  expected = loaded(changed);
  expected.pc = 4;
  require(second.retired == 2 && second.fault.kind == FaultKind::none &&
              second_cpu.registers() == expected &&
              second_bus.read({.space = space, .address = destination}) == changed &&
              bus.read({.space = space, .address = destination}) == datum && !second_io.failure(),
          "Independent native owners never retain another device's RAM pointer or data");
}

void actual_refusal(const DspSpace space, const Word address, const bool store) {
  DspBus bus;
  const std::array program{store ? write_opcode(space) : read_opcode(space), address};
  code(bus, program);
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  cpu.set_registers(initial());
  const auto result = cpu.run(1);
  const auto &failure = io.failure();
  require(result.retired == 0 &&
              result.fault.kind == (store ? FaultKind::memory_write : FaultKind::memory_read) &&
              result.fault.space == core_space(space) && result.fault.address == address &&
              result.fault.pc == 0 && failure && failure->address == address && failure->pc == 0 &&
              failure->cause != nullptr,
          "The actual owner reports the precise refused operation, bank, address and PC");
}

void native_switch() {
  DspBus bus;
  const std::array program{read_x, source, read_x, switched_tail};
  code(bus, program);
  bus.write({.space = DspSpace::x, .address = source, .value = datum});
  bus.write({.space = DspSpace::x, .address = switched_tail, .value = changed});
  bus.observe_omr(memory_switch);
  bus.advance_instruction_cycles(3);
  bus.observe_omr(0);
  DspIo io(bus);
  Core cpu(Memory::external(), &io);
  cpu.set_registers(initial());
  const auto first = cpu.run(1);
  require(first.retired == 1 && first.fault.kind == FaultKind::none &&
              cpu.registers() == loaded(datum),
          "A pending expanding switch permits the stable low RAM transfer");
  {
    DspIo early_io(bus);
    Core early(Memory::external(), &early_io);
    early.set_registers(cpu.registers());
    const auto stopped = early.run(1);
    require(stopped.retired == 0 && stopped.fault.kind == FaultKind::memory_read &&
                stopped.fault.address == switched_tail && stopped.fault.pc == 2 &&
                early_io.failure(),
            "A separate owner cannot read the high bank before the actual expanding commit");
  }
  bus.advance_instruction_cycles(3);
  const auto after = cpu.run(1);
  auto expected = loaded(changed);
  expected.pc = 4;
  require(after.retired == 1 && after.fault.kind == FaultKind::none &&
              cpu.registers() == expected && !io.failure(),
          "The commit revision rebinds the expanded high RAM before native execution");
  bus.observe_omr(memory_switch);
  bus.advance_instruction_cycles(3);
  require(bus.read({.address = 0x1000}) == changed,
          "The high X tail becomes genuine P storage after the completed remap");
  require(refused([&] {
            static_cast<void>(bus.read({.space = DspSpace::x, .address = switched_tail}));
          }).contains("Unmapped DSP memory"),
          "The remapped physical block cannot retain a writable X alias");
}

} // namespace

int main() {
  try {
    layouts();
    for (const auto space : banks) {
      for (const auto address : {source, destination, rom_base + source}) {
        native_known_bytes(space, address);
      }
      independent_storage(space);
      const auto hole = space == DspSpace::x ? low_x_end : low_y_end;
      for (const auto address : {source, hole, rom_base + 1}) {
        actual_refusal(space, address, false);
      }
      actual_refusal(space, rom_base, true);
    }
    native_switch();
    std::cout << checks << " actual board data-window checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "board data windows: " << error.what() << '\n';
    return 1;
  }
}
