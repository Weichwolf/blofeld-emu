// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <string>
#include <utility>

namespace {
using namespace blofeld;
using namespace dsp56300;
constexpr Word low_x_end = 0x3000, low_y_end = 0x2000, rom_base = 0x4000, high_base = 0xc000;
constexpr Word default_x_end = 0x12000, default_y_end = 0x16000;
constexpr Word source = 0x80, destination = 0xc080, datum = 0x9654a1, changed = 0x2579b3;
constexpr Word boot = 0xff0000, jump_zero = 0x0c0000, switched_tail = 0x10000;
constexpr Word read_x = 0x44f000, read_y = 0x4cf000, write_x = 0x447000, write_y = 0x4c7000;
constexpr Word memory_switch = 0x80, memory_switch_shift = 21;
constexpr std::uint64_t initial_x = 0x112233aabbcc;
constexpr unsigned word_bits = 24;
constexpr unsigned extal = 12000000, timebase = 540000000, reset_ticks = 90;
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
  OwnDspIo io(bus);
  Core cpu(io);
  cpu.set_registers(initial());
  require(io.memory_windows(Space::program).empty(), "The adapter never grants program windows");
  const auto before = cpu.registers();
  const auto unknown = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  require(unknown.stop == RunStop::external_effect && unknown.execution.dispatches == 0 &&
              cpu.registers() == before && cpu.progress() == Progress{} && !io.failure(),
          "Unknown RAM stops before all instruction effects and before any fabricated bus read");
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
  const auto result = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  require(result.execution.dispatches == 1 && result.execution.fault.kind == FaultKind::none &&
              cpu.registers() == loaded(datum) &&
              cpu.progress() == Progress{.instructions = 1, .estimated_cycles = 2} &&
              cpu.jit_statistics().native_move_instructions == 1 && !io.failure(),
          "Initializing the real known byte admits a native read with exact full CPU state");
  const auto compiled = cpu.jit_statistics().compiled_blocks;
  initialize(changed);
  cpu.set_registers(initial());
  const auto second = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  require(second.execution.dispatches == 1 && cpu.registers() == loaded(changed) &&
              cpu.jit_statistics().native_move_instructions == 2 &&
              cpu.jit_statistics().compiled_blocks == compiled &&
              bus.program_revision() == revision,
          "Native code reads subsequent owner writes from the same storage without stale data");
}

void store_and_restore(const DspSpace space) {
  DspBus bus;
  const std::array program{read_opcode(space), source, write_opcode(space), destination};
  code(bus, program);
  bus.write({.space = space, .address = source, .value = datum});
  OwnDspIo io(bus);
  Core cpu(io);
  cpu.set_registers(initial());
  require(cpu.run(1).dispatches == 1 && cpu.registers() == loaded(datum),
          "The genuine native load establishes the independent store source");
  const auto checkpoint = cpu.checkpoint();
  auto restored_bus = bus;
  require(restored_bus.data_windows(space)[0].words.data() !=
              bus.data_windows(space)[0].words.data(),
          "The paired bus copy owns different actual RAM allocations");
  const auto stored = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  auto expected = loaded(datum);
  expected.pc = 4;
  const auto offset = destination - high_base;
  require(stored.execution.dispatches == 1 && cpu.registers() == expected &&
              bus.data_windows(space)[2].known[offset] == 1 &&
              bus.read({.space = space, .address = destination}) == datum &&
              cpu.jit_statistics().native_move_instructions == 2,
          "The native store initializes the actual high-bank word and its knowledge byte");
  bus = std::move(restored_bus);
  cpu.restore(checkpoint);
  require(bus.data_windows(space)[2].known[offset] == 0,
          "Paired restore retains the independently saved unknown destination");
  require(cpu.run(1).dispatches == 1 && cpu.registers() == expected &&
              bus.read({.space = space, .address = destination}) == datum &&
              bus.data_windows(space)[2].known[offset] == 1 && !io.failure(),
          "Restore rebinds the native windows before writing the newly owned allocation");
}

void actual_refusal(const DspSpace space, const Word address, const bool store) {
  DspBus bus;
  const std::array program{store ? write_opcode(space) : read_opcode(space), address};
  code(bus, program);
  OwnDspIo io(bus);
  Core cpu(io);
  cpu.set_registers(initial());
  const auto boundary = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  require(boundary.execution.dispatches == 0 && boundary.stop == RunStop::external_effect &&
              cpu.registers() == initial() && cpu.progress() == Progress{} && !io.failure(),
          "Missing storage and readonly stores retain a genuine pre-effect owner boundary");
  const auto result = cpu.run(1);
  const auto &failure = io.failure();
  require(result.dispatches == 0 &&
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
  OwnDspIo io(bus);
  Core cpu(io);
  cpu.set_registers(initial());
  require(cpu.run(RunBudget{.instructions = 1, .cycles = 2}).execution.dispatches == 1,
          "The pending expanding switch still permits a stable low RAM transfer");
  const auto before = cpu.registers();
  const auto stopped = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  require(stopped.execution.dispatches == 0 && stopped.stop == RunStop::external_effect &&
              cpu.registers() == before && !io.failure(),
          "Old native descriptors cannot expose the pending high-bank expansion");
  bus.advance_instruction_cycles(3);
  const auto after = cpu.run(RunBudget{.instructions = 1, .cycles = 2});
  auto expected = loaded(changed);
  expected.pc = 4;
  require(after.execution.dispatches == 1 && cpu.registers() == expected &&
              cpu.jit_statistics().native_move_instructions == 2 && !io.failure(),
          "The actual commit revision rebinds the expanded high RAM before native execution");
  bus.observe_omr(memory_switch);
  bus.advance_instruction_cycles(3);
  require(bus.read({.address = 0x1000}) == changed,
          "The high X tail becomes genuine P storage after the completed remap");
  require(refused([&] {
            static_cast<void>(bus.read({.space = DspSpace::x, .address = switched_tail}));
          }).contains("Unmapped DSP memory"),
          "The remapped physical block cannot retain a writable X alias");
}

struct Observations {
  unsigned reads = 0, writes = 0;
  bool exact = true;

  static void read(void *context, const DspIoReadAccess &access) {
    auto &self = *static_cast<Observations *>(context);
    ++self.reads;
    self.exact = self.exact && access.address.space == DspSpace::x &&
                 access.address.address == source && access.value == datum && access.pc == 0;
  }

  static void write(void *context, const DspStore &store, const unsigned pc) {
    auto &self = *static_cast<Observations *>(context);
    ++self.writes;
    self.exact = self.exact && store.space == DspSpace::x && store.address == destination &&
                 store.value == datum && pc == 2;
  }
};

void observers() {
  OwnDspMachine machine({.extal_hz = extal,
                         .timebase_hz = timebase,
                         .switch_timing = DspClockSwitch::reference_cycle_boundary});
  constexpr std::array bootstrap{jump_zero};
  machine.bus().install_rom({.start = boot, .words = bootstrap});
  const std::array program{read_x, source, write_x, destination, jump_zero};
  code(machine.bus(), program);
  machine.bus().write({.space = DspSpace::x, .address = source, .value = datum});
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The actual machine owns its reset assertion periods");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "The actual machine owns its reset recovery periods");
  require(machine.run_unclocked().dispatches == 1 && machine.cpu().registers().pc == 0,
          "The genuine reset ROM jump reaches the authored RAM program");
  require(machine.run_unclocked().dispatches == 1 && machine.run_unclocked().dispatches == 1 &&
              machine.run_unclocked().dispatches == 1,
          "An unobserved native load/store loop warms genuine direct-window code");
  Observations observed;
  require(machine.run_observed_unclocked({.context = &observed, .completed = Observations::read})
                      .dispatches == 1 &&
              observed.reads == 1 && observed.exact,
          "An attached full-read observer cannot be bypassed by the warm RAM grant");
  auto native = machine.cpu().jit_statistics().native_move_instructions;
  require(machine.run_unclocked().dispatches == 1 &&
              machine.cpu().jit_statistics().native_move_instructions == native + 1 &&
              observed.reads == 1,
          "Observer detachment rebinds direct windows before the next native store");
  require(machine.run_unclocked().dispatches == 1 && machine.run_unclocked().dispatches == 1,
          "The actual program returns to its store after another genuine load");
  require(
      machine.run_observed_unclocked({}, {.context = &observed, .completed = Observations::write})
                  .dispatches == 1 &&
          observed.writes == 1 && observed.exact,
      "A full-write observer retains the actual committed RAM store exactly once");
  require(machine.run_unclocked().dispatches == 1,
          "The real jump resumes without a borrowed observer");
  native = machine.cpu().jit_statistics().native_move_instructions;
  require(machine.run_unclocked().dispatches == 1 &&
              machine.cpu().jit_statistics().native_move_instructions == native + 1,
          "The read path returns to native storage after the write observer is detached");
}
} // namespace

int main() {
  try {
    layouts();
    for (const auto space : banks) {
      for (const auto address : {source, destination, rom_base + source}) {
        native_known_bytes(space, address);
      }
      store_and_restore(space);
      const auto hole = space == DspSpace::x ? low_x_end : low_y_end;
      for (const auto address : {source, hole, rom_base + 1}) {
        actual_refusal(space, address, false);
      }
      actual_refusal(space, rom_base, true);
    }
    native_switch();
    observers();
    std::cout << checks << " actual board data-window checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "board data windows: " << error.what() << '\n';
    return 1;
  }
}
