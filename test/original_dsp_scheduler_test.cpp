// SPDX-License-Identifier: GPL-3.0-only
// Original scheduler observations adapted from the source checkpoint recorded in
// doc/dsp-scheduler-reference.json. Independent equations remain in Python.
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <set>
#include <string>

namespace {
using enum blofeld::DspSpace;
using blofeld::test::require;
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned scheduler_tail = 0x22d;
constexpr unsigned countdown_test = 0x242;
constexpr unsigned after_countdown = 0x244;
constexpr unsigned scheduler_end = 0x246;
constexpr unsigned service_countdown = 0x28f2;
constexpr unsigned request_state_data = 0x28fe;
constexpr unsigned service_end = 0x290a;
constexpr unsigned main_loop = 0x198;
constexpr unsigned default_queue = 0x1853;
constexpr unsigned data_queue = 0x1856;
constexpr unsigned target_slot = 0x800;
constexpr unsigned mode_slot = 0x801;
constexpr unsigned register_slots = 0x803;
constexpr unsigned rounding_mode = 1U << 21U;
constexpr unsigned scaling_modes = 0xc00;
constexpr unsigned transition_flag = 0x80;
constexpr unsigned guard_base = 0x13579b;
constexpr unsigned guard_stride = 0x10203;
constexpr unsigned y_guard_bias = 0x456789;
constexpr unsigned pending_guard = 0x2468ac;
constexpr unsigned before_guard = 0x765432;
constexpr unsigned after_guard = 0x654321;
constexpr unsigned sound = 0x1000;
constexpr unsigned entry = 0xc500;
constexpr unsigned scratch = 0x700;
constexpr unsigned original_record_count = 127;
constexpr unsigned step_limit = 100;
constexpr unsigned guard_words = 64;
constexpr unsigned word_bits = 24;
constexpr unsigned data_bits = 48;
constexpr unsigned byte_mask = 255;
constexpr unsigned seed_slot = 0x11;
constexpr unsigned tick_slot = 4;
constexpr unsigned timer_slot = 0x1e;
constexpr unsigned threshold_slot = 0x1d;
constexpr unsigned count_slot = 0x19;
constexpr unsigned pending_slot = 0x1c;

struct Source {
  blofeld::Firmware firmware;
  std::map<unsigned, unsigned> program;

  explicit Source(const std::string &path) : firmware(blofeld::Firmware::load(path)) {
    require(firmware.dsp.size() == original_record_count,
            "All original ordered records are supplied");
    std::set<unsigned> overwritten;
    for (const auto &record : firmware.dsp) {
      if (record.space != 0) { continue; }
      for (unsigned i = 0; i < record.words.size(); ++i) {
        if (program.contains(record.address + i)) { overwritten.insert(record.address + i); }
        program[record.address + i] = record.words[i];
      }
    }
    constexpr unsigned second_vector_word = 0x11;
    require(overwritten == std::set<unsigned>{16, second_vector_word},
            "Only the original later two-word vector replacement is present");
  }
};

struct Case {
  unsigned id = 0, target = 0, seed = 0, tick = 0, timer = 0, threshold = 0, count = 0, status = 0,
           flags = 0, mode = 0;
  std::uint64_t initial_a = 0, initial_b = 0;
  unsigned initial_x0 = 0, initial_y0 = 0, initial_y1 = 0;
};

struct Fixture {
  blofeld::DspBus bus;
  blofeld::DspCore cpu;
  const Source &source;
  std::ostream &trace;
  unsigned caller_return = 0;
  std::set<unsigned> visited;
  unsigned long long instructions = 0;
  unsigned steps = 0;

  Fixture(const Source &image, std::ostream &output) : cpu(bus), source(image), trace(output) {
    harness();
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (const auto &record : source.firmware.dsp) {
      for (unsigned i = 0; i < record.words.size(); ++i) {
        bus.write({.space = static_cast<blofeld::DspSpace>(record.space),
                   .address = record.address + i,
                   .value = record.words[i]});
      }
    }
    trace << "CALLER_RETURN " << std::hex << caller_return << '\n';
  }

  void harness() {
    blofeld::test::DspProgram code;
    code.immediate("x1", memory_mode);
    code.emit("move x1,omr");
    for (unsigned i = 0; i < 3; ++i) { code.emit("nop"); }
    const auto begin = code.pc();
    for (unsigned i = 0; i < 8; ++i) { code.immediate("m" + std::to_string(i), word_mask); }
    for (unsigned i = 0; i < 8; ++i) { code.immediate("n" + std::to_string(i), 0); }
    unsigned slot = register_slots;
    for (const auto *const reg : {"a2", "a1", "a0", "b2", "b1", "b0", "x0", "y0", "y1"}) {
      code.emit("move x:>" + blofeld::test::number(slot++) + ',' + reg);
    }
    code.emit("move x:>" + blofeld::test::number(mode_slot) + ",x1");
    code.emit("move x1,sr");
    code.immediate("r0", sound);
    code.immediate("r2", entry);
    code.immediate("r5", entry);
    code.immediate("r3", scratch);
    code.immediate("r4", target_slot);
    code.immediate("r6", 0);
    code.immediate("r7", 0);
    code.emit("move x:>" + blofeld::test::number(target_slot) + ",r1");
    code.emit("jsr (r1)");
    caller_return = code.pc();
    code.emit("nop");
    code.word(absolute_jump);
    code.word(begin);
    code.install(bus);
  }

  void put(const unsigned at, const unsigned value) {
    bus.write({.space = x, .address = at, .value = value});
  }

  unsigned get(const unsigned at) { return bus.read({.space = x, .address = at}); }

  static bool firmware_pc(const unsigned pc) {
    return (pc >= scheduler_tail && pc < scheduler_end) ||
           (pc >= service_countdown && pc < service_end);
  }

  void step() {
    const auto pc = cpu.state().pc;
    require(steps++ < step_limit, "Scheduler window instruction bound");
    require(firmware_pc(pc) || (pc >= boot && pc < caller_return),
            "Undeclared scheduler instruction at " + blofeld::hex(pc, 6));
    if (firmware_pc(pc)) { visited.insert(pc); }
    cpu.step();
    bus.advance_instruction_cycles(1);
    ++instructions;
  }

  void state(const char *phase) {
    const auto s = cpu.state();
    trace << phase << std::hex << ' ' << s.pc << ' ' << s.sp << ' ' << s.stack_pc << ' ' << s.a
          << ' ' << s.b << ' ' << s.x << ' ' << s.y << ' ' << s.sr;
    for (auto value : s.r) { trace << ' ' << value; }
    trace << '\n';
    require((s.sr & scaling_modes) == 0, "Unexpected accumulator scaling");
    for (auto value : s.m) { require(value == word_mask, "Linear modifier changed"); }
    for (auto value : s.n) { require(value == 0, "Untouched offset register changed"); }
  }

  void ram(const char *phase) {
    trace << phase << std::hex;
    for (auto space : {x, y}) {
      for (unsigned at = 0; at < guard_words; ++at) {
        trace << ' ' << bus.read({.space = space, .address = at});
      }
    }
    trace << '\n';
  }

  std::array<unsigned, 9> prepare(const Case &c) {
    require(c.mode == 0 || c.mode == rounding_mode, "Invalid declared rounding mode");
    require((c.flags & transition_flag) == 0, "Parameter transition is outside this window");
    cpu.reset_spi_boot();
    for (auto space : {x, y}) {
      for (unsigned at = 0; at < guard_words; ++at) {
        bus.write({.space = space,
                   .address = at,
                   .value = (guard_base + (at * guard_stride) + (space == y ? y_guard_bias : 0U)) &
                            word_mask});
      }
    }
    put(seed_slot, c.seed);
    put(tick_slot, c.tick);
    put(timer_slot, c.timer);
    put(threshold_slot, c.threshold);
    put(count_slot, c.count);
    put(7, c.status);
    put(6, c.flags);
    put(pending_slot, pending_guard);
    put(target_slot - 1, before_guard);
    put(mode_slot + 1, after_guard);
    put(target_slot, c.target);
    put(mode_slot, c.mode);
    const std::array<unsigned, 9> incoming{
        static_cast<unsigned>((c.initial_a >> data_bits) & byte_mask),
        static_cast<unsigned>((c.initial_a >> word_bits) & word_mask),
        static_cast<unsigned>(c.initial_a & word_mask),
        static_cast<unsigned>((c.initial_b >> data_bits) & byte_mask),
        static_cast<unsigned>((c.initial_b >> word_bits) & word_mask),
        static_cast<unsigned>(c.initial_b & word_mask),
        c.initial_x0,
        c.initial_y0,
        c.initial_y1};
    for (unsigned i = 0; i < incoming.size(); ++i) { put(register_slots + i, incoming[i]); }
    return incoming;
  }

  void run(const Case &c) {
    const auto incoming = prepare(c);
    steps = 0;
    trace << "CASE " << std::hex << c.id << '\n';
    while (cpu.state().pc != c.target) { step(); }
    state("ENTRY");
    if (c.target == scheduler_tail) {
      while (cpu.state().pc != countdown_test) { step(); }
      require(cpu.state().sp == 1 && cpu.state().stack_pc == caller_return,
              "Original incoming scheduler frame differs");
      state("PREFIX");
      ram("PREFIX_RAM");
      const auto before = cpu.state();
      step(); // The original JSEQ creates its actual extra return frame.
      const auto after = cpu.state();
      require(before.a == after.a && before.b == after.b && before.x == after.x &&
                  before.y == after.y,
              "Conditional call changed numerical endpoints");
      if (after.pc == service_countdown) {
        require(after.sp == 2 && after.stack_pc == after_countdown,
                "Original conditional call frame differs");
      } else {
        require(after.pc == after_countdown && after.sp == 1 && after.stack_pc == caller_return,
                "Original noncalling branch frame differs");
      }
      state("CALL");
      while (cpu.state().pc != main_loop && cpu.state().pc != default_queue) { step(); }
    } else if (c.target == service_countdown) {
      while (cpu.state().pc != caller_return && cpu.state().pc != default_queue) { step(); }
    } else if (c.target == request_state_data) {
      while (cpu.state().pc != data_queue) { step(); }
    } else {
      throw blofeld::Fault("Unknown scheduler case target");
    }
    state("END");
    ram("END_RAM");
    require((cpu.state().sr & rounding_mode) == c.mode, "Declared rounding mode changed");
    require(get(target_slot) == c.target && get(mode_slot) == c.mode &&
                get(target_slot - 1) == before_guard && get(mode_slot + 1) == after_guard,
            "Caller argument or neighboring guard changed");
    for (unsigned i = 0; i < incoming.size(); ++i) {
      require(get(register_slots + i) == incoming[i], "Caller register argument changed");
    }
    for (const auto &[at, value] : source.program) {
      require(bus.peek_program(at) == value, "Original executable word changed");
    }
    trace << "STEPS " << std::dec << steps << '\n';
  }
};
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original firmware, directed cases and trace path");
    const Source source(arguments[1]);
    std::ifstream cases(arguments[2]);
    require(cases.is_open(), "Missing scheduler cases");
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    Fixture fixture(source, trace);
    Case c{};
    unsigned completed = 0;
    while (cases >> std::hex >> c.id >> c.target >> c.seed >> c.tick >> c.timer >> c.threshold >>
           c.count >> c.status >> c.flags >> c.mode >> c.initial_a >> c.initial_b >> c.initial_x0 >>
           c.initial_y0 >> c.initial_y1) {
      fixture.run(c);
      ++completed;
    }
    require(cases.eof(), "Malformed directed scheduler cases");
    trace << "VISITED" << std::hex;
    for (auto pc : fixture.visited) { trace << ' ' << pc; }
    trace << "\nTOTAL " << std::dec << completed << ' ' << fixture.instructions << '\n';
    std::cout << "Original scheduler windows executed with actual call and queue frames.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
