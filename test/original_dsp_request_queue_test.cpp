// SPDX-License-Identifier: GPL-3.0-only
// Observe original request-queue encoding and scheduler-to-queue calls.
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_request_readiness.hpp"
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
constexpr unsigned boot = 0xff0000, word_mask = 0xffffff, memory_mode = 0x600389;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned scheduler_tail = 0x22d, after_countdown = 0x244;
constexpr unsigned service_countdown = 0x28f2, request_state_data = 0x28fe, service_end = 0x290a;
constexpr unsigned default_queue = 0x1853, data_queue = 0x1856;
constexpr unsigned target_slot = 0x800, mode_slot = 0x801, register_slots = 0x803;
constexpr unsigned rounding_mode = 1U << 21U, scaling_modes = 0xc00;
constexpr unsigned queue_frontier = 0x1876, table_compare = 0x186a;
constexpr unsigned table_base = 0x26a5, table_end = 0x26b5;
constexpr unsigned head_word = 0x2a14, tail_word = 0x2a15, ring_base = 0x2700;
constexpr unsigned guard_base = 0x13579b, guard_stride = 0x10203, y_guard_bias = 0x456789;
constexpr unsigned original_record_count = 127, step_limit = 200, ring_size = 256, last_head = 252;
constexpr unsigned guard_words = 64, pending_guard = 0x2468ac;
constexpr unsigned ring_guard = 0x234567, ring_guard_stride = 0x12345;
constexpr unsigned table_guard = 0x345678, before_guard = 0x765432, after_guard = 0x654321;
constexpr unsigned word_bits = 24, data_bits = 48, byte_mask = 255;
constexpr unsigned seed_slot = 0x11, timer_slot = 0x1e, threshold_slot = 0x1d, count_slot = 0x19;
constexpr unsigned pending_slot = 0x1c, entry = 0xc500, scratch = 0x700;
constexpr unsigned timer_control = 0xffff8b, timer_input = 12, compare_enable = 2;
constexpr unsigned input_wait = 0x188a, service_return = 0x1879, transfer_entry = 0x187e;
constexpr unsigned probe_limit = 32;

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
  unsigned id, target, seed, tick, timer, threshold, count, status, flags, mode;
  std::uint64_t initial_a, initial_b;
  unsigned x0, y0, y1, x1, r0, m0, head, tail, scan_stop;
  unsigned ready_profile = 0;
  std::array<unsigned, 16> bases{};
};

struct Fixture {
  blofeld::DspBus bus;
  blofeld::DspCore cpu;
  const Source &source;
  unsigned caller_return = 0, steps = 0;
  std::ostream &trace;
  unsigned probes_returned = 0, probes_faulted = 0;
  unsigned long long probe_instructions = 0;
  unsigned long long instructions = 0;
  std::set<unsigned> visited;
  bool readiness = false;

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
    for (const auto *const reg : {"a2", "a1", "a0", "b2", "b1", "b0"}) {
      code.emit("move x:>" + blofeld::test::number(slot++) + ',' + reg);
    }
    code.emit("move x:>" + blofeld::test::number(register_slots + 10) + ",x0");
    code.emit("move x0,m0");
    for (const auto *const reg : {"x0", "y0", "y1"}) {
      code.emit("move x:>" + blofeld::test::number(slot++) + ',' + reg);
    }
    code.emit("move x:>" + blofeld::test::number(mode_slot) + ",x1");
    code.emit("move x1,sr");
    code.emit("move x:>" + blofeld::test::number(register_slots + 11) + ",x1");
    code.emit("move x:>" + blofeld::test::number(register_slots + 9) + ",r0");
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

  void put(unsigned at, unsigned v) { bus.write({.space = x, .address = at, .value = v}); }

  unsigned get(unsigned at) { return bus.read({.space = x, .address = at}); }

  void step() {
    const auto pc = cpu.state().pc;
    require(steps++ < step_limit, "Request queue instruction bound");
    const bool firmware = (pc >= default_queue && pc < queue_frontier) ||
                          (pc >= scheduler_tail && pc < after_countdown) ||
                          (pc >= service_countdown && pc < service_end);
    require(firmware || (pc >= boot && pc < caller_return), "Undeclared queue instruction");
    if (pc == table_compare) {
      require(cpu.state().r[1] < table_end, "Outside table prefetch executed");
    }
    if (firmware) { visited.insert(pc); }
    cpu.step();
    bus.advance_instruction_cycles(1);
    ++instructions;
  }

  void state(const char *label, const blofeld::DspState &s) {
    trace << label << std::hex << ' ' << s.pc << ' ' << s.sp << ' ' << s.stack_pc << ' ' << s.a
          << ' ' << s.b << ' ' << s.x << ' ' << s.y << ' ' << s.sr;
    for (auto v : s.r) { trace << ' ' << v; }
    for (auto v : s.m) { trace << ' ' << v; }
    trace << '\n';
    require((s.sr & scaling_modes) == 0, "Accumulator scaling changed");
    for (auto v : s.n) { require(v == 0, "Unused offset register changed"); }
  }

  void state(const char *label) { state(label, cpu.state()); }

  void ram() {
    trace << "LOW" << std::hex;
    for (auto space : {x, y}) {
      for (unsigned at = 0; at < guard_words; ++at) {
        trace << ' ' << bus.read({.space = space, .address = at});
      }
    }
    trace << "\nRING";
    for (unsigned at = ring_base - 1; at <= ring_base + ring_size; ++at) {
      trace << ' ' << get(at);
    }
    trace << "\nTABLE";
    for (unsigned at = table_base - 1; at <= table_end; ++at) { trace << ' ' << get(at); }
    trace << "\nCURSORS " << get(head_word) << ' ' << get(tail_word) << '\n';
  }

  std::array<unsigned, 12> prepare(const Case &c) {
    require(c.mode == 0 || c.mode == rounding_mode, "Unsupported rounding mode");
    require(c.head >= ring_base && c.head <= ring_base + last_head && (c.head & 3U) == 0,
            "Unaligned queue head");
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
    put(4, c.tick);
    put(timer_slot, c.timer);
    put(threshold_slot, c.threshold);
    put(count_slot, c.count);
    put(7, c.status);
    put(6, c.flags);
    put(pending_slot, pending_guard);
    for (unsigned at = ring_base - 1; at <= ring_base + ring_size; ++at) {
      put(at, (ring_guard + ((at - (ring_base - 1)) * ring_guard_stride)) & word_mask);
    }
    put(table_base - 1, table_guard);
    put(table_end, y_guard_bias);
    for (unsigned i = 0; i < 16; ++i) { put(table_base + i, c.bases[i]); }
    put(head_word, c.head);
    put(tail_word, c.tail);
    put(target_slot - 1, before_guard);
    put(mode_slot + 1, after_guard);
    put(target_slot, c.target);
    put(mode_slot, c.mode);
    const std::array<unsigned, 12> args{
        static_cast<unsigned>((c.initial_a >> data_bits) & byte_mask),
        static_cast<unsigned>((c.initial_a >> word_bits) & word_mask),
        static_cast<unsigned>(c.initial_a & word_mask),
        static_cast<unsigned>((c.initial_b >> data_bits) & byte_mask),
        static_cast<unsigned>((c.initial_b >> word_bits) & word_mask),
        static_cast<unsigned>(c.initial_b & word_mask),
        c.x0,
        c.y0,
        c.y1,
        c.r0,
        c.m0,
        c.x1};
    for (unsigned i = 0; i < args.size(); ++i) { put(register_slots + i, args[i]); }
    return args;
  }

  struct ProbeReads {
    unsigned clears = 0;
    unsigned pending = 0;
  };

  static void observe_timer(const blofeld::DspNativeAccess &access, ProbeReads &reads) {
    if (access.kind != blofeld::DspNativeAccessKind::bit ||
        access.target.address != timer_control) {
      return;
    }
    if (access.target.space != x || !access.bit.has_value()) {
      throw blofeld::Fault("Expected actual TCSR1 bit read");
    }
    const auto bit = access.bit.value().index;
    require(access.bit.value().operation == blofeld::DspBitOperation::test,
            "Timer probe must not write a register");
    if (bit == compare_enable) {
      require(access.pc == queue_frontier, "TCIE read has its original instruction context");
      if (access.stage == blofeld::DspNativeAccessStage::after) {
        require(access.value == 0, "Reset TCIE is actually clear");
        ++reads.clears;
      }
    } else {
      require(bit == timer_input && access.pc == input_wait, "Expected original TIO1 input read");
      require(access.stage == blofeld::DspNativeAccessStage::before && !access.value.has_value(),
              "Unknown TIO1 has no supplied value");
      ++reads.pending;
    }
  }

  void probe(const Case &c) {
    const auto expected_head = ring_base + ((c.head - ring_base + 4) % ring_size);
    const bool empty = expected_head == c.tail;
    const auto returned = c.target == scheduler_tail ? after_countdown : caller_return;
    const auto expected_sp = c.target == scheduler_tail ? 2U : 1U;
    unsigned attempts = 0;
    ProbeReads reads;
    std::string error;
    blofeld::DspState before;
    const blofeld::DspNativeAccessObserver observer{
        .observe = [&](const blofeld::DspNativeAccess &access) { observe_timer(access, reads); }};
    while (cpu.state().pc != returned) {
      require(attempts++ < probe_limit, "Original queue service probe instruction bound");
      before = cpu.state();
      require((before.pc >= queue_frontier && before.pc <= service_return) ||
                  (before.pc >= transfer_entry && before.pc <= input_wait),
              "Probe left original service window");
      try {
        cpu.step(observer);
        bus.advance_instruction_cycles(1);
        ++probe_instructions;
      } catch (const blofeld::Fault &failure) {
        error = failure.what();
        break;
      }
    }
    require(reads.clears == 1 && get(head_word) == expected_head && get(tail_word) == c.tail,
            "Actual TCIE decision preserves both queue cursors");
    if (empty) {
      require(error.empty() && reads.pending == 0 && cpu.state().pc == returned &&
                  cpu.state().sp == expected_sp - 1,
              "Original empty queue uses its genuine RTS");
      ++probes_returned;
      trace << "SERVICE_RETURN " << std::hex << returned << ' ' << cpu.state().sp << '\n';
      state("SERVICE_STATE");
    } else {
      require(reads.pending == 1 && before.pc == input_wait && before.sp == expected_sp &&
                  error.contains("pc=0x00188a") && error.contains("omr=0x600389") &&
                  error.contains("Unknown DSP timer DI read at X:0xffff8b"),
              "Original queue must fail visibly on unobserved TIO1");
      bool rejected = false;
      try {
        cpu.step();
      } catch (const blofeld::Fault &) { rejected = true; }
      require(rejected, "Failed service CPU requires diagnostic reset");
      ++probes_faulted;
      trace << "SERVICE_FAULT " << error << '\n';
      state("SERVICE_STATE", before);
    }
  }

  void run(const Case &c) {
    const auto args = prepare(c);
    steps = 0;
    trace << "CASE " << std::hex << c.id << '\n';
    while (cpu.state().pc != c.target) { step(); }
    state("ENTRY");
    if (c.target == scheduler_tail || c.target == request_state_data) {
      const auto end = c.target == scheduler_tail ? default_queue : data_queue;
      while (cpu.state().pc != end) { step(); }
      state("HANDOFF");
    }
    while (cpu.state().pc != data_queue) { step(); }
    state("QUEUE");
    while (cpu.state().pc != queue_frontier) {
      if (cpu.state().pc == table_compare && cpu.state().r[1] == table_end) { break; }
      step();
    }
    require((cpu.state().pc == table_compare) == (c.scan_stop != 0), "Unexpected table frontier");
    require((cpu.state().sr & rounding_mode) == c.mode, "Rounding mode changed");
    state("END");
    ram();
    require(get(target_slot) == c.target && get(mode_slot) == c.mode &&
                get(target_slot - 1) == before_guard && get(mode_slot + 1) == after_guard,
            "Caller guard changed");
    for (unsigned i = 0; i < args.size(); ++i) {
      require(get(register_slots + i) == args[i], "Caller argument changed");
    }
    trace << "STEPS " << std::dec << steps << '\n';
    if (readiness) {
      blofeld::test::request::probe(*this, c);
    } else if (c.scan_stop == 0) {
      probe(c);
    }
    for (const auto &[at, v] : source.program) {
      require(bus.peek_program(at) == v, "Original program changed");
    }
  }
};
} // namespace

int main(const int count, char **arguments) {
  try {
    require(count == 4 || count == 5, "Expected original firmware, cases, trace and optional mode");
    const Source source(arguments[1]);
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    Fixture fixture(source, trace);
    if (count == 5) {
      require(std::string(arguments[4]) == "readiness", "Unknown queue-test mode");
      fixture.readiness = true;
    }
    std::ifstream cases(arguments[2]);
    require(cases.is_open(), "Missing queue cases");
    Case c{};
    unsigned completed = 0;
    while (cases >> std::hex >> c.id >> c.target >> c.seed >> c.tick >> c.timer >> c.threshold >>
           c.count >> c.status >> c.flags >> c.mode >> c.initial_a >> c.initial_b >> c.x0 >> c.y0 >>
           c.y1 >> c.x1 >> c.r0 >> c.m0 >> c.head >> c.tail >> c.scan_stop) {
      for (auto &value : c.bases) {
        require(static_cast<bool>(cases >> value), "Truncated base table");
      }
      if (fixture.readiness) {
        require(static_cast<bool>(cases >> c.ready_profile), "Missing controlled input profile");
      }
      fixture.run(c);
      ++completed;
    }
    require(cases.eof(), "Malformed queue cases");
    trace << "VISITED" << std::hex;
    for (auto pc : fixture.visited) { trace << ' ' << pc; }
    trace << "\nTOTAL " << std::dec << completed << ' ' << fixture.instructions << '\n';
    trace << "PROBE_TOTAL " << fixture.probes_returned << ' ' << fixture.probes_faulted << ' '
          << fixture.probe_instructions << '\n';
    std::cout << "Original request descriptors and timer-input frontier checked.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
