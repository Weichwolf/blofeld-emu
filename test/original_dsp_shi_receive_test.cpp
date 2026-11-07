// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_io.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"
#include "firmware/firmware.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
using enum blofeld::DspSpace;
using blofeld::test::DspRomRegister;
using blofeld::test::esai::require;
enum class Kind : std::uint8_t { read = 2, write = 3 };
constexpr unsigned call_r6 = 0x0be680; // FM JSR (R6).
constexpr unsigned maximum_effects = 64;
constexpr unsigned route_record = 0x23df;
constexpr unsigned route_words = 189;
constexpr unsigned idle_steps = 17;
constexpr unsigned tagged_steps = 18;
constexpr unsigned idle_units = 36;
constexpr unsigned tagged_units = 38;
// Original initializer: nine non-NOP instructions and 256 repeated NOP steps.
constexpr unsigned initializer_steps = 265;
// Nominal table: 256 NOPs, seven two-unit MOVEs, REP=5, RTS=3.
// BSCLR already has a fixed five-unit estimate on either conditional path.
constexpr unsigned initializer_units = 256 + (7 * 2) + 5 + 3;
constexpr unsigned rx_vector = 0x44;
constexpr unsigned error_vector = 0x4a;
constexpr unsigned initializer = 0x2462;
constexpr unsigned original_transmit = 0x245d;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned ordinary_status = 0xc00000;
constexpr unsigned priority_register = 0xfffffe;
constexpr unsigned peripheral_offset_mask = 0x3f;
constexpr unsigned priority_movep = 0x08f480 | (priority_register & peripheral_offset_mask);
// FM MOVEP #imm,X:pp. The documented pp field selects the actual SHI priority register.
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
static_assert(blofeld::test::dma_test::native_clocks.extal_hz == extal &&
              blofeld::test::dma_test::native_clocks.timebase_hz == timebase);
constexpr unsigned step_limit = 2048;
constexpr unsigned word_bits = 24;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned config = 0x1029;
constexpr unsigned idle_marker = 0x800000;
constexpr unsigned tx_empty = 0x8000;
constexpr unsigned tx_underrun = 0x4000;
constexpr unsigned rx_nonempty = 0x20000;
constexpr unsigned rx_full = 0x80000;
constexpr unsigned rx_overrun = 0x100000;
constexpr unsigned idle_flag = 0x200;
constexpr unsigned fifo_capacity = 10;
constexpr unsigned ring_base = 0x2c80;
constexpr unsigned ring_words = 64;
constexpr unsigned read_cursor = 0x2c34;
constexpr unsigned write_cursor = 0x2c35;
constexpr unsigned note_on = 0x903c64;
constexpr unsigned note_off = 0x803c00;
constexpr unsigned controller = 0xb0077f;
constexpr unsigned tagged_note_zero = 0x800001;
constexpr unsigned reset_word = 0x123456;
constexpr unsigned address_seed = 0x123450;
constexpr unsigned offset_seed = 0x246870;
constexpr unsigned a_low = 0x654321;
constexpr unsigned a_high = 0x123456;
constexpr unsigned b_low = 0xabcdef;
constexpr unsigned b_high = 0x345678;
constexpr unsigned b_extension = 0xfe;
// DSP56300FM Rev. 5 JMP ea: absolute extension, MMMRRR=110000.
constexpr unsigned absolute_jmp = 0x0af080;

struct Case {
  const char *name;
  std::vector<std::uint32_t> words;
  unsigned burst;
};

std::vector<std::uint32_t> tagged_words(const unsigned count) {
  constexpr std::array pattern{note_on, note_off, controller, tagged_note_zero};
  std::vector<std::uint32_t> result;
  result.reserve(count);
  for (unsigned index = 0; index < count; ++index) {
    result.push_back(pattern[index % pattern.size()]);
  }
  return result;
}

std::span<const std::uint32_t> processed_words(const std::span<const std::uint32_t> words) {
  std::size_t count = 0;
  for (const auto word : words.first(std::min<std::size_t>(words.size(), fifo_capacity))) {
    ++count;
    if ((word & idle_marker) == 0) { break; } // Original reset discards the remaining FIFO.
  }
  return words.first(count);
}

std::vector<std::uint32_t> wrapped_reset_words() {
  auto words = tagged_words(ring_words);
  words.push_back(0);
  words.push_back(note_on);
  return words;
}

void install(blofeld::DspBus &bus, const blofeld::Firmware &firmware) {
  bus.observe_omr(memory_mode);
  bus.advance_instruction_cycles(3);
  unsigned found = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0 || (record.address != route_record && record.address != rx_vector &&
                              record.address != error_vector)) {
      continue;
    }
    require(record.words.size() == (record.address == route_record ? route_words : 2U),
            "Complete unchanged source records are required");
    ++found;
    for (unsigned index = 0; index < record.words.size(); ++index) {
      bus.write({.space = p, .address = record.address + index, .value = record.words[index]});
    }
  }
  require(found == 3, "Original receiver and both original stored vectors are supplied in order");
}

struct Event {
  Kind kind;
  unsigned address;
  unsigned value;
  unsigned pc;
};

struct CallerStep {
  unsigned pc;
  std::uint64_t units;
};

struct Counts {
  unsigned runs = 0;
  unsigned wire = 0;
  unsigned received = 0;
  unsigned processed = 0;
  unsigned resets = 0;
  unsigned flushed = 0;
  unsigned queued = 0;
  unsigned idle = 0;
  unsigned lost = 0;
  unsigned overwrites = 0;
  unsigned handler_steps = 0;
  std::uint64_t units = 0;
};

struct Experiment {
  const blofeld::Firmware &firmware;
  std::ostream &trace;
  blofeld::DspMachine machine{blofeld::test::dma_test::native_clocks};
  unsigned unmask = 0;
  unsigned foreground = 0;
  std::vector<Event> events;
  std::vector<CallerStep> caller_steps;
  std::vector<unsigned> vectors;
  bool vector_read = false;

  Experiment(const blofeld::Firmware &input, std::ostream &output)
      : firmware(input), trace(output) {
    install(machine.bus(), firmware);
    blofeld::test::DspRomProgram caller;
    caller.immediate(DspRomRegister::omr, memory_mode);
    for (unsigned index = 0; index < 3; ++index) { caller.word(0); }
    caller.immediate(DspRomRegister::sr, masked_status);
    caller.immediate(DspRomRegister::r6, initializer);
    caller.word(call_r6);
    caller.word(priority_movep);
    caller.word(4);
    for (unsigned index = 0; index < 8; ++index) {
      caller.immediate(
          static_cast<DspRomRegister>(static_cast<unsigned>(DspRomRegister::r0) + index),
          address_seed + index);
      caller.immediate(
          static_cast<DspRomRegister>(static_cast<unsigned>(DspRomRegister::n0) + index),
          offset_seed + index);
      caller.immediate(
          static_cast<DspRomRegister>(static_cast<unsigned>(DspRomRegister::m0) + index),
          word_mask);
    }
    caller.immediate(DspRomRegister::a0, a_low);
    caller.immediate(DspRomRegister::a1, a_high);
    caller.immediate(DspRomRegister::a2, 1);
    caller.immediate(DspRomRegister::b0, b_low);
    caller.immediate(DspRomRegister::b1, b_high);
    caller.immediate(DspRomRegister::b2, b_extension);
    unmask = caller.pc();
    caller.immediate(DspRomRegister::sr, ordinary_status);
    foreground = caller.pc();
    caller.immediate(DspRomRegister::sr, masked_status);
    caller.word(absolute_jmp);
    caller.word(unmask);
    blofeld::test::dma_test::NativeProgram program;
    for (const auto word : caller.words()) { program.word(word); }
    blofeld::test::dma_test::reset_native(machine, program);
    machine.bus().shi().observe_spi({.mosi = false});
    unsigned steps = 0;
    while (machine.cpu().registers().pc != unmask) {
      require(steps++ < step_limit, "Bounded own caller and unchanged original initializer");
      step();
    }
    require(events.size() == 4 && events[0].kind == Kind::write && events[0].address == hcsr &&
                events[0].value == 0 && events[1].address == hcsr && events[1].value == config &&
                events[2].address == htx && events[2].value == idle_marker &&
                events[3].address == htx && events[3].value == idle_marker && vectors.empty(),
            "The real initializer resets/configures SHI and performs both HTX writes");
    require(machine.bus().read({.space = x, .address = read_cursor}) == ring_base &&
                machine.bus().read({.space = x, .address = write_cursor}) == ring_base,
            "Original initializer establishes both software cursors");
    events.clear();
  }

  void observe(const Kind kind,
               const blofeld::DspAddress target,
               const unsigned value,
               const unsigned pc) {
    if (target.space == p && vector_read) {
      vectors.push_back(target.address);
      return;
    }
    if (target.space != x ||
        (target.address != hcsr && target.address != hrx && target.address != htx)) {
      return;
    }
    events.push_back({.kind = kind, .address = target.address, .value = value, .pc = pc});
    trace << "IO pc=" << std::hex << pc << " address=" << target.address
          << " kind=" << static_cast<unsigned>(kind) << " value=" << value << '\n';
  }

  void admit() {
    const auto state = machine.cpu().registers();
    if (!machine.bus().interrupt_request((state.sr >> 8) & 3U) ||
        machine.cpu().instruction_fetch().exception_vector) {
      return;
    }
    try {
      const auto opcode = machine.bus().inspect_program(state.pc);
      dsp56300::InstructionWords foreground_word{.address = state.pc, .opcode = opcode};
      const auto shape = dsp56300::inspect_instruction(opcode);
      if (!shape) {
        throw blofeld::Fault("Interrupt eligibility lacks a documented foreground encoding");
      }
      if (shape.value().words == 2) {
        foreground_word.extension = machine.bus().inspect_program(state.pc + 1);
        require(foreground_word.extension.has_value(), "Actual foreground extension is known");
      }
      static_cast<void>(machine.admit_interrupt(foreground_word));
    } catch (const blofeld::Fault &error) {
      throw blofeld::Fault(std::string(error.what()) + " eligibility-pc=" +
                           blofeld::hex(state.pc, 6) + " omr=" + blofeld::hex(state.omr, 6));
    }
  }

  void healthy(const dsp56300::InstructionProgress &progress) const {
    if (progress.fault.kind == dsp56300::FaultKind::none) { return; }
    std::string cause = "Original SHI native instruction fault";
    const auto &failure = machine.io().failure();
    if (failure && failure->cause) {
      try {
        std::rethrow_exception(failure->cause);
      } catch (const std::exception &error) { cause = error.what(); }
    }
    throw blofeld::Fault(cause + " pc=" + blofeld::hex(progress.fault.pc, 6) +
                         " omr=" + blofeld::hex(machine.cpu().registers().omr, 6) +
                         " address=" + blofeld::hex(progress.fault.address, 6));
  }

  void step() {
    admit();
    const auto before = machine.cpu().progress();
    const auto ticks = machine.clock().elapsed_ticks();
    auto progress = machine.begin_fetch(dsp56300::InstructionPublication::retirement);
    for (unsigned effect = 0; effect < maximum_effects && progress.pending && !progress.retired;
         ++effect) {
      const auto pending = progress.pending.value();
      vector_read = machine.interrupt_pending() &&
                    (pending.effect.kind == dsp56300::EffectKind::fetch_opcode ||
                     pending.effect.kind == dsp56300::EffectKind::fetch_extension);
      progress = machine.advance_observed(
          pending.ticket,
          {.context = this,
           .completed =
               [](void *context, const blofeld::DspIoReadAccess &access) {
                 static_cast<Experiment *>(context)->observe(
                     Kind::read, access.address, access.value, access.pc);
               }},
          {.context = this,
           .completed = [](void *context, const blofeld::DspStore &store, const unsigned pc) {
             static_cast<Experiment *>(context)->observe(
                 Kind::write, {.space = store.space, .address = store.address}, store.value, pc);
           }});
    }
    vector_read = false;
    healthy(progress);
    require(progress.retired && machine.cpu().progress().instructions == before.instructions + 1 &&
                machine.clock().elapsed_ticks() == ticks,
            "One genuine staged instruction retires without implicit time");
    const auto cycles = machine.cpu().progress().estimated_cycles - before.estimated_cycles;
    const auto outcome = machine.cpu().instruction_outcome();
    if (!outcome) { throw blofeld::Fault("Retired SHI instruction lacks its actual outcome"); }
    trace << "NATIVE pc=" << std::hex << outcome->address << " units=" << std::dec << cycles
          << '\n';
    if (outcome->address >= blofeld::test::DspRomProgram::boot) {
      caller_steps.push_back({.pc = outcome->address, .units = cycles});
    }
    for (std::uint64_t cycle = 0; cycle < cycles; ++cycle) {
      static_cast<void>(machine.advance_reference_cycle());
    }
  }

  unsigned inject(const unsigned word) {
    auto &shi = machine.bus().shi();
    shi.observe_spi({.selected = true, .mosi = false});
    unsigned received = 0;
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      const bool high = ((word >> (word_bits - 1 - bit)) & 1U) != 0;
      shi.observe_spi({.selected = true, .clock = true, .mosi = high});
      shi.observe_spi({.selected = true, .clock = false, .mosi = high});
      const auto output = shi.outputs().miso;
      if (!output.driven || !output.level) { throw blofeld::Fault("Unknown original MISO data"); }
      received = (received << 1U) | static_cast<unsigned>(*output.level);
    }
    shi.observe_spi({.mosi = false});
    return received;
  }

  void verify_cpu(const dsp56300::Registers &before) const {
    const auto after = machine.cpu().registers();
    require(after.pc == foreground && after.sp == before.sp && after.sr == ordinary_status &&
                after.omr == before.omr && after.vba == before.vba && after.la == before.la &&
                after.lc == before.lc && after.x == before.x && after.y == before.y &&
                after.a == before.a && after.b == before.b && after.r == before.r &&
                after.n == before.n && after.m == before.m,
            "Real interrupt/RTI preserves the independent caller register contract");
  }

  void verify_reset_writes(const std::size_t first) const {
    const std::array values{0U, config, idle_marker, idle_marker};
    const std::array addresses{hcsr, hcsr, htx, htx};
    for (std::size_t item = 0; item < values.size(); ++item) {
      const auto &event = events[first + item];
      require(event.kind == Kind::write && event.address == addresses[item] &&
                  event.value == values[item],
              "The original nested initializer actually resets, configures and primes SHI");
    }
    require(events[first].pc == initializer, "Actual original reset entry performs HCSR=0");
  }

  void verify_events(const std::span<const std::uint32_t> words, Counts &counts) const {
    const auto accepted = std::min<std::size_t>(words.size(), fifo_capacity);
    const auto processed = processed_words(words);
    const bool reset = (processed.back() & idle_marker) == 0;
    require(events.size() == processed.size() * 3 + (reset ? 4U : 0U) &&
                vectors.size() == processed.size() * 2,
            "Every actual receive has both vector fetches and HCSR/HRX/HTX accesses");
    for (std::size_t index = 0; index < processed.size(); ++index) {
      const auto &status = events[index * 3];
      const auto &receive = events[(index * 3) + 1];
      const bool calls_reset = (words[index] & idle_marker) == 0;
      const auto &transmit = events[(index * 3) + 2 + (calls_reset ? 4U : 0U)];
      const bool overrun = index == 0 && words.size() > fifo_capacity;
      const auto vector = overrun ? error_vector : rx_vector;
      const unsigned expected = config | idle_flag | rx_nonempty | (index < 2 ? tx_empty : 0U) |
                                (index == 0 && words.size() > 2 ? tx_underrun : 0U) |
                                (accepted - index == fifo_capacity ? rx_full : 0U) |
                                (overrun ? rx_overrun : 0U);
      require(vectors[index * 2] == vector && vectors[(index * 2) + 1] == vector + 1 &&
                  status.kind == Kind::read && status.address == hcsr && status.value == expected &&
                  receive.kind == Kind::read && receive.address == hrx &&
                  receive.value == words[index] && transmit.kind == Kind::write &&
                  transmit.address == htx && transmit.pc == original_transmit &&
                  transmit.value == idle_marker,
              "Independent FIFO/error/vector/order/transmit expectations match original execution");
      if (calls_reset) {
        verify_reset_writes((index * 3) + 2);
        ++counts.resets;
      }
      if (index >= 2 || calls_reset) { ++counts.overwrites; }
    }
  }

  [[nodiscard]] std::uint64_t verify_caller(const std::size_t processed) const {
    // The library excludes IRQ admission until one foreground retirement after RTI.
    // That is a declared boundary policy, not calibrated hardware pipeline timing.
    // Our caller then masks, jumps and unmasks; none belongs to the original handler.
    constexpr unsigned caller_stride = 3;
    constexpr std::array caller_units{2U, 4U, 2U}; // FM MOVEC #imm,SR; JMP ea; MOVEC.
    const std::array caller_pc{foreground, foreground + 2, unmask};
    require(processed != 0 && caller_steps.size() == (processed - 1) * caller_stride,
            "Every inter-handler caller trip has three independently specified instructions");
    std::uint64_t units = 0;
    for (std::size_t index = 0; index < caller_steps.size(); ++index) {
      const auto phase = index % caller_stride;
      const auto &actual = caller_steps[index];
      require(actual.pc == caller_pc[phase] && actual.units == caller_units[phase],
              "Actual authored mask/jump/unmask sequence and nominal costs remain separate");
      units += actual.units;
    }
    return units;
  }

  void burst(const std::span<const std::uint32_t> words, Counts &counts) {
    require(machine.cpu().registers().pc == unmask && machine.cpu().registers().sr == masked_status,
            "An explicit own native caller masks interrupts while buffering SPI words");
    events.clear();
    vectors.clear();
    for (std::size_t index = 0; index < words.size(); ++index) {
      const auto actual = inject(words[index]);
      require(actual == (index < 2 ? idle_marker : words[index - 1]),
              "Two primed words precede the independently expected retained-IOSR echo");
      ++counts.wire;
    }
    const auto before = machine.cpu().registers();
    step(); // Actual own MOVE unmasks; no host SR/PC or interrupt-frame writes.
    caller_steps.clear();
    const auto start = machine.clock().native_cycle_units();
    unsigned steps = 0;
    do {
      require(steps++ < step_limit, "Bounded unchanged receive interrupt and actual RTI");
      step();
    } while (machine.cpu().registers().pc != foreground ||
             machine.cpu().registers().sp != before.sp ||
             machine.bus().interrupt_request(0).has_value());
    verify_cpu(before);
    verify_events(words, counts);
    unsigned expected_steps = 0;
    unsigned expected_units = 0;
    for (const auto word : processed_words(words)) {
      const bool idle = (word & ~idle_marker) == 0;
      // Source path: one vector JSR plus 16/17 body instructions; nominal vector/mnemonic costs.
      expected_steps += idle ? idle_steps : tagged_steps;
      expected_units += idle ? idle_units : tagged_units;
      if ((word & idle_marker) == 0) {
        expected_steps += initializer_steps;
        expected_units += initializer_units;
      }
    }
    const auto caller_units = verify_caller(processed_words(words).size());
    const auto units = machine.clock().native_cycle_units() - start;
    require(steps == expected_steps + caller_steps.size() && units == expected_units + caller_units,
            "Original IRQ ledger steps=" + std::to_string(steps) + " expected=" +
                std::to_string(expected_steps) + " nominal-units=" + std::to_string(units) +
                " expected=" + std::to_string(expected_units));
    require(machine.bus().read({.space = x, .address = hcsr}) == (config | idle_flag),
            "Original reads/writes empty RX, clear errors and leave HTX pending");
    trace << "CALLER steps=" << std::dec << caller_steps.size() << " units=" << caller_units
          << '\n';
    counts.handler_steps += steps - static_cast<unsigned>(caller_steps.size());
    counts.units += units - caller_units;
  }

  void verify_program() {
    for (const auto &record : firmware.dsp) {
      if (record.space != 0 || (record.address != route_record && record.address != rx_vector &&
                                record.address != error_vector)) {
        continue;
      }
      for (unsigned index = 0; index < record.words.size(); ++index) {
        require(machine.bus().inspect_program(record.address + index) == record.words[index],
                "Original program and stored vectors remain unchanged");
      }
    }
  }

  void execute(const Case &item, Counts &counts) {
    std::array<std::optional<std::uint32_t>, ring_words> expected;
    unsigned writer = ring_base;
    for (std::size_t offset = 0; offset < item.words.size();) {
      const auto words =
          std::span(item.words)
              .subspan(offset, std::min<std::size_t>(item.burst, item.words.size() - offset));
      burst(words, counts);
      const auto accepted = std::min<std::size_t>(words.size(), fifo_capacity);
      counts.received += static_cast<unsigned>(accepted);
      counts.lost += static_cast<unsigned>(words.size() - accepted);
      const auto processed = processed_words(words);
      counts.processed += static_cast<unsigned>(processed.size());
      counts.flushed += static_cast<unsigned>(accepted - processed.size());
      for (const auto word : processed) {
        expected[writer - ring_base] = word; // Even an idle word performs this original store.
        if ((word & idle_marker) == 0) { writer = ring_base; }
        if (word == idle_marker) { ++counts.idle; }
        if ((word & idle_marker) != 0 && (word & ~idle_marker) != 0) {
          writer = ring_base + ((writer - ring_base + 1) % ring_words);
          ++counts.queued;
        }
      }
      require(machine.bus().read({.space = x, .address = read_cursor}) == ring_base &&
                  machine.bus().read({.space = x, .address = write_cursor}) == writer,
              "Independent stores precede reset rewind, idle filtering and modulo-64 publication");
      for (unsigned index = 0; index < ring_words; ++index) {
        const auto &cell = expected[index];
        if (cell) {
          require(machine.bus().read({.space = x, .address = ring_base + index}) == *cell,
                  "Every actually written queue cell matches the independent input sequence");
        }
      }
      offset += words.size();
      if (offset != item.words.size()) {
        step(); // Own caller masks interrupts again.
        step(); // Own caller jumps back to its unmask boundary.
      }
    }
    verify_program();
    ++counts.runs;
  }
};

void run(const blofeld::Firmware &firmware, std::ostream &trace) {
  const std::array cases{
      Case{.name = "one-tagged", .words = {note_on}, .burst = 1},
      Case{.name = "one-idle", .words = {idle_marker}, .burst = 1},
      Case{.name = "three-tagged", .words = tagged_words(3), .burst = 3},
      Case{.name = "three-idle", .words = {idle_marker, idle_marker, idle_marker}, .burst = 3},
      Case{.name = "mixed",
           .words = {idle_marker, note_on, tagged_note_zero, idle_marker, controller},
           .burst = 5},
      Case{.name = "full-fifo", .words = tagged_words(10), .burst = 10},
      Case{.name = "overrun", .words = tagged_words(11), .burst = 11},
      Case{.name = "ring-alias", .words = tagged_words(ring_words), .burst = 8},
      Case{.name = "reset-zero", .words = {0}, .burst = 1},
      Case{.name = "reset-word", .words = {reset_word}, .burst = 1},
      Case{.name = "reset-flush", .words = {reset_word, note_on, controller}, .burst = 3},
      Case{.name = "tag-before-reset", .words = {note_on, reset_word, controller}, .burst = 3},
      Case{.name = "reset-overrun",
           .words = {reset_word,
                     note_on,
                     note_off,
                     controller,
                     tagged_note_zero,
                     note_on,
                     note_off,
                     controller,
                     tagged_note_zero,
                     note_on,
                     note_off},
           .burst = 11},
      Case{.name = "reset-after-wrap", .words = wrapped_reset_words(), .burst = 8},
      Case{.name = "repeated-reset",
           .words = {note_on, reset_word, idle_marker, 0, controller},
           .burst = 1},
      Case{.name = "mixed-reset",
           .words = {note_on, idle_marker, reset_word, tagged_note_zero},
           .burst = 4},
      Case{.name = "zero-after-tag", .words = {note_on, 0, controller}, .burst = 3},
      Case{.name = "recover-after-reset", .words = {reset_word, note_on, controller}, .burst = 1}};
  Counts counts;
  for (const auto &item : cases) {
    trace << "SETUP " << item.name << '\n';
    Experiment experiment(firmware, trace);
    const auto checkpoint = experiment.machine.checkpoint();
    for (unsigned replay = 0; replay < 2; ++replay) {
      if (replay != 0) { experiment.machine.restore(checkpoint); }
      trace << "CASE " << item.name << " replay=" << std::dec << replay << '\n';
      experiment.execute(item, counts);
    }
  }
  trace << "RESULT cases=" << std::dec << cases.size() << " runs=" << counts.runs
        << " wire=" << counts.wire << " received=" << counts.received << " queued=" << counts.queued
        << " processed=" << counts.processed << " resets=" << counts.resets
        << " flushed=" << counts.flushed << " idle=" << counts.idle << " lost=" << counts.lost
        << " overwrites=" << counts.overwrites << " steps=" << counts.handler_steps
        << " units=" << counts.units << '\n';
}
}

int main(int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware and private trace path");
    std::ofstream trace(arguments[2]);
    require(static_cast<bool>(trace), "Cannot write private original SHI trace");
    run(blofeld::Firmware::load(arguments[1]), trace);
    require(static_cast<bool>(trace), "Incomplete original SHI trace");
    std::cout
        << "Original SHI reception, nested reset, FIFO, transmit and native RTI checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
