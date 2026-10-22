// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

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
using blofeld::test::require;
using Kind = blofeld::DspNativeAccessKind;
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
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
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

blofeld::DspRunContract contract() {
  return {.clock = {.extal_hz = extal,
                    .timebase_hz = timebase,
                    .switch_timing = blofeld::DspClockSwitch::reference_instruction_boundary},
          .timing = blofeld::DspRunTiming::native_internal_estimates,
          .interrupts = blofeld::DspInterruptTiming::native_vector_estimates};
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
  blofeld::DspMachine machine{contract()};
  unsigned unmask = 0;
  unsigned foreground = 0;
  std::vector<Event> events;
  std::vector<unsigned> vectors;

  Experiment(const blofeld::Firmware &input, std::ostream &output)
      : firmware(input), trace(output) {
    install(machine.bus(), firmware);
    blofeld::test::DspProgram caller;
    caller.immediate("omr", memory_mode);
    for (unsigned index = 0; index < 3; ++index) { caller.emit("nop"); }
    caller.immediate("sr", masked_status);
    caller.immediate("r6", initializer);
    caller.emit("jsr (r6)");
    caller.emit("movep #>$4,x:<<" + blofeld::test::number(priority_register));
    for (unsigned index = 0; index < 8; ++index) {
      const auto suffix = std::to_string(index);
      caller.immediate("r" + suffix, address_seed + index);
      caller.immediate("n" + suffix, offset_seed + index);
      caller.immediate("m" + suffix, word_mask);
    }
    caller.immediate("a0", a_low);
    caller.immediate("a1", a_high);
    caller.immediate("a2", 1);
    caller.immediate("b0", b_low);
    caller.immediate("b1", b_high);
    caller.immediate("b2", b_extension);
    unmask = caller.pc();
    caller.immediate("sr", ordinary_status);
    foreground = caller.pc();
    caller.immediate("sr", masked_status);
    caller.word(absolute_jmp);
    caller.word(unmask);
    caller.install(machine.bus());
    machine.bus().pll().observe_pinit(false);
    machine.cpu().reset_spi_boot();
    machine.bus().shi().observe_spi({.mosi = false});
    unsigned steps = 0;
    while (machine.cpu().state().pc != unmask) {
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

  void observe(const blofeld::DspNativeAccess &access) {
    if (access.stage != blofeld::DspNativeAccessStage::after) { return; }
    if (access.kind == Kind::vector) {
      vectors.push_back(access.target.address);
      return;
    }
    if (access.target.space != x ||
        (access.target.address != hcsr && access.target.address != hrx &&
         access.target.address != htx)) {
      return;
    }
    if (!access.value) { throw blofeld::Fault("Completed SHI access lacks its actual value"); }
    events.push_back({.kind = access.kind,
                      .address = access.target.address,
                      .value = *access.value,
                      .pc = access.pc});
    trace << "IO pc=" << std::hex << access.pc << " address=" << access.target.address
          << " kind=" << static_cast<unsigned>(access.kind) << " value=" << *access.value << '\n';
  }

  void step() {
    machine.cpu().step({.observe = [this](const auto &access) { observe(access); }});
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

  void verify_cpu(const blofeld::DspState &before) const {
    const auto after = machine.cpu().state();
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

  void burst(const std::span<const std::uint32_t> words, Counts &counts) {
    require(machine.cpu().state().pc == unmask && machine.cpu().state().sr == masked_status,
            "An explicit own native caller masks interrupts while buffering SPI words");
    events.clear();
    vectors.clear();
    for (std::size_t index = 0; index < words.size(); ++index) {
      const auto actual = inject(words[index]);
      require(actual == (index < 2 ? idle_marker : words[index - 1]),
              "Two primed words precede the independently expected retained-IOSR echo");
      ++counts.wire;
    }
    const auto before = machine.cpu().state();
    step(); // Actual own MOVE unmasks; no host SR/PC or interrupt-frame writes.
    const auto start = machine.run_clock().native_cycle_units();
    unsigned steps = 0;
    do {
      require(steps++ < step_limit, "Bounded unchanged receive interrupt and actual RTI");
      step();
    } while (machine.cpu().state().pc != foreground || machine.cpu().state().sp != before.sp ||
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
    const auto units = machine.run_clock().native_cycle_units() - start;
    require(steps == expected_steps && units == expected_units,
            "Original IRQ ledger steps=" + std::to_string(steps) + " expected=" +
                std::to_string(expected_steps) + " nominal-units=" + std::to_string(units) +
                " expected=" + std::to_string(expected_units));
    require(machine.bus().read({.space = x, .address = hcsr}) == (config | idle_flag),
            "Original reads/writes empty RX, clear errors and leave HTX pending");
    counts.handler_steps += steps;
    counts.units += units;
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
