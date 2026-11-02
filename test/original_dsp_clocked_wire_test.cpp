// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"
#include "esai_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {
using blofeld::test::DspRomRegister;
using blofeld::test::esai::require;
using enum blofeld::DspSpace;
constexpr unsigned producer_program = 0x2cd9;
constexpr unsigned vector = 0x54;
constexpr unsigned physical_alias = 0xf000;
constexpr unsigned timestamp = 0x1f;
constexpr unsigned timestamp_seed = 0x13579b;
constexpr unsigned memory_mode = 0x600085;
constexpr unsigned status = 0xc00000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned capture_control = 0x165;
constexpr unsigned capture_return = 0x2d03;
constexpr unsigned sample_return = 0x2d10;
constexpr unsigned input_return = 0x2cf8;
constexpr unsigned step_budget = 1000000;
constexpr unsigned producer = 0x3b;
constexpr unsigned ring_base = 0x2f80;
constexpr unsigned ring_words = 64;
constexpr unsigned sentinel = 0x2468ac;
constexpr unsigned control = 0xffff8f;
constexpr unsigned data_mask = 0x3fefff;
constexpr unsigned sampling_period = 3249;
constexpr unsigned timebase = 540000000;
constexpr unsigned extal = 12000000;
constexpr unsigned unit_ticks = timebase / extal;
constexpr std::uint64_t bit_ticks = std::uint64_t{sampling_period} * 2 * unit_ticks;

constexpr unsigned third_irq = 22;
constexpr unsigned input_handler = 0x2ce2;
constexpr unsigned changed_third = 0x63;
constexpr unsigned changed_event = 27;
constexpr unsigned data_shift = 16;
constexpr unsigned parser = 0xc53;
constexpr unsigned parameters_record = 0x808;
constexpr unsigned parser_record = 0x967;
constexpr unsigned parser_end = 0xdab;
constexpr unsigned callback = 0xcbb;
constexpr unsigned callback_return = 0xc05;
constexpr unsigned field = 0xab6;
constexpr unsigned field_seed = 0x246813;
constexpr unsigned midi_channel = 0x744;
constexpr unsigned midi_clock = 0x750;
constexpr unsigned mode_flags = 6;
constexpr unsigned consumer = 0x3c;
constexpr unsigned word_bits = 24;
// FM JSR (R6), high-X MOVEP #imm, and NOP: authored caller only.
constexpr unsigned call_r6 = 0x0be680;
constexpr unsigned priority_immediate = 0x08f4be;
constexpr auto timing =
    blofeld::OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts;
using NativeState = std::pair<dsp56300::Registers, dsp56300::Progress>;

constexpr blofeld::DspRunClockInputs clocks{.extal_hz = extal,
                                            .timebase_hz = timebase,
                                            .switch_timing =
                                                blofeld::DspClockSwitch::reference_cycle_boundary};

struct AttemptedEffect {
  unsigned pc;
  std::uint64_t start_ticks;
  std::uint64_t completion_ticks;
  std::uint64_t units;
  dsp56300::Progress progress;
};

struct NativeStep {
  dsp56300::Registers before;
  unsigned pc;
  bool interrupt;
};

NativeState native_state(const blofeld::OwnDspMachine &machine) {
  return {machine.cpu().registers(), machine.cpu().progress()};
}

void drive(blofeld::OwnDspMachine &machine, const bool level) {
  machine.bus().timers().set_tio_drive(0, level);
  machine.bus().timers().observe_synchronized_tio(0, level);
}

void load(blofeld::OwnDspMachine &machine, const blofeld::Firmware &firmware) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != producer_program && record.address != vector &&
          record.address != parameters_record && record.address != parser_record) {
        continue;
      }
      ++programs;
    } else {
      const unsigned limit = record.space == 1 ? 0x3000 : 0x2000;
      if (record.address + record.words.size() > limit) { continue; }
    }
    for (std::size_t i = 0; i < record.words.size(); ++i) {
      const bool alias = record.space == 0 && record.address == producer_program;
      machine.bus().write(
          {.space = alias ? x : static_cast<blofeld::DspSpace>(record.space),
           .address = record.address + static_cast<unsigned>(i) + (alias ? physical_alias : 0U),
           .value = record.words[i]});
    }
  }
  require(programs == 4, "All complete unchanged original producer/vector/parser components load");
}

struct Foreground {
  unsigned begin;
  unsigned end;
};

Foreground setup(blofeld::OwnDspMachine &machine, const bool parse) {
  auto &bus = machine.bus();
  bus.write({.space = x, .address = producer, .value = ring_base});
  bus.write({.space = x, .address = mode_flags, .value = 0});
  bus.write({.space = x, .address = midi_channel, .value = 0});
  bus.write({.space = x, .address = midi_clock, .value = 1});
  bus.write({.space = x, .address = field, .value = field_seed});
  bus.write({.space = x, .address = field - 1, .value = sentinel});
  bus.write({.space = x, .address = field + 1, .value = sentinel});
  bus.write({.space = x, .address = timestamp, .value = timestamp_seed});
  for (unsigned i = 0; i < ring_words; ++i) {
    bus.write({.space = x, .address = ring_base + i, .value = sentinel ^ i});
  }
  bus.write({.space = x, .address = ring_base - 1, .value = sentinel});
  bus.write({.space = x, .address = ring_base + ring_words, .value = sentinel});
  blofeld::test::DspRomProgram code;
  code.immediate(DspRomRegister::omr, memory_mode);
  for (unsigned i = 0; i < 3; ++i) { code.word(0); }
  code.immediate(DspRomRegister::sr, status);
  code.immediate(DspRomRegister::a1, timestamp_seed);
  for (unsigned i = 0; i < 8; ++i) {
    code.immediate(static_cast<DspRomRegister>(static_cast<unsigned>(DspRomRegister::m0) + i),
                   word_mask);
  }
  code.word(priority_immediate);
  code.word(0x200);
  code.immediate(DspRomRegister::r6, producer_program);
  code.word(call_r6);
  const auto foreground = code.pc();
  if (parse) {
    code.immediate(DspRomRegister::r6, parser);
    code.word(call_r6);
  }
  code.word(0);
  code.word(absolute_jump);
  code.word(foreground);
  const auto end = code.pc();
  blofeld::test::dma_test::NativeProgram program;
  for (const auto word : code.words()) { program.word(word); }
  blofeld::test::dma_test::reset_native(machine, program);
  drive(machine, true);
  return {.begin = foreground, .end = end};
}

struct Event {
  std::uint64_t ticks;
  bool level;
};

struct Wire {
  blofeld::OwnDspMachine &machine;
  std::vector<Event> events;
  std::size_t cursor = 0;
  std::optional<std::size_t> unknown;

  Wire(blofeld::OwnDspMachine &owner,
       const std::uint64_t start,
       const std::span<const unsigned> bytes)
      : machine(owner) {
    for (unsigned index = 0; index < bytes.size(); ++index) {
      const auto at = start + (static_cast<std::uint64_t>(index) * 11 * bit_ticks);
      events.push_back({.ticks = at, .level = false});
      for (unsigned bit = 0; bit < 8; ++bit) {
        events.push_back(
            {.ticks = at + ((bit + 1) * bit_ticks), .level = ((bytes[index] >> bit) & 1U) != 0});
      }
      events.push_back({.ticks = at + (9 * bit_ticks), .level = true});
    }
  }

  [[nodiscard]] std::optional<std::uint64_t> next_ticks() const {
    return cursor < events.size() ? std::optional(events.at(cursor).ticks) : std::nullopt;
  }

  void observe(const std::uint64_t ticks) {
    const auto index = cursor++;
    const auto event = events.at(index);
    require(ticks == event.ticks, "Every serial edge has its declared timestamp");
    if (unknown && index == *unknown) {
      machine.bus().timers().set_tio_drive(0, std::nullopt);
    } else {
      drive(machine, event.level);
    }
  }
};

struct Counts {
  unsigned interrupts = 0;
  unsigned returns = 0;
  unsigned calls = 0;
  unsigned completions = 0;
  bool operator==(const Counts &) const = default;
};

struct Signature {
  NativeState cpu;
  std::uint64_t ticks;
  std::uint64_t units;
  std::uint64_t steps;
  std::uint64_t delivered;
  std::vector<unsigned> values;
  Counts counts;
  bool operator==(const Signature &) const = default;
};

unsigned bend(const unsigned lsb, const unsigned msb) {
  constexpr int center = 8192;
  constexpr int scale = 1024;
  const auto centered = static_cast<int>(lsb | (msb << 7)) - center;
  return static_cast<unsigned>(centered * scale) & word_mask;
}

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected an explicit native input/checkpoint failure");
}

Foreground
prepare(blofeld::OwnDspMachine &machine, const blofeld::Firmware &firmware, const bool parse) {
  load(machine, firmware);
  return setup(machine, parse);
}

struct Rig {
  const blofeld::Firmware &firmware;
  blofeld::OwnDspMachine machine{clocks};
  std::vector<unsigned> input;
  std::vector<unsigned> expected_pitch;
  bool parse;
  Foreground foreground;
  std::uint64_t start = 0;
  std::uint64_t end = 0;
  Wire wire;
  std::unique_ptr<blofeld::OwnDspTimeline> schedule;
  std::optional<dsp56300::Registers> instruction_before;
  dsp56300::Registers sampled_before;
  Counts counts;
  bool inside = false;
  std::optional<AttemptedEffect> attempted;

  Rig(const blofeld::Firmware &image,
      const std::span<const unsigned> bytes,
      const std::span<const unsigned> expected,
      const bool consume)
      : firmware(image),
        input(bytes.begin(), bytes.end()),
        expected_pitch(expected.begin(), expected.end()),
        parse(consume),
        foreground(prepare(machine, image, consume)),
        wire(machine, 0, {}) {
    schedule = std::make_unique<blofeld::OwnDspTimeline>(machine, timing, 0);
    bool configured = false;
    for (unsigned cycles = 0; cycles < 2048 && !configured; ++cycles) {
      const auto retired = cycle(schedule->next_deadline());
      configured = retired && machine.cpu().registers().pc == foreground.begin;
    }
    require(configured && machine.bus().timers().read(control, data_mask) == capture_control &&
                machine.bus().timers().read(control - 2) == 1,
            "Actual native caller and unchanged original setup establish capture mode");
    start = machine.clock().elapsed_ticks() + bit_ticks;
    end = start + (input.size() * 11 * bit_ticks);
    // The initial Wire has no events: reception starts only after real configuration.
    Wire configured_wire(machine, start, input);
    wire.events = std::move(configured_wire.events);
  }

  void observe(const NativeStep &step) {
    if (step.interrupt) {
      const auto cpu = machine.cpu().registers();
      require(cpu.sp == step.before.sp + 1 && (cpu.stack.at(cpu.sp) >> word_bits) == step.before.pc,
              "Each original JSR vector creates the genuine interrupted foreground frame");
      ++counts.interrupts;
      return;
    }
    const auto pc = step.pc;
    if (pc == capture_return || pc == sample_return || pc == input_return) {
      require(machine.cpu().registers().sp + 1 == step.before.sp,
              "Each actual original RTI removes exactly one native frame");
      ++counts.returns;
    }
    if (pc == callback) {
      require(parse && step.before.sp == 2 &&
                  (step.before.stack.at(step.before.sp) >> word_bits) == callback_return,
              "The original parser establishes its genuine nested callback frame");
      ++counts.calls;
    }
    if (pc == callback_return) {
      require(counts.completions < expected_pitch.size() &&
                  counts.calls == counts.completions + 1 &&
                  machine.bus().read({.space = x, .address = field}) ==
                      expected_pitch.at(counts.completions),
              "Callback result differs from independent signed14-bit MIDI expectations");
      ++counts.completions;
    }
    if (pc == input_handler && counts.interrupts == third_irq) { inside = true; }
  }

  bool cycle(const std::uint64_t deadline) {
    if (!instruction_before || wire.unknown) { sampled_before = machine.cpu().registers(); }
    // Only the unknown-input case consumes this per-cycle failure diagnostic.
    if (wire.unknown) {
      const auto pending = machine.cpu().pending_instruction();
      attempted = AttemptedEffect{.pc = pending ? pending->effect.pc : sampled_before.pc,
                                  .start_ticks = machine.clock().elapsed_ticks(),
                                  .completion_ticks = deadline,
                                  .units = machine.clock().native_cycle_units(),
                                  .progress = machine.cpu().progress()};
    }
    const auto last = machine.bus().last_dma_cycle();
    const auto result =
        schedule->advance_one(deadline,
                              blofeld::OwnDspBoundary::through,
                              {.cycle = last ? *last + 1 : 0,
                               .known_requests = 15,
                               .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
                               .grant = blofeld::DspDmaGrant::available});
    if (!result) { throw blofeld::Fault("Actual native cycle missed its owned deadline"); }
    if (result->instruction_started) {
      require(!instruction_before, "A new instruction cannot overlap its prior observation");
      instruction_before = sampled_before;
    }
    if (result->native && result->native->fault.kind != dsp56300::FaultKind::none) {
      const auto &fault = result->native->fault;
      std::string cause = "Original native clocked-wire fault";
      const auto &failure = machine.io().failure();
      if (failure && failure->cause) {
        try {
          std::rethrow_exception(failure->cause);
        } catch (const std::exception &error) { cause = error.what(); }
      }
      throw blofeld::Fault("DSP reference step pc=" + blofeld::hex(fault.pc, 6) +
                           " omr=" + blofeld::hex(machine.cpu().registers().omr, 6) + ": " + cause);
    }
    if (!result->native || !result->native->retired) { return false; }
    const auto outcome = machine.cpu().instruction_outcome();
    if (!outcome || !instruction_before) {
      throw blofeld::Fault("Native retirement lost its actual instruction/entry observations");
    }
    observe({.before = *instruction_before,
             .pc = outcome->address,
             .interrupt = outcome->address == vector || outcome->address == vector + 2});
    instruction_before.reset();
    return true;
  }

  [[nodiscard]] std::uint64_t next_event_ticks() const {
    const auto native = schedule->next_deadline();
    const auto external = wire.next_ticks();
    return external && *external < native ? *external : native;
  }

  struct CycleBudget {
    unsigned maximum = step_budget;
  };

  void advance(const std::uint64_t horizon, const CycleBudget budget = {.maximum = step_budget}) {
    unsigned cycles = 0;
    unsigned delivered = 0;
    while (true) {
      const auto native = schedule->next_deadline();
      const auto external = wire.next_ticks();
      const auto next = external && *external < native ? *external : native;
      if (next > horizon) { return; }
      if (external && *external <= native) {
        require(delivered++ < 128, "Bounded declared external-before-DSP event delivery");
        wire.observe(*external);
      } else {
        require(cycles++ < budget.maximum,
                "Bounded native reference cycles; no instruction is skipped");
        static_cast<void>(cycle(native));
      }
    }
  }

  void ring(const unsigned delivered) {
    for (unsigned slot = 0; slot < ring_words; ++slot) {
      unsigned expected = sentinel ^ slot;
      if (delivered > slot) {
        const auto latest = slot + (((delivered - 1 - slot) / ring_words) * ring_words);
        expected = input.at(latest) << data_shift;
      }
      require(machine.bus().read({.space = x, .address = ring_base + slot}) == expected,
              "Actual receiver word/guard differs at ring slot " + std::to_string(slot));
    }
    require(machine.bus().read({.space = x, .address = producer}) ==
                ring_base + (delivered % ring_words),
            "Original producer cursor differs after the declared byte prefix");
    for (const auto at : {ring_base - 1, ring_base + ring_words, field - 1, field + 1}) {
      require(machine.bus().read({.space = x, .address = at}) == sentinel,
              "Original receiver/parser damaged an adjacent guard");
    }
  }

  void seek() {
    for (unsigned steps = 0; !inside; ++steps) {
      require(steps < step_budget, "Original third-byte ISR checkpoint bound exhausted");
      advance(next_event_ticks(), {.maximum = 1});
    }
    require(counts.interrupts == third_irq && counts.returns + 1 == counts.interrupts &&
                machine.cpu().registers().sp > 0 &&
                machine.cpu().registers().pc == input_handler + 1 && counts.calls == 0 &&
                counts.completions == 0 && wire.cursor < changed_event &&
                machine.bus().read({.space = x, .address = field}) == field_seed &&
                machine.bus().read({.space = x, .address = producer}) == ring_base + 2 &&
                machine.bus().read({.space = x, .address = consumer}) == ring_base + 2,
            "Snapshot is inside the genuine third-bit ISR with a consumed saved LSB only");
  }

  void finish() {
    advance(end);
    require(wire.cursor == wire.events.size() && counts.interrupts == input.size() * 9 &&
                counts.returns == counts.interrupts && counts.calls == expected_pitch.size() &&
                counts.completions == counts.calls && machine.cpu().registers().sp <= 1 &&
                !machine.bus().interrupt_request(0),
            "All original serial/IRQ/parser/callback effects finish naturally");
    const auto pc = machine.cpu().registers().pc;
    require((pc >= foreground.begin && pc < foreground.end) ||
                (parse && pc >= parser_record && pc < parser_end),
            "Completed continuation remains in the declared own loop/original parser");
    ring(static_cast<unsigned>(input.size()));
    if (parse) {
      require(!expected_pitch.empty() &&
                  machine.bus().read({.space = x, .address = consumer}) ==
                      ring_base + (input.size() % ring_words) &&
                  machine.bus().read({.space = x, .address = field}) == expected_pitch.back(),
              "Actual consumer/callback endpoints differ");
    }
  }

  Signature signature() {
    std::vector<unsigned> values;
    for (const auto &record : firmware.dsp) {
      if (record.space == 0) {
        if (record.address != producer_program && record.address != vector &&
            record.address != parameters_record && record.address != parser_record) {
          continue;
        }
        for (unsigned i = 0; i < record.words.size(); ++i) {
          values.push_back(machine.bus().inspect_program(record.address + i));
        }
      } else {
        const auto limit = record.space == 1 ? 0x3000U : 0x2000U;
        if (record.address + record.words.size() > limit) { continue; }
        for (unsigned i = 0; i < record.words.size(); ++i) {
          values.push_back(
              machine.bus().read({.space = static_cast<blofeld::DspSpace>(record.space),
                                  .address = record.address + i}));
        }
      }
    }
    for (unsigned i = 0; i < ring_words; ++i) {
      values.push_back(machine.bus().read({.space = x, .address = ring_base + i}));
    }
    for (const auto at : {producer, consumer, timestamp, field, field - 1, field + 1}) {
      values.push_back(machine.bus().read({.space = x, .address = at}));
    }
    for (const unsigned offset : {0U, 2U, 3U}) {
      values.push_back(machine.bus().timers().read(control - offset));
    }
    values.push_back(machine.bus().pll().read());
    return {.cpu = native_state(machine),
            .ticks = machine.clock().elapsed_ticks(),
            .units = machine.clock().native_cycle_units(),
            .steps = machine.cpu().progress().instructions,
            .delivered = wire.cursor,
            .values = std::move(values),
            .counts = counts};
  }
};

void test_bytes(const blofeld::Firmware &firmware, std::ostream &trace) {
  std::array<unsigned, 256> bytes{};
  for (unsigned i = 0; i < bytes.size(); ++i) { bytes.at(i) = i; }
  Rig rig(firmware, bytes, {}, false);
  for (unsigned delivered = 1; delivered <= bytes.size(); ++delivered) {
    rig.advance(rig.start + (static_cast<std::uint64_t>(delivered) * 11 * bit_ticks));
    rig.ring(delivered);
  }
  rig.finish();
  trace << "RESULT scope=bytes bytes=" << bytes.size() << " interrupts=" << rig.counts.interrupts
        << " returns=" << rig.counts.returns << " callbacks=" << rig.counts.completions
        << " ring-wraps=" << bytes.size() / ring_words << '\n';
}

void test_pitch(const blofeld::Firmware &firmware, std::ostream &trace) {
  constexpr std::array bytes{0xe0U, 0U, 0xf8U, 0U, 127U, 127U, 0U, 64U, 1U, 64U, 127U, 63U};
  constexpr std::array expected{0x800000U, 0x7ffc00U, 0U, 0x400U, 0xfffc00U};
  Rig rig(firmware, bytes, expected, true);
  rig.finish();
  trace << "RESULT scope=pitch bytes=" << bytes.size() << " interrupts=" << rig.counts.interrupts
        << " returns=" << rig.counts.returns << " callbacks=" << rig.counts.completions << '\n';
}

void test_checkpoint(const blofeld::Firmware &firmware, std::ostream &trace) {
  constexpr std::array bytes{0xe0U, 0x35U, 0x23U};
  const std::array expected{bend(bytes.at(1), bytes.at(2))};
  Rig rig(firmware, bytes, expected, true);
  rig.seek();
  const auto owner = rig.schedule->checkpoint();
  const auto cursor = rig.wire.cursor;
  const auto counts = rig.counts;
  const auto original_events = rig.wire.events;
  const auto restore = [&] {
    rig.schedule.reset();
    rig.schedule = std::make_unique<blofeld::OwnDspTimeline>(rig.machine, owner);
    rig.instruction_before.reset();
    rig.wire.events = original_events;
    rig.wire.cursor = cursor;
    rig.wire.unknown.reset();
    rig.input.at(2) = bytes.at(2);
    rig.expected_pitch.at(0) = expected.at(0);
    rig.counts = counts;
    rig.attempted.reset();
  };
  restore();
  rig.finish();
  const auto first_suffix = rig.signature();
  for (unsigned attempt = 1; attempt < 3; ++attempt) {
    restore();
    if (attempt == 2) {
      require(!rig.wire.events.at(changed_event).level && cursor < changed_event,
              "Changed bit belongs strictly to the unobserved input suffix");
      rig.wire.events.at(changed_event).level = true;
      rig.input.at(2) = changed_third;
      rig.expected_pitch.at(0) = bend(bytes.at(1), changed_third);
    }
    rig.finish();
    const auto state = rig.signature();
    if (attempt == 1) {
      require(state == first_suffix, "Identical whole observed native suffix differs");
    } else {
      require(state.values != first_suffix.values && rig.expected_pitch.at(0) != expected.at(0),
              "Changed future input must produce the independent changed original MIDI result");
    }
  }
  restore();
  rig.wire.unknown = changed_event;
  const auto error = rejects([&] { rig.finish(); });
  constexpr unsigned fault_pc = 0x2ce4;
  require(
      error.contains("DSP reference step pc=0x002ce4 omr=0x600085") &&
          error.contains("Unknown DSP timer DI read at X:0xffff8f") && rig.schedule->faulted() &&
          rig.machine.cpu().fault().kind != dsp56300::FaultKind::none && rig.machine.io().failure(),
      "Unknown raw input must fail in the actual original input-register read with context");
  if (!rig.attempted) { throw blofeld::Fault("Missing original failed instruction preview"); }
  require(
      rig.attempted->pc == fault_pc &&
          rig.machine.clock().elapsed_ticks() == rig.attempted->completion_ticks &&
          rig.attempted->completion_ticks == rig.attempted->start_ticks + unit_ticks &&
          rig.machine.clock().native_cycle_units() == rig.attempted->units + 1 &&
          rig.machine.cpu().progress() == rig.attempted->progress &&
          rig.machine.bus().read({.space = x, .address = field}) == field_seed &&
          rig.machine.bus().read({.space = x, .address = producer}) == ring_base + 2 &&
          rig.machine.bus().read({.space = x, .address = consumer}) == ring_base + 2,
      "Failed DI sample retains its actual reference cycle without retiring or completing MIDI");
  static_cast<void>(rejects([&] { static_cast<void>(rig.schedule->checkpoint()); }));
  const auto diagnostic = rig.machine.checkpoint();
  blofeld::OwnDspMachine fresh(clocks);
  fresh.restore(diagnostic);
  require(native_state(fresh) == native_state(rig.machine) &&
              fresh.cpu().fault() == rig.machine.cpu().fault() && fresh.io().failure() &&
              fresh.clock().elapsed_ticks() == rig.machine.clock().elapsed_ticks(),
          "Diagnostic restore preserves the real failed-cycle prefix, not a healthy continuation");
  trace << "EXPECTED_FAILURE " << error << '\n';
  restore();
  rig.finish();
  require(rig.signature() == first_suffix, "Coordinated native/schedule/input recovery differs");
  trace << "RESULT scope=checkpoint snapshots=1 identical-suffixes=2 changed-suffixes=1 "
           "unknown-DI-failures=1 recovered-suffixes=1 callbacks=4\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Expected original firmware ZIP, private trace path and directed scope");
    const auto firmware = blofeld::Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    const std::string scope(arguments[3]);
    if (scope == "bytes") {
      test_bytes(firmware, trace);
    } else if (scope == "pitch") {
      test_pitch(firmware, trace);
    } else if (scope == "checkpoint") {
      test_checkpoint(firmware, trace);
    } else {
      throw blofeld::Fault("Unknown directed original clocked-wire scope: " + scope);
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
