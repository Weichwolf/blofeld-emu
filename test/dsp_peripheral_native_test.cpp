// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace blofeld::test::peripheral_native {
namespace {
enum class Scenario : std::uint8_t { transmit_status, clock_control, control_forward };
enum class Reader : std::uint8_t { word, set, clear };
constexpr unsigned hckr = 0xffff90;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned baseline_status = 0x4082a9;
constexpr unsigned settled_status = 0x4002a9;
constexpr unsigned first_word = 0x123456;
constexpr unsigned second_word = 0x654321;
constexpr unsigned control_word = 0xa9;
constexpr unsigned target = 0xff0020;
constexpr std::uint64_t write_epoch = 102;
constexpr auto execution = DspReferenceExecution::access_and_retirement_boundaries;

struct Window {
  Scenario scenario = Scenario::transmit_status;
  Reader reader = Reader::word;
  bool baseline = false;
  bool immediate = false;
  unsigned gap = 0;
};

unsigned port(const Scenario scenario) {
  return scenario == Scenario::clock_control ? hckr : hcsr;
}

unsigned source(const Scenario scenario) {
  switch (scenario) {
    case Scenario::transmit_status: return second_word;
    case Scenario::clock_control: return 3;
    case Scenario::control_forward: return control_word;
    default: throw Fault("Invalid own native source scenario");
  }
}

std::string read_instruction(const Window window) {
  const auto operand = "x:<<" + number(port(window.scenario));
  if (window.reader == Reader::word) { return "movep " + operand + ",x0"; }
  const auto bit = window.scenario == Scenario::clock_control ? 1U : 15U;
  return std::string(window.reader == Reader::set ? "jset #" : "jclr #") + std::to_string(bit) +
         ',' + operand + ',' + number(target);
}

void epoch(DspMachine &machine, const std::uint64_t selected) {
  const auto previous = machine.bus().last_dma_cycle();
  const auto first = previous ? *previous + 1 : 0;
  for (auto cycle = first; cycle <= selected; ++cycle) {
    const auto transfer = machine.advance_dma({.cycle = cycle,
                                               .known_requests = 15,
                                               .timing = DspDmaTiming::reference_sampled_cycles,
                                               .grant = DspDmaGrant::available});
    require(!transfer, "Own explicit native read phase unexpectedly executed DMA");
  }
  require(machine.bus().last_dma_cycle() == selected, "Own DMA epoch went backwards");
}

DspReferencePending pending(DspMachine &machine) {
  const auto value = machine.reference_pending();
  if (!value) { throw Fault("Own native read phase lost its actual pending effect"); }
  return *value;
}

void program_words(DspMachine &machine) {
  for (unsigned count = 0; count < 3; ++count) {
    const auto value = pending(machine);
    const auto *access = std::get_if<DspNativeAccess>(&value.effect);
    if (access == nullptr || (access->kind != DspNativeAccessKind::opcode &&
                              access->kind != DspNativeAccessKind::extension)) {
      return;
    }
    static_cast<void>(machine.advance_reference_access(value.ticket));
  }
  throw Fault("Own native read exceeded its independent opcode/extension word bound");
}

void fetch(DspMachine &machine, const unsigned words) {
  static_cast<void>(machine.start_reference_fetch(pending(machine).ticket));
  for (unsigned index = 0; index < words; ++index) {
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::address));
    static_cast<void>(machine.advance_reference_fetch(DspProgramFetchStage::read));
  }
}

struct Fixture {
  explicit Fixture(const Window window) {
    program.immediate("sr", masked_status);
    program.immediate("r1", source(window.scenario));
    const auto address = window.scenario == Scenario::transmit_status ? htx : port(window.scenario);
    const auto operand = window.immediate ? "#>" + number(source(window.scenario)) : "r1";
    program.emit(window.baseline ? "nop" : "movep " + operand + ",x:<<" + number(address));
    reader = program.pc();
    program.emit(read_instruction(window));
    program.install(machine.bus());
    checkpoint::run(machine, 2);
    before = machine.cpu().debug_state();
    machine.bus().shi().observe_spi({.mosi = false});
    if (window.scenario != Scenario::clock_control) {
      machine.bus().write({.space = DspSpace::x, .address = hcsr, .value = control_word});
      machine.bus().write({.space = DspSpace::x, .address = htx, .value = first_word});
      machine.bus().shi().observe_spi({.selected = true, .mosi = false});
    }
    static_cast<void>(machine.begin_reference_instruction(execution));
    const auto producer_words = !window.baseline && window.immediate ? 2U : 1U;
    fetch(machine, producer_words + (window.reader == Reader::word ? 1U : 2U));
    program_words(machine);
  }

  void finish_producer() {
    const auto value = pending(machine);
    if (std::holds_alternative<DspNativeAccess>(value.effect)) {
      static_cast<void>(machine.advance_reference_access(value.ticket));
    }
    const auto held = pending(machine);
    require(std::holds_alternative<DspInstructionRetire>(held.effect),
            "Own native producer did not hold its actual retirement");
    const auto completed = machine.advance_reference_access(held.ticket);
    require(completed.retired && machine.cpu().state().pc == reader,
            "Actual native producer did not retire into the independent reader address");
    static_cast<void>(machine.begin_reference_instruction(execution));
  }

  DspMachine machine;
  DspProgram program;
  DspDebugState before;
  unsigned reader = 0;
};

unsigned expected_word(const Scenario scenario, const bool old) {
  if (scenario == Scenario::clock_control) { return old ? 1U : 3U; }
  if (scenario == Scenario::control_forward) { return baseline_status; }
  return old ? baseline_status : settled_status;
}

bool expected_bit(const Window window) {
  const auto cost = window.immediate ? 2U : 1U;
  const bool old = window.baseline || cost + window.gap <= 2;
  if (window.scenario == Scenario::clock_control) { return !window.baseline; }
  if (window.scenario == Scenario::control_forward) { return window.baseline || !old; }
  return old;
}

void complete_reader(Fixture &fixture, const Window window, const DspPeripheralReadTicket &read) {
  auto &machine = fixture.machine;
  program_words(machine);
  const auto before_data = machine.cpu().debug_state();
  const auto value = pending(machine);
  const auto response = machine.advance_reference_peripheral_read(read, value.ticket);
  if (!response.completed || !response.pending || response.retired ||
      machine.cpu().debug_state() != before_data) {
    throw Fault("Owned actual read must hold the native CPU result without early publication");
  }
  const auto *retirement = std::get_if<DspInstructionRetire>(&response.pending->effect);
  if (retirement == nullptr) { throw Fault("Owned read did not reach real native retirement"); }
  auto expected = fixture.before;
  ++expected.native_instructions;
  expected.native_cycles += (!window.baseline && window.immediate) ? 2U : 1U;
  if (window.reader == Reader::word) {
    const bool old = window.baseline || (window.immediate ? 2U : 1U) + window.gap <= 2;
    const auto word = expected_word(window.scenario, old);
    expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | word;
    expected.cpu.pc = fixture.reader + 1;
    ++expected.native_cycles;
  } else {
    const bool taken = expected_bit(window) == (window.reader == Reader::set);
    expected.cpu.pc = taken ? target : fixture.reader + 2;
    expected.native_cycles += 4;
    require(retirement->redirected == taken, "Actual branch predicate differs from NXP window");
  }
  ++expected.native_instructions;
  require(retirement->next_pc == expected.cpu.pc && retirement->next_omr == reset_omr,
          "Actual held native PC differs from independent source/condition expectations");
  const auto completed = machine.advance_reference_access(response.pending->ticket);
  require(completed.retired && machine.cpu().debug_state() == expected,
          "Real native reader retirement differs in register/CCR/stack/counters/PC state");
}

void one_window(const Window window) {
  Fixture fixture(window);
  const auto cost = window.immediate ? 2U : 1U;
  const auto sample_epoch = write_epoch + cost + window.gap - 2;
  std::optional<DspPeripheralReadTicket> read;
  if (sample_epoch <= write_epoch) {
    epoch(fixture.machine, sample_epoch);
    read = fixture.machine.capture_reference_peripheral_read(
        {.address = fixture.reader, .epoch = sample_epoch});
  }
  epoch(fixture.machine, write_epoch);
  fixture.finish_producer();
  if (!read) {
    epoch(fixture.machine, sample_epoch);
    read = fixture.machine.capture_reference_peripheral_read(
        {.address = fixture.reader, .epoch = sample_epoch});
  }
  epoch(fixture.machine, sample_epoch + 2);
  complete_reader(fixture, window, *read);
  const auto actual =
      fixture.machine.bus().read({.space = DspSpace::x, .address = port(window.scenario)});
  require(actual == expected_word(window.scenario, window.baseline),
          "Admitting native read data altered the actual settled peripheral state");
}

void visibility() {
  constexpr std::array scenarios{
      Scenario::transmit_status, Scenario::clock_control, Scenario::control_forward};
  constexpr std::array readers{Reader::word, Reader::set, Reader::clear};
  for (const auto scenario : scenarios) {
    for (const auto reader : readers) {
      one_window({.scenario = scenario, .reader = reader, .baseline = true});
      for (const bool immediate : {false, true}) {
        for (unsigned gap = 0; gap < 3; ++gap) {
          one_window({.scenario = scenario, .reader = reader, .immediate = immediate, .gap = gap});
        }
      }
    }
  }
}

void ownership() {
  constexpr Window window{.scenario = Scenario::clock_control, .baseline = true};
  Fixture fixture(window);
  epoch(fixture.machine, write_epoch - 1);
  const auto before_capture = fixture.machine.checkpoint();
  const auto discarded = fixture.machine.capture_reference_peripheral_read(
      {.address = fixture.reader, .epoch = write_epoch - 1});
  fixture.machine.restore(before_capture);
  const auto read = fixture.machine.capture_reference_peripheral_read(
      {.address = fixture.reader, .epoch = write_epoch - 1});
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.rebind_reference_peripheral_read(discarded)); },
      "Discarded-future sample aliased a new capture at the same slot and serial"));
  Fixture foreign(window);
  epoch(foreign.machine, write_epoch - 1);
  const auto foreign_read = foreign.machine.capture_reference_peripheral_read(
      {.address = foreign.reader, .epoch = write_epoch - 1});
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.rebind_reference_peripheral_read(foreign_read)); },
      "A foreign owner's sample aliased matching slot/serial metadata"));
  epoch(fixture.machine, write_epoch);
  fixture.finish_producer();
  epoch(fixture.machine, write_epoch + 1);
  program_words(fixture.machine);
  const auto old_pending = pending(fixture.machine);
  const auto saved = fixture.machine.checkpoint();
  fixture.machine.restore(saved);
  const auto fresh = pending(fixture.machine);
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(fixture.machine.advance_reference_peripheral_read(read, fresh.ticket));
      },
      "Restore accepted a stale sample owner without trusted rebind"));
  const auto rebound = fixture.machine.rebind_reference_peripheral_read(read);
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(
            fixture.machine.advance_reference_peripheral_read(rebound, old_pending.ticket));
      },
      "Restore accepted a stale native data ticket"));
  const auto before = fixture.machine.cpu().debug_state();
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.advance_reference_access(fresh.ticket)); },
      "An ordinary admission performed the already captured peripheral read again"));
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.advance_reference_bus_cycle({})); },
      "Unqualified early-read bus-cycle resources were silently accepted"));
  require(fixture.machine.cpu().debug_state() == before &&
              fixture.machine.bus().last_dma_cycle() == write_epoch + 1,
          "Refused owner/admission paths advanced native CPU or admitted clock");
  complete_reader(fixture, window, rebound);
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.rebind_reference_peripheral_read(read)); },
      "A consumed sample could be rebound and admitted again"));
  fixture.machine.cpu().reset_spi_boot();
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(fixture.machine.rebind_reference_peripheral_read(rebound)); },
      "Hardware reset retained a grant for the old actual peripheral sample"));
}

void spi_word(DspBus &bus, const std::uint32_t value) {
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

void retire(DspMachine &machine) {
  const auto held = pending(machine);
  require(std::holds_alternative<DspInstructionRetire>(held.effect),
          "Actual native data admission lost its held retirement");
  require(machine.advance_reference_access(held.ticket).retired,
          "Actual held native reader did not retire");
}

void fifo_replay() {
  constexpr unsigned hrx = 0xffff94;
  constexpr unsigned receive_not_empty = 0x20000;
  DspMachine machine;
  DspProgram program;
  program.immediate("sr", masked_status);
  const auto first_pc = program.pc();
  program.emit("movep x:<<$ffff94,x0");
  const auto second_pc = program.pc();
  program.emit("movep x:<<$ffff94,y0");
  program.install(machine.bus());
  checkpoint::run(machine, 1);
  const auto before = machine.cpu().debug_state();
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = hcsr, .value = control_word});
  machine.bus().write({.space = DspSpace::x, .address = htx, .value = first_word});
  machine.bus().write({.space = DspSpace::x, .address = htx, .value = second_word});
  machine.bus().shi().observe_spi({.selected = true, .mosi = false});
  spi_word(machine.bus(), first_word);
  spi_word(machine.bus(), second_word);
  static_cast<void>(machine.begin_reference_instruction(execution));
  fetch(machine, 2);
  epoch(machine, 0);
  const auto first = machine.capture_reference_peripheral_read({.address = first_pc});
  program_words(machine);
  epoch(machine, 2);
  const auto at_data = machine.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(at_data);
    const auto read = machine.rebind_reference_peripheral_read(first);
    static_cast<void>(machine.advance_reference_peripheral_read(read, pending(machine).ticket));
    retire(machine);
    require((machine.cpu().state().x & word_mask) == first_word &&
                (machine.bus().read({.space = DspSpace::x, .address = hcsr}) & receive_not_empty) !=
                    0,
            "Restored native read duplicated the first FIFO pop or lost its actual data");
  }
  static_cast<void>(machine.begin_reference_instruction(execution));
  epoch(machine, 3);
  const auto second = machine.capture_reference_peripheral_read({.address = second_pc, .epoch = 3});
  program_words(machine);
  epoch(machine, 5);
  static_cast<void>(machine.advance_reference_peripheral_read(second, pending(machine).ticket));
  const auto at_retirement = machine.checkpoint();
  auto expected = before;
  expected.cpu.pc = second_pc + 1;
  expected.cpu.x = (expected.cpu.x & ~std::uint64_t{word_mask}) | first_word;
  expected.cpu.y = (expected.cpu.y & ~std::uint64_t{word_mask}) | second_word;
  expected.native_instructions += 2;
  expected.native_cycles += 2;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(at_retirement);
    static_cast<void>(checkpoint::rejects(
        [&] { static_cast<void>(machine.rebind_reference_peripheral_read(second)); },
        "A post-data checkpoint retained an already consumed FIFO response grant"));
    retire(machine);
    require(machine.cpu().debug_state() == expected &&
                (machine.bus().read({.space = DspSpace::x, .address = hcsr}) & receive_not_empty) ==
                    0,
            "Restored native E2 changed registers/flags/counters or replayed FIFO I/O");
    static_cast<void>(checkpoint::rejects(
        [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = hrx})); },
        "Exactly two SPI words did not leave the FIFO empty after two native reads"));
  }
}

void phase_refusals() {
  Fixture fixture({.scenario = Scenario::clock_control, .baseline = true});
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(
            fixture.machine.capture_reference_peripheral_read({.address = fixture.reader}));
      },
      "A peripheral sample accepted an unadmitted clock epoch"));
  DspMachine machine;
  DspProgram program;
  program.word(reference::long_jump);
  program.word(target);
  const auto skipped = program.pc();
  program.emit("movep x:<<$ffff94,x0");
  program.install(machine.bus());
  static_cast<void>(machine.begin_reference_instruction(execution));
  fetch(machine, 3);
  program_words(machine);
  epoch(machine, 0);
  const auto error = checkpoint::rejects(
      [&] { static_cast<void>(machine.capture_reference_peripheral_read({.address = skipped})); },
      "An unresolved producer jump allowed speculative target-path FIFO I/O");
  require(error.find("control flow") != std::string::npos,
          "Unresolved control flow reached the destructive empty FIFO read");
}

void selective_bit(const bool level, const bool set) {
  constexpr unsigned port_c_data = 0xffffbd;
  DspMachine machine;
  DspProgram program;
  program.immediate("sr", masked_status);
  const auto reader = program.pc();
  program.emit(std::string(set ? "jset" : "jclr") + " #0,x:<<$ffffbd," + number(target));
  program.install(machine.bus());
  checkpoint::run(machine, 1);
  auto expected = machine.cpu().debug_state();
  machine.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = 1, .mask = 1});
  machine.bus().gpio(DspPort::c).set_drive({.mask = 1, .levels = level ? 1U : 0U});
  static_cast<void>(machine.begin_reference_instruction(execution));
  fetch(machine, 2);
  epoch(machine, 0);
  const auto read = machine.capture_reference_peripheral_read({.address = reader});
  program_words(machine);
  const auto data = pending(machine);
  const auto *access = std::get_if<DspNativeAccess>(&data.effect);
  require(access != nullptr && access->kind == DspNativeAccessKind::bit,
          "Native selective GPIO branch did not produce a real bit callback");
  epoch(machine, 2);
  static_cast<void>(machine.advance_reference_peripheral_read(read, data.ticket));
  retire(machine);
  expected.cpu.pc = level == set ? target : reader + 2;
  ++expected.native_instructions;
  expected.native_cycles += 4;
  require(machine.cpu().debug_state() == expected,
          "Actual native bit callback changed flags/registers or selected the wrong branch");
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = port_c_data}));
      },
      "Native bit admission invented data for unobserved GPIO pins"));
}

void bits() {
  for (const bool level : {false, true}) {
    for (const bool set : {false, true}) { selective_bit(level, set); }
  }
}
}
}

int main(const int argc, const char *const argv[]) {
  try {
    blofeld::test::require(argc == 2, "Select native visibility, ownership, fifo, phases or bits");
    const std::string_view mode(argv[1]);
    if (mode == "visibility") {
      blofeld::test::peripheral_native::visibility();
    } else if (mode == "ownership") {
      blofeld::test::peripheral_native::ownership();
    } else if (mode == "fifo") {
      blofeld::test::peripheral_native::fifo_replay();
    } else if (mode == "phases") {
      blofeld::test::peripheral_native::phase_refusals();
    } else if (mode == "bits") {
      blofeld::test::peripheral_native::bits();
    } else {
      throw blofeld::Fault("Unknown native peripheral read group");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
