// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_read_encodings.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace blofeld::test::peripheral_read {
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
constexpr std::uint64_t write_epoch = 102;

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
    default: throw Fault("Invalid own visibility scenario");
  }
}

std::string read_instruction(const Window window) {
  const auto operand = "x:<<" + number(port(window.scenario));
  if (window.reader == Reader::word) { return "movep " + operand + ",x0"; }
  const auto bit = window.scenario == Scenario::clock_control ? 1U : 15U;
  return std::string(window.reader == Reader::set ? "jset #" : "jclr #") + std::to_string(bit) +
         ',' + operand + ",$ff0020";
}

struct Fixture {
  explicit Fixture(const Window window) {
    program.immediate("sr", masked_status);
    program.immediate("r1", source(window.scenario));
    producer = program.pc();
    const auto target = window.scenario == Scenario::transmit_status ? htx : port(window.scenario);
    const auto operand = window.immediate ? "#>" + number(source(window.scenario)) : "r1";
    program.emit(window.baseline ? "nop" : "movep " + operand + ",x:<<" + number(target));
    reader = program.pc();
    program.emit(read_instruction(window));
    program.install(machine.bus());
    checkpoint::run(machine, 2);
    machine.bus().shi().observe_spi({.mosi = false});
    if (window.scenario != Scenario::clock_control) {
      machine.bus().write({.space = DspSpace::x, .address = hcsr, .value = control_word});
      machine.bus().shi().observe_spi({.selected = true, .mosi = false});
      machine.bus().write({.space = DspSpace::x, .address = htx, .value = first_word});
    }
    const auto baseline =
        machine.bus().read({.space = DspSpace::x, .address = port(window.scenario)});
    require(baseline == (window.scenario == Scenario::clock_control ? 1U : baseline_status),
            "Own source baseline differs from independently observed window");
    fetch.start(reader, reset_omr);
    const auto words = window.reader == Reader::word ? 1U : 2U;
    for (unsigned index = 0; index < words; ++index) {
      fetch.issue();
      fetch.capture(machine.bus());
    }
  }

  [[nodiscard]] std::vector<DspNativeAccess> write() {
    std::vector<DspNativeAccess> events;
    machine.cpu().step({.observe = [&](const DspNativeAccess &event) { events.push_back(event); }});
    require(machine.cpu().state().pc == reader, "Actual native producer did not reach reader");
    return events;
  }

  DspMachine machine;
  DspProgram program;
  DspProgramFetch fetch;
  unsigned producer = 0;
  unsigned reader = 0;
};

struct Consumer {
  DspNativeAccess opcode;
  std::optional<DspNativeAccess> extension = std::nullopt;
  DspNativeAccess pending;
};

DspNativeAccess native_access(const DspReferenceEffect &effect) {
  const auto *access = std::get_if<DspNativeAccess>(&effect);
  if (access == nullptr) { throw Fault("Native peripheral boundary has a different effect kind"); }
  return *access;
}

Consumer pending_read(DspMachine &machine) {
  auto progress =
      machine.begin_reference_instruction(DspReferenceExecution::access_and_retirement_boundaries);
  if (!progress.pending) { throw Fault("Native reader lost its opcode boundary"); }
  progress = machine.advance_reference_access(progress.pending->ticket);
  if (!progress.completed) { throw Fault("Native reader lost its actual opcode receipt"); }
  Consumer result{.opcode = native_access(*progress.completed), .pending = {}};
  if (!progress.pending) { throw Fault("Native reader lost its actual data/extension boundary"); }
  if (native_access(progress.pending->effect).kind == DspNativeAccessKind::extension) {
    progress = machine.advance_reference_access(progress.pending->ticket);
    if (!progress.completed) { throw Fault("Native reader lost its extension receipt"); }
    result.extension = native_access(*progress.completed);
  }
  if (!progress.pending) { throw Fault("Native reader did not stop before actual port access"); }
  result.pending = native_access(progress.pending->effect);
  return result;
}

void forward(DspPeripheralRead &read,
             const std::vector<DspNativeAccess> &events,
             const bool expect_forward) {
  std::optional<DspNativeAccess> opcode;
  bool forwarded = false;
  for (const auto &event : events) {
    if (event.stage != DspNativeAccessStage::after) { continue; }
    if (event.kind == DspNativeAccessKind::opcode) { opcode = event; }
    if (event.kind != DspNativeAccessKind::write) { continue; }
    if (!opcode || !opcode->value) { throw Fault("Actual native producer has no captured opcode"); }
    forwarded = read.forward(
        {.address = opcode->target.address, .value = *opcode->value}, event, write_epoch);
  }
  require(forwarded == expect_forward, "Forwarding did not follow actual producer/address/age");
}

unsigned expected_word(const Scenario scenario, const bool old) {
  if (scenario == Scenario::clock_control) { return old ? 1U : 3U; }
  if (scenario == Scenario::control_forward) { return baseline_status; }
  return old ? baseline_status : settled_status;
}

void check_value(const Window window, const std::uint32_t value) {
  const auto cost = window.immediate ? 2U : 1U;
  const bool old = window.baseline || cost + window.gap <= 2;
  if (window.reader == Reader::word) {
    const auto expected = expected_word(window.scenario, old);
    require(value == expected, "Full-word read differs from independent NXP continuous window");
    return;
  }
  const auto bit = window.scenario == Scenario::clock_control ? 1U : 15U;
  bool expected = old;
  if (window.scenario == Scenario::clock_control) { expected = !window.baseline; }
  if (window.scenario == Scenario::control_forward) { expected = window.baseline || !old; }
  require(((value >> bit) & 1U) == static_cast<unsigned>(expected),
          "Bit-branch source differs from independent NXP continuous window");
}

void one_window(const Window window) {
  Fixture fixture(window);
  const auto cost = window.immediate ? 2U : 1U;
  const auto sample_epoch = write_epoch + cost + window.gap - 2;
  std::optional<DspPeripheralRead> read;
  if (sample_epoch <= write_epoch) {
    read = DspPeripheralRead::capture(
        fixture.machine.bus(), fixture.fetch, {.address = fixture.reader, .epoch = sample_epoch});
  }
  const auto events = fixture.write();
  if (!read) {
    read = DspPeripheralRead::capture(
        fixture.machine.bus(), fixture.fetch, {.address = fixture.reader, .epoch = sample_epoch});
  }
  const bool expect_forward = !window.baseline && window.reader != Reader::word &&
                              window.scenario != Scenario::transmit_status &&
                              sample_epoch <= write_epoch;
  forward(*read, events, expect_forward);
  const auto consumer = pending_read(fixture.machine);
  const auto value =
      read->consume(consumer.opcode, consumer.extension, consumer.pending, sample_epoch + 2);
  check_value(window, value);
  const auto final =
      fixture.machine.bus().read({.space = DspSpace::x, .address = port(window.scenario)});
  const auto expected = expected_word(window.scenario, window.baseline);
  require(final == expected, "Latched CPU response changed actual peripheral state");
  const auto error = checkpoint::rejects(
      [&] {
        static_cast<void>(
            read->consume(consumer.opcode, consumer.extension, consumer.pending, sample_epoch + 2));
      },
      "An already consumed peripheral response was accepted twice");
  require(error.find("pc=") != std::string::npos && error.find("read X:") != std::string::npos,
          "Unavailable response lost its operation/address/PC context");
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

void encoding_checks() {
  for (const auto &encoding : encodings) {
    const auto form = decode_peripheral_read(encoding.opcode);
    require(form && form->target.space == encoding.space &&
                form->target.address == encoding.address && form->bit == encoding.bit &&
                form->set == encoding.set && form->words == encoding.words,
            "Port operand differs from independent NXP encoding " + number(encoding.opcode));
  }
  constexpr std::array refused{0x000000U, 0x018098U, 0x01809fU, 0x04d190U, 0x08d03cU};
  for (const auto opcode : refused) {
    require(!decode_peripheral_read(opcode), "Unqualified opcode accepted as a port read");
  }
}

void identity_checks() {
  Fixture fixture({.reader = Reader::set, .baseline = true});
  auto read = DspPeripheralRead::capture(
      fixture.machine.bus(), fixture.fetch, {.address = fixture.reader, .epoch = write_epoch});
  static_cast<void>(fixture.write());
  const auto actual = pending_read(fixture.machine);
  enum class Mismatch : std::uint8_t { word, extension, address, pc, omr, ordinal, kind, phase };
  constexpr std::array mismatches{Mismatch::word,
                                  Mismatch::extension,
                                  Mismatch::address,
                                  Mismatch::pc,
                                  Mismatch::omr,
                                  Mismatch::ordinal,
                                  Mismatch::kind,
                                  Mismatch::phase};
  for (const auto mismatch : mismatches) {
    auto candidate = read;
    auto consumer = actual;
    if (!consumer.extension) { throw Fault("Own bit branch lost its actual extension"); }
    switch (mismatch) {
      case Mismatch::word: consumer.opcode.value = 0; break;
      case Mismatch::extension: consumer.extension->value = 0; break;
      case Mismatch::address: ++consumer.pending.target.address; break;
      case Mismatch::pc: ++consumer.pending.pc; break;
      case Mismatch::omr: consumer.pending.omr = 0; break;
      case Mismatch::ordinal: ++consumer.pending.ordinal; break;
      case Mismatch::kind: consumer.pending.kind = DspNativeAccessKind::write; break;
      case Mismatch::phase: consumer.pending.stage = DspNativeAccessStage::after; break;
      default: throw Fault("Invalid own identity mismatch case");
    }
    const auto error = checkpoint::rejects(
        [&] {
          static_cast<void>(candidate.consume(
              consumer.opcode, consumer.extension, consumer.pending, write_epoch + 2));
        },
        "An incorrect native read identity consumed a peripheral response");
    require(error.find("pc=") != std::string::npos && error.find("read X:") != std::string::npos,
            "Read-identity refusal lost its operation/address/PC context");
    require(candidate.consume(actual.opcode, actual.extension, actual.pending, write_epoch + 2) ==
                baseline_status,
            "An identity refusal changed the still-owned actual data sample");
  }
}

void capture_words(DspProgramFetch &fetch, DspBus &bus, const unsigned words) {
  fetch.start(boot, reset_omr);
  for (unsigned index = 0; index < words; ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
}

void unknown_port() {
  DspMachine machine;
  DspProgram program;
  program.emit("jset #0,x:<<$ffff80,$ff0020");
  program.install(machine.bus());
  DspProgramFetch fetch;
  capture_words(fetch, machine.bus(), 1);
  const auto missing = checkpoint::rejects(
      [&] {
        static_cast<void>(DspPeripheralRead::capture(
            machine.bus(), fetch, {.address = boot, .epoch = write_epoch}));
      },
      "A peripheral read proceeded without the real F2 extension");
  require(missing.find("awaits real F2 extension") != std::string::npos,
          "Missing extension reached the unknown port instead of stopping before the read");
  fetch.issue();
  fetch.capture(machine.bus());
  const auto unknown = checkpoint::rejects(
      [&] {
        static_cast<void>(DspPeripheralRead::capture(
            machine.bus(), fetch, {.address = boot, .epoch = write_epoch}));
      },
      "An unknown peripheral port returned a successful data sample");
  require(unknown.find("read X:0xffff80") != std::string::npos &&
              unknown.find("pc=0xff0000") != std::string::npos &&
              unknown.find("omr=0x000305") != std::string::npos,
          "Unknown early read lost address/operation/PC/OMR context");
}

void selective_bit() {
  DspMachine machine;
  DspProgram program;
  program.emit("jset #0,x:<<$ffffbd,$ff0020");
  program.install(machine.bus());
  machine.bus().gpio(DspPort::c).write({.reg = DspGpioRegister::control, .value = 1, .mask = 1});
  machine.bus().gpio(DspPort::c).set_drive({.mask = 1, .levels = 1});
  DspProgramFetch fetch;
  capture_words(fetch, machine.bus(), 2);
  auto read =
      DspPeripheralRead::capture(machine.bus(), fetch, {.address = boot, .epoch = write_epoch});
  const auto consumer = pending_read(machine);
  require(consumer.pending.kind == DspNativeAccessKind::bit,
          "Actual native GPIO bit test lost its selective peripheral callback");
  require(read.consume(consumer.opcode, consumer.extension, consumer.pending, write_epoch + 2) == 1,
          "Selective bit read did not observe the one known GPIO pin");
  constexpr unsigned port_c_data = 0xffffbd;
  static_cast<void>(checkpoint::rejects(
      [&] {
        static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = port_c_data}));
      },
      "A selective bit sample fabricated values for unobserved GPIO pins"));
}

void spi_word(DspBus &bus, const std::uint32_t value) {
  for (unsigned remaining = word_bits; remaining != 0; --remaining) {
    const bool bit = ((value >> (remaining - 1)) & 1U) != 0;
    bus.shi().observe_spi({.selected = true, .clock = true, .mosi = bit});
    bus.shi().observe_spi({.selected = true, .mosi = bit});
  }
}

void fifo_once() {
  DspMachine machine;
  DspProgram program;
  program.emit("movep x:<<$ffff94,x0");
  program.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = hcsr, .value = control_word});
  machine.bus().write({.space = DspSpace::x, .address = htx, .value = first_word});
  machine.bus().write({.space = DspSpace::x, .address = htx, .value = second_word});
  machine.bus().shi().observe_spi({.selected = true, .mosi = false});
  spi_word(machine.bus(), first_word);
  spi_word(machine.bus(), second_word);
  DspProgramFetch fetch;
  capture_words(fetch, machine.bus(), 1);
  auto read =
      DspPeripheralRead::capture(machine.bus(), fetch, {.address = boot, .epoch = write_epoch});
  const auto consumer = pending_read(machine);
  require(read.consume(consumer.opcode, consumer.extension, consumer.pending, write_epoch + 2) ==
              first_word,
          "Latched receive data differs from the actual first SPI word");
  constexpr unsigned hrx = 0xffff94;
  require(machine.bus().read({.space = DspSpace::x, .address = hrx}) == second_word,
          "Preparing or consuming the native response popped the FIFO a second time");
  static_cast<void>(checkpoint::rejects(
      [&] { static_cast<void>(machine.bus().read({.space = DspSpace::x, .address = hrx})); },
      "Receive FIFO did not retain the exact two SPI words"));
}

void boundaries() {
  identity_checks();
  unknown_port();
  selective_bit();
  fifo_once();
}
}
}

int main(const int argc, const char *const argv[]) {
  try {
    blofeld::test::require(argc == 2, "Select encodings, visibility or boundaries");
    const std::string_view mode(argv[1]);
    if (mode == "encodings") {
      blofeld::test::peripheral_read::encoding_checks();
    } else if (mode == "visibility") {
      blofeld::test::peripheral_read::visibility();
    } else if (mode == "boundaries") {
      blofeld::test::peripheral_read::boundaries();
    } else {
      throw blofeld::Fault("Unknown peripheral read component group");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
