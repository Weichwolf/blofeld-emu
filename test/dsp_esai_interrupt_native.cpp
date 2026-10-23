// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp_norm_fixture.hpp"
#include "esai_interrupt_fixture.hpp"

#include <array>
#include <optional>
#include <string>
#include <variant>

namespace blofeld::test::esai_irq {
namespace {
constexpr unsigned handler = 0x300, ordinary_sr = 0xc00000, ipl_mask = 0x300;
constexpr unsigned absolute_jsr = 0x0bf080, secondary_priority = 0x400;

struct Scenario {
  bool second;
  unsigned mode;
  bool reference;
};

unsigned install(DspMachine &machine, const Scenario input) {
  DspProgram foreground;
  foreground.immediate("sr", ordinary_sr);
  const auto resume = foreground.pc();
  foreground.emit("nop");
  foreground.install(machine.bus());
  machine.cpu().step();
  require(machine.cpu().state().pc == resume && machine.cpu().state().sr == ordinary_sr,
          "Own native foreground sets actual SR before any transmit request exists");
  DspProgram body;
  const auto space = input.second ? std::string("y") : std::string("x");
  if (input.mode == 1) {
    body.emit("movep " + space + ":<<" + number(base(input.second) + status_offset) + ",x0");
  }
  body.emit("movep #>" + number(next_first) + ',' + space + ":<<" + number(base(input.second)));
  body.emit("movep #>" + number(next_second) + ',' + space + ":<<" +
            number(base(input.second) + 5));
  body.emit("rti");
  DspBus storage;
  body.install(storage);
  for (unsigned n = 0; n < body.pc() - boot; ++n) {
    machine.bus().write({.address = handler + n, .value = storage.inspect_program(boot + n)});
  }
  const auto vector = expected(input.second, input.mode, {.even = true, .error = input.mode == 1});
  if (!vector.has_value()) { throw Fault("Own native interrupt scenario lacks a vector"); }
  machine.bus().write({.address = vector.value(), .value = absolute_jsr});
  machine.bus().write({.address = vector.value() + 1, .value = handler});
  setup(machine.bus(), input.second, input.mode);
  machine.bus().write({.space = DspSpace::x,
                       .address = priority_register,
                       .value = input.second ? secondary_priority : 1U});
  require(word(machine.bus(), input.second) == std::array{first_data, second_data},
          "Declared priming words actually leave both serial lanes");
  if (input.mode == 1) { static_cast<void>(word(machine.bus(), input.second)); }
  return resume;
}

void reference_entry(DspMachine &machine, const DspInterruptRequest expected_request) {
  const auto before = machine.cpu().debug_state();
  auto progress = machine.begin_reference_interrupt(DspReferenceExecution::access_boundaries);
  unsigned vectors = 0;
  unsigned acknowledgements = 0;
  while (!progress.retired) {
    if (!progress.pending) { throw Fault("Native ESAI entry lost its reference access ticket"); }
    if (const auto *access = std::get_if<DspNativeAccess>(&progress.pending->effect)) {
      require(access->kind == DspNativeAccessKind::vector && access->target.space == DspSpace::p &&
                  access->target.address == expected_request.vector + vectors && vectors < 2,
              "Actual reference entry fetches both own vector words in order");
      ++vectors;
    } else if (const auto *ack = std::get_if<DspInterruptAcknowledge>(&progress.pending->effect)) {
      require(ack->request == expected_request && vectors == 2 && acknowledgements == 0,
              "Reference entry admits only the fetched ESAI vector/priority");
      ++acknowledgements;
    } else {
      throw Fault("Unexpected effect in own native ESAI vector entry");
    }
    require(machine.cpu().debug_state() == before &&
                machine.bus().interrupt_request(0) == expected_request,
            "Pending vector observations cannot retire CPU state or clear a level source");
    progress = machine.advance_reference_access(progress.pending->ticket);
  }
  require(vectors == 2 && acknowledgements == 1,
          "Reference native JSR retires after two vector reads and one acknowledgement");
}

void service(DspMachine &machine, const Scenario input, const unsigned resume) {
  const auto before = machine.cpu().debug_state();
  const auto vector = expected(input.second, input.mode, {.even = true, .error = input.mode == 1});
  if (!vector.has_value()) { throw Fault("Native ESAI test has no expected request"); }
  const DspInterruptRequest request{.vector = vector.value(), .level = 0};
  require(machine.bus().interrupt_request(0) == request,
          "Real peripheral state supplies the vector");
  if (input.reference) {
    reference_entry(machine, request);
  } else {
    machine.cpu().step();
  }
  const auto entry = machine.cpu().state();
  require(entry.pc == handler && entry.sp == 1 && entry.stack_pc == resume &&
              entry.stack_sr == ordinary_sr && (entry.sr & ipl_mask) == 0x100 &&
              machine.bus().interrupt_request(0) == request,
          "Native JSR stacks the foreground; vector acknowledgement retains the level IRQ");
  if (input.mode == 1) {
    machine.cpu().step();
    require((machine.cpu().state().x & status_underrun) != 0,
            "Native error ISR reads actual SAISR before refilling enabled TX registers");
  }
  machine.cpu().step();
  require(machine.bus().interrupt_request(0) == request,
          "First native MOVEP cannot complete a two-lane transmitter refill");
  machine.cpu().step();
  require(!machine.bus().interrupt_request(0), "Second native MOVEP services the actual level IRQ");
  machine.cpu().step();
  require(machine.cpu().state().pc == resume && machine.cpu().state().sp == 0 &&
              machine.cpu().state().sr == ordinary_sr &&
              machine.cpu().debug_state().native_instructions - before.native_instructions ==
                  (input.mode == 1 ? 5U : 4U),
          "Native ISR/RTI preserve instruction count and restore foreground PC/SR/SP");
  require(word(machine.bus(), input.second) == std::array{next_first, next_second},
          "Actual ISR writes become the next independent 24-bit serial words");
}

void native_case(const Scenario input) {
  DspMachine machine;
  const auto resume = install(machine, input);
  const auto saved = machine.checkpoint();
  std::optional<DspDebugState> result;
  for (unsigned replay = 0; replay < 2; ++replay) {
    machine.restore(saved);
    service(machine, input, resume);
    const auto actual = machine.cpu().debug_state();
    if (result.has_value()) {
      require(actual == result.value(),
              "Trusted restore repeats every native ISR register/counter");
    } else {
      result = actual;
    }
  }
}

void native_unknown_clock(const bool second) {
  DspMachine machine;
  DspProgram program;
  program.immediate("sr", ordinary_sr);
  const auto resume = program.pc();
  program.emit("nop");
  program.install(machine.bus());
  machine.cpu().step();
  setup(machine.bus(), second, 4, false);
  machine.bus().write({.space = DspSpace::x,
                       .address = priority_register,
                       .value = second ? secondary_priority : 1U});
  const auto saved = machine.checkpoint();
  const auto error = rejects([&] { machine.cpu().step(); });
  require(error.contains(second ? "Y:0xffff93" : "X:0xffffb3") &&
              error.contains("pc=" + hex(resume, 6)) && error.contains("omr=") &&
              rejects([&] {
                static_cast<void>(machine.checkpoint());
              }).contains("completed reference step"),
          "Unknown eligible ESAI clock rejects actual native entry with PC/OMR and status address");
  const auto failed = machine.cpu().debug_state();
  static_cast<void>(rejects([&] { machine.cpu().step(); }));
  require(machine.cpu().debug_state() == failed, "A rejected native owner cannot consume a vector");
  machine.restore(saved);
  machine.bus().observe_esai_transmit(instance(second), {.clock = true, .frame_sync = false});
  machine.cpu().step();
  require(
      machine.cpu().state().pc == resume + 1 && !machine.bus().interrupt_request(0),
      "Trusted recovery plus a real clock baseline permits only the unrequested foreground NOP");
}
} // namespace

void native_checks() {
  for (const bool second : {false, true}) {
    native_unknown_clock(second);
    for (const unsigned mode : {1U, 2U, 4U}) {
      for (const bool reference : {false, true}) {
        native_case({.second = second, .mode = mode, .reference = reference});
      }
    }
  }
}
} // namespace blofeld::test::esai_irq
