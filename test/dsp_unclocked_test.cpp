// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <exception>
#include <iostream>
#include <tuple>

namespace {
using namespace blofeld;
using test::esai::rejects;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned data_address = 0x100, missing_address = 0x101;
constexpr unsigned data = 0x123456, sentinel = 0x654321;
constexpr unsigned immediate_x0 = 0x44f400, immediate_x1 = 0x45f400;
constexpr unsigned store_x0 = 0x447000, load_x1 = 0x45f000, immediate_sr = 0x05f439;
constexpr unsigned reset_ticks = 90, completed_reset_ticks = 2 * reset_ticks;
constexpr unsigned priority_register = 0xffffff, word_bits = 24;
constexpr OwnDspResetInputs reset_pins{
    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2};

auto observe(const OwnDspMachine &machine) {
  return std::tuple{machine.cpu().registers(),
                    machine.cpu().progress(),
                    machine.cpu().fault(),
                    machine.clock().elapsed_ticks(),
                    machine.interrupt_pending()};
}

void refused(OwnDspMachine &machine) {
  const auto before = observe(machine);
  const auto error = rejects([&] { static_cast<void>(machine.run_unclocked()); });
  require(error.contains("pc=") && observe(machine) == before,
          "Synchronous execution is refused contextually before CPU or clock mutation");
}

void step(OwnDspMachine &machine) {
  const auto count = machine.cpu().progress().instructions;
  const auto ticks = machine.clock().elapsed_ticks();
  const auto result = machine.run_unclocked();
  require(result.fault.kind == dsp56300::FaultKind::none && result.dispatches == 1 &&
              machine.cpu().progress().instructions == count + 1 &&
              machine.clock().elapsed_ticks() == ticks,
          "A real synchronous instruction retires without invented clock periods");
}

native::NativeProgram memory_program() {
  native::NativeProgram program;
  // FM full-word MOVE #imm,X0/X1, MOVE X0,X:abs, MOVE X:abs,X1, NOP.
  for (const auto word : {immediate_x0,
                          data,
                          immediate_x1,
                          sentinel,
                          store_x0,
                          data_address,
                          load_x1,
                          missing_address,
                          0U}) {
    program.word(word);
  }
  return program;
}

void at_store(OwnDspMachine &machine) {
  native::reset_native(machine, memory_program());
  machine.bus().write({.space = DspSpace::x, .address = data_address, .value = sentinel});
  step(machine);
  step(machine);
  require(machine.cpu().registers().pc == native::boot + 4 &&
              machine.cpu().progress().instructions == 2,
          "Authored setup reaches its actual absolute store");
}

struct StoreObserver {
  OwnDspMachine &machine;
  unsigned calls = 0;
  bool fail = false;
  bool reenter = false;

  static void completed(void *context, const DspStore &store, const unsigned pc) {
    auto &self = *static_cast<StoreObserver *>(context);
    ++self.calls;
    require(store.space == DspSpace::x && store.address == data_address && store.value == data &&
                pc == native::boot + 4 &&
                self.machine.bus().read({.space = store.space, .address = store.address}) == data &&
                self.machine.cpu().progress().instructions == 2 &&
                self.machine.clock().elapsed_ticks() == completed_reset_ticks,
            "Observer sees the actual committed store before retirement with unchanged time");
    if (self.reenter) {
      const auto before = observe(self.machine);
      refused(self.machine);
      static_cast<void>(rejects([&] { static_cast<void>(self.machine.checkpoint()); }));
      static_cast<void>(rejects([&] { self.machine.begin_hardware_reset(reset_pins); }));
      require(observe(self.machine) == before,
              "Observer cannot reenter execution, capture or reset its active owner");
    }
    if (self.fail) { throw Fault("Authored observer refusal after the committed store"); }
  }

  [[nodiscard]] DspIoWriteObserver borrowed() { return {.context = this, .completed = completed}; }
};

void missing_read_and_restore() {
  OwnDspMachine machine(native::native_clocks);
  at_store(machine);
  StoreObserver observer{.machine = machine, .reenter = true};
  const auto healthy = machine.checkpoint();
  const auto stored = machine.run_unclocked(observer.borrowed());
  require(stored.fault.kind == dsp56300::FaultKind::none && stored.dispatches == 1 &&
              observer.calls == 1 && machine.cpu().progress().instructions == 3,
          "Observed store retires exactly once through the real bus");
  const auto failed = machine.run_unclocked();
  const auto &failure = machine.io().failure();
  require(failed.fault.kind == dsp56300::FaultKind::memory_read &&
              failed.fault.space == dsp56300::Space::x && failed.fault.address == missing_address &&
              failed.fault.pc == native::boot + 6 &&
              machine.cpu().registers().x >> word_bits == sentinel &&
              machine.cpu().progress().instructions == 3 && failure &&
              failure->operation == OwnDspIo::Operation::read && failure->pc == native::boot + 6,
          "Missing input preserves the destination and reports the actual board read and PC");
  const auto faulted = machine.checkpoint();
  const auto state = observe(machine);
  OwnDspMachine fresh(native::native_clocks);
  for (auto *const target : {&machine, &fresh}) {
    target->restore(faulted);
    const auto retry = target->run_unclocked();
    const auto &restored_failure = target->io().failure();
    require(observe(*target) == state && retry.fault == failed.fault && restored_failure &&
                restored_failure->cause &&
                target->bus().read({.space = DspSpace::x, .address = data_address}) == data,
            "Same/fresh diagnostic restore preserves the fault and committed bus without replay");
  }
  machine.restore(healthy);
  step(machine);
  require(observer.calls == 1,
          "A returned observer is detached and is not captured in a healthy checkpoint");
}

void observer_refusal() {
  OwnDspMachine machine(native::native_clocks);
  at_store(machine);
  const auto healthy = machine.checkpoint();
  StoreObserver observer{.machine = machine, .fail = true};
  const auto failed = machine.run_unclocked(observer.borrowed());
  const auto &failure = machine.io().failure();
  require(failed.fault.kind == dsp56300::FaultKind::memory_write &&
              failed.fault.pc == native::boot + 4 && failed.fault.address == data_address &&
              machine.cpu().progress().instructions == 2 && observer.calls == 1 &&
              machine.bus().read({.space = DspSpace::x, .address = data_address}) == data &&
              failure && failure->cause,
          "Observer exception quarantines CPU execution after retaining the actual physical write");
  const auto failed_state = observe(machine);
  const auto diagnostic = machine.checkpoint();
  machine.restore(diagnostic);
  static_cast<void>(machine.run_unclocked());
  require(observe(machine) == failed_state && observer.calls == 1,
          "A faulted callback cannot be repeated by restore or another run");
  machine.restore(healthy);
  step(machine);
  require(observer.calls == 1 && !machine.io().failure(),
          "Restoring healthy CPU and bus permits execution without retaining an expired callback");
}

void reset_and_execution_owners() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram program;
  for (unsigned index = 0; index < 4; ++index) { program.word(0); }
  program.install(machine.bus());
  refused(machine);
  machine.begin_hardware_reset(reset_pins);
  refused(machine);
  require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks, "Actual reset hold");
  machine.release_hardware_reset();
  refused(machine);
  require(machine.advance_hardware_reset_ticks(reset_ticks) == reset_ticks,
          "Actual reset recovery");
  {
    const OwnDspFetch fetch(machine);
    refused(machine);
  }
  {
    const OwnDspTimeline timeline(machine, OwnDspAutomaticTiming::one_effect_per_cycle, 0);
    refused(machine);
  }
  auto pending = machine.begin_fetch(dsp56300::InstructionPublication::retirement);
  require(pending.pending.has_value(), "Actual staged NOP has an outstanding effect");
  refused(machine);
  for (unsigned count = 0; count < 8 && pending.pending && !pending.retired; ++count) {
    pending = machine.advance(pending.pending->ticket);
  }
  require(pending.retired && machine.cpu().progress().instructions == 1 &&
              machine.clock().elapsed_ticks() == completed_reset_ticks,
          "Rejected synchronous entry preserves the original staged continuation");
  step(machine);
}

void vector_acknowledgement_owner() {
  OwnDspMachine machine(native::native_clocks);
  native::NativeProgram program;
  program.word(immediate_sr); // FM MOVE #imm,SR; allow the independently configured IRQA level.
  program.word(0);
  program.word(0);
  native::reset_native(machine, program);
  step(machine);
  constexpr unsigned vector = 0x10;
  machine.bus().write({.address = vector, .value = 0});
  machine.bus().write({.address = vector + 1, .value = 0});
  machine.bus().observe_irqa(true);
  machine.bus().write({.space = DspSpace::x, .address = priority_register, .value = 6});
  machine.bus().observe_irqa(false);
  require(machine.admit_interrupt({.address = native::boot + 2, .opcode = 0}) &&
              machine.interrupt_pending() && machine.bus().interrupt_request(0),
          "An actual unmasked IRQA edge owns the unacknowledged vector pair");
  refused(machine);
  require(machine.bus().interrupt_request(0).has_value(),
          "Rejected synchronous execution cannot acknowledge the peripheral request");
  native::step_native(machine);
  native::step_native(machine);
  require(!machine.interrupt_pending() && !machine.bus().interrupt_request(0) &&
              machine.cpu().registers().pc == native::boot + 2,
          "The real two-slot staged vector completes acknowledgement and resumes foreground");
  step(machine);
}
}

int main() {
  try {
    missing_read_and_restore();
    observer_refusal();
    reset_and_execution_owners();
    vector_acknowledgement_owner();
    std::cout
        << "Synchronous DSP: actual stores/read faults, diagnostic restore, borrowed observer "
           "lifetime, reentry, reset/fetch/timeline/staged/vector ownership passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
