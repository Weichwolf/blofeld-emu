// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/mcu_machine.hpp"

#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_cop.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "mcu/mcu_sci.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
class ResetAdvance {
public:
  explicit ResetAdvance(bool &active) : active_(active) { active_ = true; }

  ~ResetAdvance() { active_ = false; }

  ResetAdvance(const ResetAdvance &) = delete;
  ResetAdvance &operator=(const ResetAdvance &) = delete;

private:
  bool &active_;
};

struct RunState {
  std::optional<McuRunContract> contract;
  std::optional<McuRunClock> clock;
  std::uint8_t timer_inputs = 0;
  std::uint64_t idle_cycles = 0;
  bool wait_faulted = false;
  std::optional<McuResetSequence> reset = std::nullopt;
  ExecutionContext reset_context{};
  bool reset_faulted = false;
};

RunState initial_run(const std::optional<McuRunContract> contract) {
  if (!contract) { return {}; }
  if (contract->memory_timing != McuRunMemory::unstretched_internal &&
      contract->memory_timing != McuRunMemory::internal_flash_reads) {
    throw Fault("MCU run requires an explicit internal-memory timing contract");
  }
  return {.contract = contract, .clock = McuRunClock(contract->clock)};
}

}

struct McuMachine::State {
  State(std::unique_ptr<McuBus> bus_state,
        const std::filesystem::path &specification,
        RunState run_state)
      : bus(std::move(bus_state)), run(run_state), cpu(*bus, specification) {
    bus->configure_flash_timing(run.contract ? run.contract->flash_timing : FlashTimingInputs{});
    bus->configure_ram_program(run.contract ? run.contract->ram_program : std::nullopt);
    if (run.clock) { cycle_origin = run.clock->bus_cycles() - run.idle_cycles; }
    refresh_memory();
    cpu.set_clock_observer([this](const auto cycles, const auto context, const bool boundary) {
      synchronize(cycles, context);
      if (boundary) { refresh_memory(); }
    });
  }

  void refresh_memory() {
    bus->qualify_ram_program(cpu.state().pc);
    const auto revision = bus->code_revision();
    if (memory_revision == revision) { return; }
    const auto windows = bus->memory_windows();
    std::vector<cpu12::Ram> grants;
    grants.reserve(windows.size());
    for (const auto &window : windows) {
      grants.push_back({.first = window.first,
                        .bytes = window.bytes,
                        .known = window.known,
                        .writable = window.writable,
                        .identity = bus->native_memory_identity()});
    }
    cpu.bind_memory(grants.empty() ? cpu12::Memory::external() : cpu12::Memory::mapped(grants));
    memory_revision = revision;
  }

  void notify(const ExecutionContext context,
              const unsigned completed,
              const std::uint64_t before,
              const bool waiting) {
    if (observer && run.clock) {
      observer({.context = context,
                .duration_ticks = run.clock->elapsed_ticks() - before,
                .cycles = completed,
                .waiting = waiting});
    }
  }

  void synchronize(const std::uint64_t cycles, const ExecutionContext context) {
    if (cycles < accounted_cycles) { throw Fault("MCU nominal execution time moved backwards"); }
    if (!run.clock) {
      accounted_cycles = cycles;
      return;
    }
    while (accounted_cycles < cycles) {
      if (run.timer_inputs != 0) { bus->sample_timer_inputs(run.timer_inputs); }
      const auto before = run.clock->elapsed_ticks();
      const auto count = static_cast<unsigned>(
          std::min<std::uint64_t>(cycles - accounted_cycles, std::numeric_limits<unsigned>::max()));
      unsigned completed = 0;
      try {
        completed = run.clock->advance(*bus, context, count);
      } catch (...) {
        notify(context, 0, before, false);
        throw;
      }
      accounted_cycles += completed;
      notify(context, completed, before, false);
    }
  }

  void check_inactive() const {
    if (cpu.executing() || bus->notifying_ || reset_advancing) {
      throw Fault("MCU configuration requires an inactive execution owner");
    }
    if (!run.clock) { throw Fault("MCU machine has no device clock"); }
  }

  void check_clock_boundary() const {
    if (run.clock &&
        (run.clock->faulted() || run.wait_faulted || run.reset_faulted ||
         accounted_cycles != cpu.cycles() ||
         run.clock->bus_cycles() != cycle_origin + run.idle_cycles + accounted_cycles)) {
      throw Fault("MCU run requires a coordinated healthy native boundary");
    }
  }

  std::unique_ptr<McuBus> bus;
  RunState run;
  std::function<void(McuSpan)> observer;
  std::function<void(McuResetPeriod)> reset_period_observer;
  bool reset_advancing = false;
  Hc12 cpu;
  std::uint64_t cycle_origin = 0;
  std::uint64_t accounted_cycles = 0;
  std::optional<std::uint64_t> memory_revision;
};

McuMachine::McuMachine(const McuFlashImage &image, const std::filesystem::path &specification)
    : McuMachine(image, specification, std::nullopt) {}

McuMachine::McuMachine(const McuFlashImage &image,
                       const std::filesystem::path &specification,
                       const std::optional<McuRunContract> run)
    : specification_(specification),
      state_(std::make_unique<State>(
          std::make_unique<McuBus>(image), specification, initial_run(run))) {
  state_->cpu.reset();
}

McuMachine::McuMachine(const McuFlashImage &image,
                       const std::filesystem::path &specification,
                       const McuRunContract run,
                       const McuPowerOnInputs power_on)
    : specification_(specification),
      state_(std::make_unique<State>(
          std::make_unique<McuBus>(image), specification, initial_run(run))) {
  state_->run.reset.emplace(McuResetSource::power_on, power_on.synchronization_cycles);
  auto &clock = state_->run.clock;
  if (!clock) { throw Fault("MCU power-on requires its reference clock"); }
  clock->begin_power_on_reset();
  state_->bus->reset_asserted_ = true;
}

McuMachine::~McuMachine() = default;

McuBus &McuMachine::bus() {
  return *state_->bus;
}

Hc12 &McuMachine::cpu() {
  return state_->cpu;
}

const McuBus &McuMachine::bus() const {
  return *state_->bus;
}

const Hc12 &McuMachine::cpu() const {
  return state_->cpu;
}

bool McuMachine::clocked() const {
  return state_->run.clock.has_value();
}

const McuRunClock &McuMachine::run_clock() const {
  const auto &clock = state_->run.clock;
  if (!clock) { throw Fault("MCU machine has no reference run clock"); }
  return *clock;
}

bool McuMachine::timing_faulted() const {
  const auto &clock = state_->run.clock;
  return (clock && clock->faulted()) || state_->run.wait_faulted || state_->run.reset_faulted;
}

void McuMachine::validate_run_boundary() const {
  if (system_reset_active()) { throw Fault("MCU native boundary is held in system reset"); }
  state_->check_inactive();
  state_->check_clock_boundary();
}

void McuMachine::validate_reset_boundary() const {
  if (!system_reset_active()) {
    throw Fault("MCU reset boundary requires an asserted system reset");
  }
  state_->check_inactive();
  state_->check_clock_boundary();
  static_cast<void>(run_clock().next_reset_edge_ticks());
}

void McuMachine::set_span_observer(std::function<void(McuSpan)> observer) {
  state_->check_inactive();
  state_->observer = std::move(observer);
}

CpuStep McuMachine::run_until(const std::uint64_t ticks) {
  validate_run_boundary();
  const auto &clock = run_clock();
  if (ticks <= clock.elapsed_ticks()) { return {.instruction_executed = false}; }
  state_->refresh_memory();
  const auto remaining = ticks - clock.elapsed_ticks();
  const auto period = clock.next_cycle_ticks(bus());
  auto count = (remaining / period) + (remaining % period != 0 ? 1U : 0U);
  if (const auto event = clock.cycles_until_event(bus(), cpu().waiting())) {
    count = std::min(count, *event);
  }
  if (count > std::numeric_limits<std::uint64_t>::max() - cpu().cycles()) {
    throw Fault("MCU event horizon exhausted nominal time");
  }
  if (cpu().waiting() && !cpu().pending_interrupt()) {
    static_cast<void>(advance_wait(static_cast<unsigned>(
        std::min<std::uint64_t>(count, std::numeric_limits<unsigned>::max()))));
    return {.instruction_executed = false};
  }
  return cpu().run_until(cpu().cycles() + count);
}

void McuMachine::set_reset_period_observer(std::function<void(McuResetPeriod)> observer) {
  state_->check_inactive();
  state_->reset_period_observer = std::move(observer);
}

void McuMachine::set_sci_receive_reference(const std::optional<SciReceiveReference> reference) {
  state_->check_inactive();
  state_->check_clock_boundary();
  state_->bus->sci_.set_receive_reference(reference);
}

std::optional<SciReceiveReference> McuMachine::sci_receive_reference() const {
  return state_->bus->sci_.receive_reference();
}

std::uint64_t McuMachine::idle_bus_cycles() const {
  return state_->run.idle_cycles;
}

unsigned McuMachine::advance_wait(const unsigned maximum_bus_cycles) {
  state_->check_inactive();
  state_->check_clock_boundary();
  auto &cpu = state_->cpu;
  auto &clock = state_->run.clock;
  if (!clock) { throw Fault("MCU wait requires a device clock"); }
  const auto registers = cpu.state();
  const ExecutionContext context{.pc = registers.pc, .ccr = registers.ccr};
  cpu.begin_wait_clock();
  unsigned completed = 0;
  try {
    state_->bus->begin_instruction(context);
    static_cast<void>(state_->bus->wait_clock_gates());
    while (completed < maximum_bus_cycles && !cpu.pending_interrupt()) {
      const auto gates = state_->bus->wait_clock_gates();
      if (gates.timer && state_->run.timer_inputs != 0) {
        state_->bus->sample_timer_inputs(state_->run.timer_inputs);
        if (cpu.pending_interrupt()) { break; }
      }
      const auto before = clock->elapsed_ticks();
      const auto before_cycles = clock->bus_cycles();
      unsigned count = 0;
      try {
        count = clock->advance(*state_->bus, context, maximum_bus_cycles - completed, true);
      } catch (...) {
        const auto prefix = static_cast<unsigned>(clock->bus_cycles() - before_cycles);
        state_->run.idle_cycles += prefix;
        state_->notify(context, prefix, before, true);
        throw;
      }
      completed += count;
      state_->run.idle_cycles += count;
      state_->notify(context, count, before, true);
    }
    cpu.end_wait_clock(true);
    return completed;
  } catch (const std::exception &error) {
    cpu.end_wait_clock(false);
    state_->run.wait_faulted = true;
    throw Fault("MCU wait advance pc=" + hex(context.pc) + " ccr=" + hex(context.ccr, 2) +
                " completed-idle-cycles=" + std::to_string(state_->run.idle_cycles) + ": " +
                error.what());
  } catch (...) {
    cpu.end_wait_clock(false);
    state_->run.wait_faulted = true;
    throw;
  }
}

void McuMachine::set_timer_inputs(const std::uint8_t mask) {
  if (system_reset_active()) { throw Fault("Cannot prime timer inputs during asserted MCU reset"); }
  state_->check_inactive();
  state_->bus->sample_timer_inputs(mask);
  state_->run.timer_inputs = mask;
}

void McuMachine::attach_flash_store(std::shared_ptr<VirtualDeviceState> store) {
  state_->check_inactive();
  state_->bus->attach_flash_store(std::move(store));
}

bool McuMachine::system_reset_active() const {
  const auto &reset = state_->run.reset;
  return reset && reset->active();
}

std::optional<McuResetState> McuMachine::system_reset_state() const {
  const auto &reset = state_->run.reset;
  return reset ? std::optional(reset->state()) : std::nullopt;
}

void McuMachine::begin_system_reset(const McuResetSource source,
                                    const unsigned synchronization_cycles) {
  state_->check_inactive();
  if (system_reset_active()) { throw Fault("MCU system reset is already active"); }
  if (source == McuResetSource::power_on) {
    throw Fault("MCU power-on requires a fresh machine with explicit power-on inputs");
  }
  const McuResetSequence sequence(source, synchronization_cycles);
  if (source == McuResetSource::cop && state_->bus->cop_state().reset == McuCopReset::none) {
    throw Fault("MCU COP system reset requires an actual pending watchdog request");
  }
  auto run = state_->run;
  auto &clock = run.clock;
  if (!clock) { throw Fault("MCU reset requires a reference run clock"); }
  clock->begin_reset_epoch();
  run.reset = sequence;
  run.reset_context = state_->bus->context_;
  run.wait_faulted = false;
  run.reset_faulted = false;
  run.timer_inputs = 0;
  // A hardware reset preserves the single external store owner. Public clones
  // reject that ownership instead.
  auto bus = std::unique_ptr<McuBus>(new McuBus(*state_->bus, McuBus::CloneState{}));
  bus->reset_modules();
  bus->reset_asserted_ = true;
  auto replacement = std::make_unique<State>(std::move(bus), specification_, run);
  state_.swap(replacement);
}

unsigned McuMachine::advance_system_reset(const unsigned maximum_sysclk_edges,
                                          const McuResetPad pad) {
  state_->check_inactive();
  if (!system_reset_active() || timing_faulted()) {
    throw Fault("MCU reset advance requires a healthy asserted system reset");
  }
  auto &run = state_->run;
  auto &reset = run.reset;
  auto &clock = run.clock;
  if (!reset || !clock) { throw Fault("MCU reset requires its sequence and reference clock"); }
  const ResetAdvance advancing(state_->reset_advancing);
  auto &sequence = *reset;
  auto &reset_clock = *clock;
  unsigned completed = 0;
  try {
    while (completed < maximum_sysclk_edges) {
      auto next = sequence;
      next.edge(pad); // Qualify before committing this edge or its elapsed time.
      if (state_->reset_period_observer) {
        state_->reset_period_observer({.context = run.reset_context,
                                       .duration_ticks = reset_clock.next_reset_edge_ticks(),
                                       .native_epoch = reset_clock.native_epoch()});
      }
      reset_clock.advance_reset_edge();
      sequence = next;
      ++completed;
      if (next.state().phase == McuResetPhase::ready) {
        reset_clock.finish_reset();
        state_->bus->reset_asserted_ = false;
        state_->cpu.reset(next.state().selected == McuResetSource::cop ? Hc12ResetSource::cop
                                                                       : Hc12ResetSource::external);
        sequence.complete();
        break;
      }
    }
  } catch (const std::exception &error) {
    run.reset_faulted = true;
    throw Fault("MCU system reset pc=" + hex(run.reset_context.pc) +
                " ccr=" + hex(run.reset_context.ccr, 2) + " completed-SYSCLK-edges=" +
                std::to_string(sequence.state().edges) + ": " + error.what());
  } catch (...) {
    run.reset_faulted = true;
    throw;
  }
  return completed;
}

}
