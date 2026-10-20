// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/mcu_machine.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_memory.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "blofeld/mcu_sci.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <ios>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld {
namespace {
struct RunState {
  std::optional<McuRunContract> contract;
  std::optional<McuRunClock> clock;
  std::uint8_t timer_inputs = 0;
  std::uint64_t idle_cycles = 0;
  bool wait_faulted = false;
};

RunState initial_run(const std::optional<McuRunContract> contract) {
  if (!contract) { return {}; }
  if (contract->memory_timing != McuRunMemory::unstretched_internal &&
      contract->memory_timing != McuRunMemory::internal_flash_reads) {
    throw Fault("MCU run requires an explicit internal-memory timing contract");
  }
  return {.contract = contract, .clock = McuRunClock(contract->clock)};
}

bool internal_ram(const McuSpace space) {
  return space == McuSpace::smram || space == McuSpace::qram;
}

std::vector<std::uint8_t> read_specification(const std::filesystem::path &path) {
  constexpr std::uintmax_t maximum_size = 16777216;
  const auto size = std::filesystem::file_size(path);
  if (size == 0 || size > maximum_size) { throw Fault("Invalid MCU checkpoint backend size"); }
  std::ifstream stream(path, std::ios::binary);
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!stream || stream.peek() != std::ifstream::traits_type::eof()) {
    throw Fault("Cannot read complete MCU checkpoint backend: " + path.string());
  }
  return bytes;
}
}

struct McuCheckpoint::Storage {
  Storage(std::unique_ptr<McuBus> bus_state,
          const Hc12Checkpoint &cpu_state,
          const std::vector<std::uint8_t> &backend,
          RunState run_state)
      : bus(std::move(bus_state)), cpu(cpu_state), specification(backend), run(run_state) {}

  const std::unique_ptr<const McuBus> bus;
  const Hc12Checkpoint cpu;
  const std::vector<std::uint8_t> specification;
  const RunState run;
};

struct McuMachine::State {
  State(std::unique_ptr<McuBus> bus_state,
        const std::filesystem::path &specification,
        RunState run_state)
      : bus(std::move(bus_state)),
        run(run_state),
        timed(run.clock ? std::make_unique<Hc12CycleBus>(
                              *bus,
                              [this](const auto cycle) { advance(cycle); },
                              [this](const auto transfer) { return read_timing(transfer); })
                        : nullptr),
        cpu(timed ? static_cast<Bus &>(*timed) : *bus, specification) {
    if (const auto &clock = run.clock; timed && clock) {
      if (run.idle_cycles > clock->bus_cycles()) { throw Fault("Invalid MCU idle clock count"); }
      timed->restore_boundary(clock->bus_cycles() - run.idle_cycles, bus->context_);
    }
  }

  [[nodiscard]] Hc12ReadTiming read_timing(const MemoryRead transfer) const {
    const auto &contract = run.contract;
    constexpr unsigned last_address = 0xffff;
    if (!contract || contract->memory_timing != McuRunMemory::internal_flash_reads ||
        transfer.size != 2 || (transfer.address & 1U) == 0 || transfer.address == last_address) {
      return Hc12ReadTiming::single_cycle;
    }
    const auto first = bus->memory_.locate(transfer.address);
    const auto last = bus->memory_.locate(static_cast<std::uint16_t>(transfer.address + 1U));
    if (first.space == McuSpace::flash && last.space == McuSpace::flash &&
        last.offset == first.offset + 1U) {
      return Hc12ReadTiming::byte_cycles;
    }
    return Hc12ReadTiming::single_cycle;
  }

  void advance(const Hc12BusCycle cycle) {
    if (run.wait_faulted) { throw Fault("MCU wait clock failure requires coordinated recovery"); }
    if (cycle.address) {
      const auto first = bus->memory_.locate(*cycle.address);
      bool qualified = first.space != McuSpace::unmapped && first.space != McuSpace::reserved;
      if (cycle.size == 2) {
        const auto last = bus->memory_.locate(static_cast<std::uint16_t>(*cycle.address + 1U));
        qualified = qualified && first.space == last.space && last.offset == first.offset + 1U &&
                    ((*cycle.address & 1U) == 0 || internal_ram(first.space));
      }
      if (!qualified) {
        throw Fault("Unqualified MCU reference " + std::string(1, cycle.kind) +
                    " address=" + hex(*cycle.address) + " size=" + std::to_string(cycle.size));
      }
    }
    if (observer) { observer(cycle); }
    if (run.timer_inputs != 0) { bus->sample_timer_inputs(run.timer_inputs); }
    auto &clock = run.clock;
    if (!clock) { throw Fault("MCU reference cycle has no run clock"); }
    clock->advance_run_cycle(*bus, cycle.context);
  }

  [[nodiscard]] bool advance_wait_cycle(const ExecutionContext context) {
    bus->begin_instruction(context);
    if (wait_observer) { wait_observer({.context = context}); }
    if (cpu.pending_interrupt()) { return false; }
    const auto gates = bus->wait_clock_gates();
    if (gates.timer && run.timer_inputs != 0) { bus->sample_timer_inputs(run.timer_inputs); }
    if (cpu.pending_interrupt()) { return false; }
    auto &clock = run.clock;
    if (!clock) { throw Fault("MCU wait cycle has no run clock"); }
    if (wait_period_observer) { wait_period_observer({.context = context}); }
    clock->advance_wait_cycle(*bus, context);
    ++run.idle_cycles;
    return true;
  }

  void check_inactive() const {
    if (cpu.executing() || bus->notifying_) {
      throw Fault("MCU run configuration requires an inactive CPU and bus observer");
    }
    if (!timed) { throw Fault("MCU machine has no reference run clock"); }
  }

  void check_clock_boundary() const {
    const auto &clock = run.clock;
    if (timed &&
        (!clock || clock->faulted() || run.wait_faulted || run.idle_cycles > clock->bus_cycles() ||
         timed->checkpoint_cycles() != clock->bus_cycles() - run.idle_cycles)) {
      throw Fault("MCU run checkpoint requires coordinated healthy cycle counters");
    }
  }

  std::unique_ptr<McuBus> bus;
  RunState run;
  std::unique_ptr<Hc12CycleBus> timed;
  std::function<void(Hc12BusCycle)> observer;
  std::function<void(McuWaitCycle)> wait_observer;
  std::function<void(McuWaitPeriod)> wait_period_observer;
  Hc12 cpu;
};

McuCheckpoint::McuCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

McuCheckpoint::~McuCheckpoint() = default;

McuMachine::McuMachine(const McuFlashImage &image, const std::filesystem::path &specification)
    : McuMachine(image, specification, std::nullopt) {}

McuMachine::McuMachine(const McuFlashImage &image,
                       const std::filesystem::path &specification,
                       const std::optional<McuRunContract> run)
    : specification_(specification),
      specification_bytes_(read_specification(specification)),
      state_(std::make_unique<State>(
          std::make_unique<McuBus>(image), specification, initial_run(run))) {
  check_specification();
  state_->cpu.reset();
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
  return state_->timed &&
         (!clock || state_->timed->faulted() || clock->faulted() || state_->run.wait_faulted);
}

void McuMachine::validate_run_boundary() const {
  state_->check_inactive();
  state_->check_clock_boundary();
}

void McuMachine::set_cycle_observer(std::function<void(Hc12BusCycle)> observer) {
  state_->check_inactive();
  state_->observer = std::move(observer);
}

void McuMachine::set_wait_observer(std::function<void(McuWaitCycle)> observer) {
  state_->check_inactive();
  state_->wait_observer = std::move(observer);
}

void McuMachine::set_wait_period_observer(std::function<void(McuWaitPeriod)> observer) {
  state_->check_inactive();
  state_->wait_period_observer = std::move(observer);
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
  const auto registers = cpu.state();
  const ExecutionContext context{.pc = registers.pc, .ccr = registers.ccr};
  cpu.begin_wait_clock();
  unsigned completed = 0;
  try {
    state_->bus->begin_instruction(context);
    static_cast<void>(state_->bus->wait_clock_gates());
    while (completed < maximum_bus_cycles && !cpu.pending_interrupt()) {
      if (!state_->advance_wait_cycle(context)) { break; }
      ++completed;
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
  state_->check_inactive();
  state_->bus->sample_timer_inputs(mask);
  state_->run.timer_inputs = mask;
}

void McuMachine::check_specification() const {
  if (read_specification(specification_) != specification_bytes_) {
    throw Fault("MCU checkpoint backend changed after machine construction");
  }
}

McuCheckpoint McuMachine::checkpoint() const {
  const auto cpu_state = state_->cpu.checkpoint();
  state_->check_clock_boundary();
  check_specification();
  return McuCheckpoint(std::make_shared<const McuCheckpoint::Storage>(
      state_->bus->clone(), cpu_state, specification_bytes_, state_->run));
}

void McuMachine::restore(const McuCheckpoint &checkpoint) {
  if (state_->cpu.executing() || state_->bus->notifying_) {
    throw Fault("Cannot restore during active MCU execution or bus observation");
  }
  if (!checkpoint.storage_) { throw Fault("Cannot restore an empty MCU checkpoint"); }
  if (checkpoint.storage_->specification != specification_bytes_) {
    throw Fault("MCU checkpoint has an incompatible instruction backend");
  }
  if (checkpoint.storage_->run.contract != state_->run.contract) {
    throw Fault("MCU checkpoint has an incompatible reference run contract");
  }
  check_specification();
  auto replacement = std::make_unique<State>(
      checkpoint.storage_->bus->clone(), specification_, checkpoint.storage_->run);
  replacement->cpu.restore_checkpoint(checkpoint.storage_->cpu);
  check_specification();
  state_.swap(replacement);
}

}
