// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_machine.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"

#include <memory>
#include <utility>

namespace blofeld {

struct DspCheckpoint::Storage {
  Storage(DspBus bus_state, DspCoreCheckpoint cpu_state)
      : bus(std::move(bus_state)), cpu(std::move(cpu_state)) {}

  const DspBus bus;
  const DspCoreCheckpoint cpu;
};

struct DspMachine::State {
  State() : cpu(bus) {}

  DspBus bus;
  DspCore cpu;
};

DspCheckpoint::DspCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspCheckpoint::~DspCheckpoint() = default;

DspMachine::DspMachine() : state_(std::make_unique<State>()) {}

DspMachine::~DspMachine() = default;

DspBus &DspMachine::bus() {
  return state_->bus;
}

DspCore &DspMachine::cpu() {
  return state_->cpu;
}

const DspBus &DspMachine::bus() const {
  return state_->bus;
}

const DspCore &DspMachine::cpu() const {
  return state_->cpu;
}

DspCheckpoint DspMachine::checkpoint() const {
  const auto cpu_state = state_->cpu.checkpoint();
  return DspCheckpoint(std::make_shared<const DspCheckpoint::Storage>(state_->bus, cpu_state));
}

void DspMachine::restore(const DspCheckpoint &checkpoint) {
  if (state_->cpu.executing()) { throw Fault("Cannot restore during active DSP execution"); }
  if (!checkpoint.storage_) { throw Fault("Cannot restore an empty DSP checkpoint"); }
  auto replacement = std::make_unique<State>();
  replacement->bus = checkpoint.storage_->bus;
  replacement->cpu.restore_checkpoint(checkpoint.storage_->cpu);
  state_.swap(replacement);
}

}
