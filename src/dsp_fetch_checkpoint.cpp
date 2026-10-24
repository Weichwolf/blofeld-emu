// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_fetch_run.hpp"
#include "blofeld/dsp_machine.hpp"
#include "dsp_fetch_run.hpp"

#include <memory>
#include <utility>

namespace blofeld {
DspFetchCheckpoint::DspFetchCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspFetchCheckpoint::~DspFetchCheckpoint() = default;

DspFetchCheckpoint DspFetchRun::checkpoint(const DspMachine &machine) const {
  check_owner(machine);
  impl_->state.verify(machine);
  return DspFetchCheckpoint(
      std::make_shared<const DspFetchCheckpoint::Storage>(DspFetchCheckpoint::Storage{
          .machine = machine.checkpoint(), .clock = impl_->state, .owner = impl_->owner}));
}

void DspFetchRun::restore(const DspFetchCheckpoint &checkpoint, DspMachine &machine) {
  if (impl_->active || impl_->owner != &machine || !checkpoint.storage_ ||
      checkpoint.storage_->owner != &machine ||
      checkpoint.storage_->clock.inputs != impl_->state.inputs) {
    throw Fault("DSP fetch checkpoint requires its matching inactive owner and clock contract");
  }
  auto state = checkpoint.storage_->clock;
  machine.restore(checkpoint.storage_->machine);
  state.bind(machine);
  impl_->state = std::move(state);
  impl_->faulted = false;
}
}
