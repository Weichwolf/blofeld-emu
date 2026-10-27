// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"

#include <memory>
#include <utility>

namespace blofeld {
DspFetchCheckpoint::DspFetchCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspFetchCheckpoint::~DspFetchCheckpoint() = default;

DspFetchRun::DspFetchRun(const DspFetchCheckpoint &checkpoint, DspMachine &target)
    : impl_(std::make_unique<Impl>()) {
  if (!checkpoint.storage_) { throw Fault("DSP fetch clone requires a trusted checkpoint"); }
  const auto &saved = *checkpoint.storage_;
  if (saved.lifetime.lock() && saved.owner == &target) {
    throw Fault("DSP fetch clone requires a distinct isolated target");
  }
  target.validate_fetch_clone_target();
  auto state = saved.clock;
  target.restore(saved.machine);
  state.bind(target);
  impl_->state = std::move(state);
  impl_->owner = &target;
  validate_boundary(target);
}

DspFetchCheckpoint DspFetchRun::checkpoint(const DspMachine &machine) const {
  check_owner(machine);
  impl_->state.verify(machine);
  return DspFetchCheckpoint(std::make_shared<const DspFetchCheckpoint::Storage>(
      DspFetchCheckpoint::Storage{.machine = machine.checkpoint(),
                                  .clock = impl_->state,
                                  .owner = impl_->owner,
                                  .lifetime = impl_->lifetime}));
}

void DspFetchRun::restore(const DspFetchCheckpoint &checkpoint, DspMachine &machine) {
  if (impl_->active || impl_->owner != &machine || !checkpoint.storage_ ||
      checkpoint.storage_->owner != &machine ||
      checkpoint.storage_->lifetime.lock() != impl_->lifetime ||
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
