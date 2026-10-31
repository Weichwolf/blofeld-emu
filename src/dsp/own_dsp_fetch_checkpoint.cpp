// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace blofeld {
struct OwnDspFetchCheckpoint::Storage {
  OwnDspCheckpoint machine;
  DspProgramFetch fetch;
  std::optional<DspFetchedInstruction> admitted;
  std::optional<DspFetchedInstruction> repeated;
  std::uint64_t instructions;
  std::optional<std::uint32_t> interrupt_vector;
  bool in_flight;
  bool native_fetch;
  bool native_latch;
};

OwnDspFetchCheckpoint::OwnDspFetchCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

OwnDspFetchCheckpoint::~OwnDspFetchCheckpoint() = default;

const OwnDspFetchCheckpoint::Storage &
OwnDspFetch::saved_state(const OwnDspFetchCheckpoint &checkpoint) {
  if (!checkpoint.storage_) { throw std::invalid_argument("Empty own DSP fetch checkpoint"); }
  return *checkpoint.storage_;
}

OwnDspFetchCheckpoint OwnDspFetch::checkpoint() const {
  check_owner();
  if (machine_.cpu_.fault().kind != dsp56300::FaultKind::none) {
    throw Fault("Own DSP fetch checkpoint requires a healthy native instruction");
  }
  return OwnDspFetchCheckpoint(std::make_shared<const OwnDspFetchCheckpoint::Storage>(
      OwnDspFetchCheckpoint::Storage{.machine = machine_.checkpoint(),
                                     .fetch = fetch_,
                                     .admitted = admitted_,
                                     .repeated = repeated_,
                                     .instructions = instructions_,
                                     .interrupt_vector = interrupt_vector_,
                                     .in_flight = in_flight_,
                                     .native_fetch = native_fetch_,
                                     .native_latch = native_latch_}));
}

OwnDspFetch::OwnDspFetch(OwnDspMachine &machine, const OwnDspFetchCheckpoint &checkpoint)
    : machine_(machine),
      identity_(std::make_shared<const bool>(false)),
      fetch_(saved_state(checkpoint).fetch),
      admitted_(saved_state(checkpoint).admitted),
      repeated_(saved_state(checkpoint).repeated),
      instructions_(saved_state(checkpoint).instructions),
      interrupt_vector_(saved_state(checkpoint).interrupt_vector),
      in_flight_(saved_state(checkpoint).in_flight),
      native_fetch_(saved_state(checkpoint).native_fetch),
      native_latch_(saved_state(checkpoint).native_latch) {
  machine_.restore(saved_state(checkpoint).machine);
  machine_.captured_fetch_owner_ = identity_;
  check_owner();
}
}
