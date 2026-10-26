// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_fetch_run.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_cycle.hpp"
#include "dsp/dsp_reference_execution.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <variant>

namespace blofeld {
namespace {
bool program_read(const DspReferencePending &pending) {
  const auto *effect = std::get_if<DspNativeAccess>(&pending.effect);
  return effect != nullptr && (effect->kind == DspNativeAccessKind::opcode ||
                               effect->kind == DspNativeAccessKind::extension);
}
}

void DspFetchState::bind(const DspMachine &machine) {
  if (pipeline) { pipeline->rebind(machine); }
  const auto pending = machine.reference_pending();
  const auto previous = machine.reference_retirement();
  expected_retirement = previous ? std::optional{previous->ticket} : std::nullopt;
  expected_cpu = machine.cpu().debug_state();
  expected_ticket = pending ? std::optional{pending->ticket} : std::nullopt;
  expected_latch = pending ? pending->latched_program_word : std::nullopt;
  expected_fetch = machine.reference_fetch();
  expected_dma = machine.bus().last_dma_cycle();
  expected_owner = machine.reference_clock_owner();
}

void DspFetchState::verify(const DspMachine &machine) const {
  const auto pending = machine.reference_pending();
  const auto ticket = pending ? std::optional{pending->ticket} : std::nullopt;
  const auto latch = pending ? pending->latched_program_word : std::nullopt;
  const auto previous = machine.reference_retirement();
  const auto retirement = previous ? std::optional{previous->ticket} : std::nullopt;
  if (machine.clocked() || machine.cpu().debug_state() != expected_cpu ||
      ticket != expected_ticket || retirement != expected_retirement || latch != expected_latch ||
      machine.reference_fetch() != expected_fetch ||
      machine.bus().last_dma_cycle() != expected_dma ||
      machine.reference_clock_owner() != expected_owner) {
    throw Fault("DSP fetch clock requires coordinated native/frontend/DMA state pc=" +
                hex(expected_cpu.cpu.pc, 6) + " omr=" + hex(expected_cpu.cpu.omr, 6));
  }
}

std::uint64_t DspFetchState::period(const DspMachine &machine) const {
  const auto ratio = machine.bus().pll().target_ratio();
  const auto numerator = inputs.timebase_hz * ratio.denominator;
  const auto denominator = static_cast<std::uint64_t>(inputs.extal_hz) * ratio.numerator;
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (numerator < denominator || numerator % denominator != 0 || cycles == maximum ||
      next_dma_cycle == maximum || numerator / denominator > maximum - ticks) {
    throw Fault("Unrepresentable or exhausted DSP fetch cycle period");
  }
  return numerator / denominator;
}

DspReferenceCycleInputs DspFetchState::plan(const DspMachine &machine,
                                            const DspFetchCycleInputs input,
                                            bool &starved) const {
  DspReferenceCycleInputs result{.dma = {.cycle = next_dma_cycle,
                                         .known_requests = input.known_requests,
                                         .requests = input.requests,
                                         .timing = DspDmaTiming::reference_sampled_cycles,
                                         .grant = input.grant},
                                 .order = DspReferenceBusOrder::dma_then_fetch_then_native};
  const auto fetch = machine.reference_fetch();
  if (fetch.pending_address) { result.fetch = DspProgramFetchStage::read; }
  if (pipeline) {
    if (pipeline->view.awaiting_retirement) { result.access = pipeline->retirement(machine); }
    if (pipeline->view.cycle.memory_source_stall) {
      result.fetch.reset();
      return result;
    }
    if (pipeline->view.cycle.agu_dependencies != 0 || pipeline->held_branch_decode) {
      result.fetch.reset();
    }
    const auto ag1 = pipeline->view.cycle.ag1;
    if (pipeline->profile == DspFetchPipeline::automatic_memory && ag1) {
      const auto word = *ag1;
      if (decode_peripheral_read(word.value)) { result.peripheral_capture = word.address; }
    }
    result.address_update = pipeline->addresses[0];
    result.memory = pipeline->addresses[1];
    return result;
  }
  if (input.retire_previous) {
    const auto previous = machine.reference_retirement();
    if (!previous) { throw Fault("DSP fetch clock has no prior result to retire"); }
    result.retirement = previous->ticket;
  }
  if (!input.admit_native) { return result; }
  const auto pending = machine.reference_pending();
  if (!pending) { throw Fault("DSP fetch clock has no pending native effect to admit"); }
  starved = program_read(*pending) && !pending->latched_program_word;
  if (!starved) { result.access = pending->ticket; }
  return result;
}
}
