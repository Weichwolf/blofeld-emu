// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/own_dsp_machine.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp56300/checkpoint.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/instruction.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace blofeld {
namespace {
class EffectGuard {
public:
  explicit EffectGuard(bool &active) : active_(active) { active_ = true; }

  ~EffectGuard() { active_ = false; }

  EffectGuard(const EffectGuard &) = delete;
  EffectGuard &operator=(const EffectGuard &) = delete;

private:
  bool &active_;
};

DspSpace native_space(const dsp56300::Space space) {
  switch (space) {
    case dsp56300::Space::program: return DspSpace::p;
    case dsp56300::Space::x: return DspSpace::x;
    case dsp56300::Space::y: return DspSpace::y;
    default: throw Fault("Invalid own DSP memory space");
  }
}

void qualify_native_memory(const dsp56300::PendingInstruction &pending) {
  const auto effect = pending.effect;
  if (effect.kind != dsp56300::EffectKind::read && effect.kind != dsp56300::EffectKind::write) {
    return;
  }
  const auto shape = pending.opcode ? dsp56300::inspect_instruction(*pending.opcode) : std::nullopt;
  if (shape && shape->kind != dsp56300::InstructionKind::bit_memory &&
      shape->kind != dsp56300::InstructionKind::bit_jump &&
      shape->kind != dsp56300::InstructionKind::bit_branch) {
    return;
  }
  constexpr std::array names{"P:", "X:", "Y:"};
  const auto space = native_space(effect.space);
  const auto *operation = effect.kind == dsp56300::EffectKind::read ? "read " : "write ";
  throw Fault("Unqualified own DSP DMA/native instruction " + std::string(operation) +
              names.at(static_cast<unsigned>(space)) + hex(effect.address, 6) +
              " pc=" + hex(effect.pc, 6) + " omr=" + hex(effect.omr, 6));
}

std::optional<DspMemoryAccess> native_footprint(const dsp56300::InstructionEffect effect) {
  using enum dsp56300::EffectKind;
  switch (effect.kind) {
    case fetch_opcode:
    case fetch_extension:
    case read:
      return DspMemoryAccess{
          .target = {.space = native_space(effect.space), .address = effect.address}};
    case write:
      return DspMemoryAccess{
          .target = {.space = native_space(effect.space), .address = effect.address},
          .operation = DspMemoryOperation::write};
    case retire:
    case address_update:
    case move_update:
    case alu_update:
    case unit_update:
    case pcu_update: return std::nullopt;
    case bit: {
      constexpr std::array names{"P:", "X:", "Y:"};
      const auto space = native_space(effect.space);
      throw Fault("Unqualified own DSP DMA/native bit " +
                  std::string(names.at(static_cast<unsigned>(space))) + hex(effect.address, 6) +
                  " pc=" + hex(effect.pc, 6) + " omr=" + hex(effect.omr, 6));
    }
    case reset:
      throw Fault("Unqualified own DSP DMA/native peripheral reset pc=" + hex(effect.pc, 6) +
                  " omr=" + hex(effect.omr, 6));
    default: throw Fault("Invalid own DSP native effect pc=" + hex(effect.pc, 6));
  }
}

}

struct OwnDspCheckpoint::Storage {
  DspRunClockInputs inputs;
  DspBus bus;
  dsp56300::Checkpoint cpu;
  std::optional<DspInterruptRequest> request;
  dsp56300::Word vector;
  bool first;
  DspRunClock clock;
  std::optional<OwnDspIo::Failure> failure;
};

OwnDspCheckpoint::OwnDspCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

OwnDspCheckpoint::~OwnDspCheckpoint() = default;

OwnDspMachine::OwnDspMachine(const DspRunClockInputs clock)
    : io_(bus_), cpu_(io_), interrupts_(bus_, cpu_), clock_(clock), clock_inputs_(clock) {}

OwnDspCheckpoint OwnDspMachine::checkpoint() const {
  if (advancing_effect_) { throw Fault("Own DSP checkpoint requires an inactive effect"); }
  // The native core rejects capture during an executing callback before any bus copy.
  auto native = cpu_.checkpoint();
  return OwnDspCheckpoint(std::make_shared<const OwnDspCheckpoint::Storage>(
      OwnDspCheckpoint::Storage{.inputs = clock_inputs_,
                                .bus = bus_,
                                .cpu = std::move(native),
                                .request = interrupts_.request_,
                                .vector = interrupts_.vector_,
                                .first = interrupts_.first_,
                                .clock = clock_,
                                .failure = io_.failure_}));
}

OwnDspCheckpoint OwnDspMachine::candidate_checkpoint(const dsp56300::EffectTicket &ticket) const {
  require_running();
  if (advancing_effect_) { throw Fault("Own DSP candidate requires an inactive effect"); }
  if (interrupts_.pending()) {
    throw Fault("Own DSP candidate requires completed interrupt vector ownership pc=" +
                hex(cpu_.registers().pc, 6));
  }
  const auto pending = cpu_.pending_instruction();
  if (!pending || pending->computed.omr != cpu_.registers().omr) {
    throw Fault("Own DSP candidate requires a published OMR boundary pc=" +
                hex(cpu_.registers().pc, 6));
  }
  auto native = cpu_.candidate_checkpoint(ticket);
  return OwnDspCheckpoint(std::make_shared<const OwnDspCheckpoint::Storage>(
      OwnDspCheckpoint::Storage{.inputs = clock_inputs_,
                                .bus = bus_,
                                .cpu = std::move(native),
                                .request = interrupts_.request_,
                                .vector = interrupts_.vector_,
                                .first = interrupts_.first_,
                                .clock = clock_,
                                .failure = io_.failure_}));
}

void OwnDspMachine::restore(const OwnDspCheckpoint &checkpoint) {
  if (advancing_effect_) { throw Fault("Own DSP restore requires an inactive effect"); }
  if (!checkpoint.storage_) { throw std::invalid_argument("Empty own DSP checkpoint"); }
  const auto &state = *checkpoint.storage_;
  if (state.inputs != clock_inputs_) {
    throw std::invalid_argument("Own DSP checkpoint clock inputs differ");
  }
  // Allocate the bus copy before native restore. Native restore prepares its
  // allocations before committing CPU state; the remaining assignments cannot throw.
  auto restored_bus = state.bus;
  auto restored_failure = state.failure;
  static_assert(std::is_nothrow_move_assignable_v<DspBus>);
  static_assert(std::is_nothrow_copy_assignable_v<DspRunClock>);
  static_assert(std::is_nothrow_copy_assignable_v<decltype(interrupts_.request_)>);
  static_assert(std::is_nothrow_move_assignable_v<decltype(io_.failure_)>);
  cpu_.restore(state.cpu);
  bus_ = std::move(restored_bus);
  clock_ = state.clock;
  interrupts_.request_ = state.request;
  interrupts_.vector_ = state.vector;
  interrupts_.first_ = state.first;
  io_.failure_ = std::move(restored_failure);
  captured_fetch_owner_.reset();
}

void OwnDspMachine::require_running() const {
  const auto reset = clock_.hardware_reset_state();
  if (!reset || reset->phase != DspHardwareResetPhase::complete || clock_.faulted()) {
    throw Fault("Own DSP execution requires completed hardware reset pc=" +
                hex(cpu_.registers().pc, 6));
  }
}

void OwnDspMachine::begin_hardware_reset(const OwnDspResetInputs inputs) {
  if (advancing_effect_) { throw Fault("Own DSP reset requires an inactive native effect"); }
  constexpr unsigned mode_mask = 15;
  constexpr unsigned spi_mode = 5;
  if (inputs.mode_known != mode_mask || inputs.mode_levels != spi_mode ||
      !inputs.pinit.has_value() || inputs.pinit.value() || inputs.recovery_cycles < 2 ||
      clock_.hardware_reset_active()) {
    throw Fault("Own DSP reset requires stable mode5/PINIT-low and "
                "at least two recovery periods pc=" +
                hex(cpu_.registers().pc, 6));
  }
  auto regs = cpu_.registers();
  try {
    captured_fetch_owner_.reset();
    clock_.restart_native_epoch();
    bus_.observe_irqa((inputs.mode_levels & 1U) != 0);
    bus_.pll().observe_pinit(inputs.pinit);
    bus_.reset();
    cpu_.reset();
    interrupts_.reset_after_cpu_reset();
    // FM figures 5-3/5-4, sections 4.3.4, 5.4.3/5.4.4 and mode5 boot.
    // Retain registers without reset initialization, including EP and SZ.
    constexpr dsp56300::Word reset_status = 0xc00300;
    constexpr dsp56300::Word reset_omr = 0x305;
    constexpr dsp56300::Word boot_entry = 0xff0000;
    regs.pc = boot_entry;
    regs.sr = reset_status;
    regs.omr = reset_omr;
    regs.sp = 0;
    regs.sc = 0;
    regs.vba = 0;
    regs.m.fill(dsp56300::word_mask);
    cpu_.set_registers(regs);
    io_.clear_failure();
    clock_.begin_hardware_reset(inputs.recovery_cycles);
  } catch (...) {
    clock_.fail();
    throw;
  }
}

std::uint64_t OwnDspMachine::advance_hardware_reset_ticks(const std::uint64_t maximum_ticks) {
  return clock_.advance_hardware_reset_ticks(maximum_ticks);
}

void OwnDspMachine::release_hardware_reset() {
  clock_.release_hardware_reset();
}

dsp56300::InstructionProgress
OwnDspMachine::begin_fetch(const dsp56300::InstructionPublication publication) {
  require_running();
  if (advancing_effect_) { throw Fault("Own DSP fetch requires an inactive effect"); }
  if (!captured_fetch_owner_.expired()) {
    throw Fault("Own DSP captured fetch already owns execution pc=" + hex(cpu_.registers().pc, 6));
  }
  return cpu_.begin_fetch(publication);
}

dsp56300::InstructionProgress OwnDspMachine::advance(const dsp56300::EffectTicket &ticket) {
  require_running();
  if (advancing_effect_) { throw Fault("Own DSP native effect cannot reenter its owner"); }
  const EffectGuard guard(advancing_effect_);
  auto result = interrupts_.advance(ticket);
  if (result.fault.kind == dsp56300::FaultKind::none) { bus_.observe_omr(cpu_.registers().omr); }
  return result;
}

std::uint64_t OwnDspMachine::advance_reference_cycle() {
  require_running();
  const auto regs = cpu_.registers();
  const auto context = [&regs] { return " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6); };
  if (clock_inputs_.switch_timing != DspClockSwitch::reference_cycle_boundary) {
    throw Fault("Own DSP reference cycle requires explicit cycle-boundary PLL timing" + context());
  }
  if (advancing_effect_ || cpu_.fault().kind != dsp56300::FaultKind::none) {
    throw Fault("Own DSP reference cycle requires an inactive healthy native effect" + context());
  }
  if (cpu_.processing_state() != dsp56300::ProcessingState::running &&
      cpu_.processing_state() != dsp56300::ProcessingState::waiting) {
    throw Fault("Own DSP reference cycle requires qualified oscillator state" + context());
  }
  if (bus_.has_active_dma() || bus_.last_dma_cycle()) {
    throw Fault("Own DSP reference cycle requires DMA arbitration at X:0xfffff4" + context());
  }
  const auto duration = clock_.next_step_ticks(bus_, 1);
  clock_.begin_step(bus_, {.pc = regs.pc, .omr = regs.omr});
  clock_.finish_step(bus_, 1);
  return duration;
}

DspDmaBusResult OwnDspMachine::advance_dma_reference_cycle(const DspDmaInputs inputs) {
  return dma_cycle(inputs, false).dma;
}

OwnDspDmaCycleResult OwnDspMachine::advance_dma_native_cycle(const DspDmaInputs inputs,
                                                             const OwnDspNativeCycleModel model) {
  if (model != OwnDspNativeCycleModel::isolated_effect) {
    throw Fault("Own DSP DMA/native cycle requires an explicit isolated-effect hypothesis pc=" +
                hex(cpu_.registers().pc, 6));
  }
  return dma_cycle(inputs, true);
}

OwnDspDmaCycleResult OwnDspMachine::dma_cycle(const DspDmaInputs inputs, const bool admit_native) {
  require_running();
  const auto regs = cpu_.registers();
  const auto context = " pc=" + hex(regs.pc, 6) + " omr=" + hex(regs.omr, 6);
  if (clock_inputs_.switch_timing != DspClockSwitch::reference_cycle_boundary ||
      advancing_effect_ || cpu_.fault().kind != dsp56300::FaultKind::none ||
      (cpu_.processing_state() != dsp56300::ProcessingState::running &&
       cpu_.processing_state() != dsp56300::ProcessingState::waiting)) {
    throw Fault("Own DSP DMA cycle requires an inactive healthy cycle clock" + context);
  }
  if (!captured_fetch_owner_.expired()) {
    throw Fault("Own DSP DMA cycle requires an unoccupied captured-fetch bus" + context);
  }
  const auto pending = cpu_.pending_instruction();
  if (admit_native && !pending) {
    throw Fault("Own DSP DMA/native cycle requires an actual pending effect" + context);
  }
  std::array<DspMemoryAccess, 1> access{};
  std::size_t count = 0;
  if (admit_native) {
    qualify_native_memory(*pending);
    if (const auto footprint = native_footprint(pending->effect)) { access[count++] = *footprint; }
  }
  const EffectGuard guard(advancing_effect_);
  static_cast<void>(clock_.next_step_ticks(bus_, 1));
  const bool switch_was_pending = bus_.switch_delay_ != 0;
  clock_.begin_step(bus_, {.pc = regs.pc, .omr = regs.omr});
  try {
    OwnDspDmaCycleResult result;
    result.dma = bus_.advance_dma(inputs,
                                  {.accesses = std::span(access).first(count),
                                   .knowledge = DspCoreBusKnowledge::reference_cycle_occupancy});
    if (admit_native && !result.dma.core_stall) {
      result.native = interrupts_.advance(pending->ticket);
      if (result.native->fault.kind == dsp56300::FaultKind::none) {
        bus_.observe_omr(cpu_.registers().omr);
      }
    }
    clock_.finish_dma_cycle(bus_, inputs.cycle, switch_was_pending);
    return result;
  } catch (const std::exception &error) {
    clock_.fail();
    throw Fault("Own DSP DMA cycle" + context + ": " + error.what());
  }
}

bool OwnDspMachine::admit_interrupt(const dsp56300::InstructionWords foreground) {
  require_running();
  if (advancing_effect_) { throw Fault("Own DSP interrupt requires an inactive effect"); }
  if (!captured_fetch_owner_.expired()) {
    throw Fault("Own DSP captured fetch cannot admit interrupt pc=" + hex(cpu_.registers().pc, 6));
  }
  return interrupts_.admit(foreground);
}
}
