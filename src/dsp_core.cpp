// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_core.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_interrupts.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_program_fetch.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp56kEmu/peripherals.h"
#include "dsp56kEmu/registers.h"
#include "dsp56kEmu/types.h"
#include "dsp_address_phase.hpp"
#include "dsp_interrupt_timing.hpp"
#include "dsp_native_access.hpp"
#include "dsp_reference_execution.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

namespace blofeld {
namespace {
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned address_space = 0x1000000;
constexpr unsigned native_xy_words = 0x18000;
constexpr unsigned spi_mode = 5;
constexpr unsigned reset_core_dma_priority = 0x300;
constexpr unsigned boot_entry = 0xff0000;
constexpr std::uint64_t data_mask = 0xffffffffffff;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;

class ExecutionGuard {
public:
  explicit ExecutionGuard(bool &active) : active_(active) {
    if (active_) { throw Fault("Cannot reenter active DSP execution"); }
    active_ = true;
  }

  ~ExecutionGuard() { active_ = false; }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;

private:
  bool &active_;
};

DspSpace space(const dsp56k::EMemArea area) {
  switch (area) {
    case dsp56k::MemArea_P: return DspSpace::p;
    case dsp56k::MemArea_X: return DspSpace::x;
    case dsp56k::MemArea_Y: return DspSpace::y;
    case dsp56k::MemArea_COUNT: throw Fault("Invalid native DSP memory-space count sentinel");
    default: throw Fault("Unsupported native DSP memory space");
  }
}

DspBitOperation bit_operation(const dsp56k::PeripheralBitOp operation) {
  switch (operation) {
    case dsp56k::PeripheralBitOp::Test: return DspBitOperation::test;
    case dsp56k::PeripheralBitOp::Clear: return DspBitOperation::clear;
    case dsp56k::PeripheralBitOp::Set: return DspBitOperation::set;
    case dsp56k::PeripheralBitOp::Change: return DspBitOperation::change;
    default: throw Fault("Invalid native DSP peripheral bit operation");
  }
}

class MemoryAccess final : public dsp56k::IMemoryValidator {
public:
  MemoryAccess(DspBus &bus, DspNativeBus &native) : bus_(bus), native_(native) {}

  [[nodiscard]] bool memValidateAccess(dsp56k::EMemArea /*area*/,
                                       dsp56k::TWord /*address*/,
                                       bool /*write*/) const override {
    throw Fault("Native DSP array access bypassed the strict board bus");
  }

  bool readWord(const dsp56k::EMemArea area,
                const dsp56k::TWord address,
                dsp56k::TWord &value) const override {
    value = native_.read({.space = space(area), .address = address});
    return true;
  }

  [[nodiscard]] bool writeWord(const dsp56k::EMemArea area,
                               const dsp56k::TWord address,
                               const dsp56k::TWord value) const override {
    native_.write({.space = space(area), .address = address, .value = value});
    return true;
  }

  bool readExtension(const dsp56k::TWord address, dsp56k::TWord &value) const override {
    value = native_.read({.address = address}, DspNativeAccessKind::extension);
    return true;
  }

  bool readOpcode(const dsp56k::TWord address,
                  dsp56k::TWord &first,
                  dsp56k::TWord &second) const override {
    const bool vector = interrupt_ && address == interrupt_->address;
    first = native_.read({.address = address},
                         vector ? DspNativeAccessKind::vector : DspNativeAccessKind::opcode);
    if (vector) {
      second = native_.read({.address = address + 1}, DspNativeAccessKind::vector);
      if (interrupt_->expected && *interrupt_->expected != std::array{first, second}) {
        throw Fault("DSP interrupt vector changed after timing preview read P:" + hex(address, 6));
      }
      if (native_.reference().intercepting()) {
        native_.reference().acknowledge_interrupt(interrupt_->request);
      } else if (interrupt_->admitted) {
        bus_.acknowledge_admitted_interrupt(interrupt_->request);
      } else {
        bus_.acknowledge_interrupt(interrupt_->request);
      }
      interrupt_.reset();
    } else {
      second = native_.peek_program(address + 1);
    }
    return true;
  }

  void arm_interrupt(const DspInterruptRequest request,
                     const std::uint32_t address,
                     const std::optional<std::array<std::uint32_t, 2>> expected = std::nullopt,
                     const bool admitted = false) {
    if (interrupt_) { throw Fault("DSP interrupt vector fetch is already armed"); }
    interrupt_ = VectorFetch{
        .request = request, .address = address, .expected = expected, .admitted = admitted};
  }

  void verify_interrupt_fetch() const {
    if (interrupt_) { throw Fault("Native DSP interrupt bypassed the strict vector fetch"); }
  }

  void reset() { interrupt_.reset(); }

private:
  struct VectorFetch {
    DspInterruptRequest request;
    std::uint32_t address = 0;
    std::optional<std::array<std::uint32_t, 2>> expected;
    bool admitted = false;
  };

  DspBus &bus_;
  DspNativeBus &native_;
  mutable std::optional<VectorFetch> interrupt_;
};

class Peripherals final : public dsp56k::IPeripherals {
public:
  Peripherals(DspNativeBus &native, const DspSpace area)
      : IPeripherals(dsp56k::PeripheralType::PeripheralsCustom), native_(native), area_(area) {}

  dsp56k::TWord read(const dsp56k::TWord address, dsp56k::Instruction /*instruction*/) override {
    return native_.read({.space = area_, .address = address});
  }

  bool bitAccess(const dsp56k::TWord address,
                 const dsp56k::TWord bit,
                 const dsp56k::PeripheralBitOp operation,
                 bool &old) override {
    const auto result = native_.bit(
        {.space = area_, .address = address, .bit = bit, .operation = bit_operation(operation)});
    if (!result.has_value()) { return false; }
    old = *result;
    return true;
  }

  const dsp56k::TWord *readAsPtr(dsp56k::TWord /*address*/,
                                 dsp56k::Instruction /*instruction*/) override {
    throw Fault("Native DSP direct peripheral pointer bypassed the board bus");
  }

  void write(const dsp56k::TWord address, const dsp56k::TWord value) override {
    native_.write({.space = area_, .address = address, .value = value});
  }

  std::uint32_t exec() override { return MaxDelayCycles; }

  void reset() override {
    if (area_ == DspSpace::x) { native_.reset_peripherals(); }
  }

  void setSymbols(dsp56k::Disassembler & /*disassembler*/) const override {}

  void terminate() override {}

private:
  DspNativeBus &native_;
  DspSpace area_;
};

std::uint32_t word(const dsp56k::TReg24 &value) {
  return static_cast<std::uint32_t>(value.var) & word_mask;
}

std::uint64_t register_value(const dsp56k::DSP &cpu, const dsp56k::EReg reg) {
  std::int64_t value = 0;
  if (!cpu.readRegToInt(reg, value)) { throw Fault("Cannot read native DSP register"); }
  return std::bit_cast<std::uint64_t>(value);
}
}

struct DspRetirementResult {
  dsp56k::DSP::ReferenceAttemptState cpu;
  bool redirected = false;
};

struct DspNativeResultState {
  std::optional<dsp56k::DSP::ReferenceAttemptState> base;
  std::optional<DspRetirementResult> retirement;
  DspAddressPhase addresses;
};

struct DspCoreCheckpoint::Storage {
  Storage(dsp56k::DSP::ReferenceState state,
          const DspReferenceState &reference_state,
          const std::uint64_t serial,
          const DspNativeResultState &result)
      : cpu(std::move(state)), reference(reference_state), generation(serial), results(result) {}

  const dsp56k::DSP::ReferenceState cpu;
  const DspReferenceState reference;
  const std::uint64_t generation;
  const DspNativeResultState results;
};

DspCoreCheckpoint::DspCoreCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspCoreCheckpoint::~DspCoreCheckpoint() = default;

class DspCore::Impl {
public:
  explicit Impl(DspBus &board)
      : bus(board),
        native(board),
        access(board, native),
        memory(access, address_space, native_xy_words, 0),
        peripherals_x(native, DspSpace::x),
        peripherals_y(native, DspSpace::y),
        cpu(memory, &peripherals_x, &peripherals_y) {
    static_assert(!dsp56k::g_useJIT);
    static_assert(std::is_trivially_copyable_v<dsp56k::DSP::ReferenceAttemptState>);
    static_assert(sizeof(dsp56k::DSP::ReferenceAttemptState) <= 1024);
  }

  void invalidate() {
    const auto changes = bus.take_code_changes();
    if (changes.all) {
      cpu.clearOpcodeCache();
    } else {
      for (const auto address : changes.addresses) { cpu.clearOpcodeCache(address); }
    }
  }

  void begin_run(const DspState before, const bool interrupt) const {
    if (run.begin) {
      const auto units = run.fetched && !interrupt ? cpu.referenceNextCycles() : 0;
      run.begin(before, interrupt, units);
    }
  }

  [[nodiscard]] DspNativeAccessObserver timing_observer(DspNativeAccessObserver observer) const {
    if (!run.fetched && !run.data) { return observer; }
    return {.observe = [this, observer = std::move(observer)](const DspNativeAccess &event) {
      if (event.kind == DspNativeAccessKind::opcode && event.stage == DspNativeAccessStage::after &&
          run.fetched) {
        if (!event.value) { throw Fault("DSP interlock opcode fetch lost its value"); }
        run.fetched(*event.value);
      }
      if (event.stage == DspNativeAccessStage::before && run.data &&
          (event.kind == DspNativeAccessKind::read || event.kind == DspNativeAccessKind::write)) {
        run.data({.target = event.target,
                  .operation = event.kind == DspNativeAccessKind::write
                                   ? DspMemoryOperation::write
                                   : DspMemoryOperation::read});
      }
      if (observer.observe) { observer.observe(event); }
    }};
  }

  [[nodiscard]] StepEstimate interrupt_estimate(const DspState &before,
                                                const DspInterruptRequest request) const {
    const auto address = before.vba + request.vector;
    const DspInterruptVector vector{
        .request = request,
        .address = address,
        .words = {bus.inspect_program(address), bus.inspect_program(address + 1)}};
    return {.units = nominal_vector_cycles(vector, opcodes, memory.getBridgedMemoryAddress()),
            .interrupt = vector};
  }

  [[nodiscard]] std::optional<DspInterruptRequest>
  select_interrupt(const DspState &before, const Admission *admission) const {
    constexpr unsigned mask_shift = 8;
    constexpr unsigned interrupt_mask = 3;
    const auto mask = (before.sr >> mask_shift) & interrupt_mask;
    auto selected = admission != nullptr ? admission->interrupt : bus.interrupt_request(mask);
    if (!selected) { return std::nullopt; }
    const bool eligible = cpu.referenceInterruptible(bus.peek_program(before.pc));
    if (admission != nullptr) {
      if (!eligible || selected->level < mask) {
        throw Fault("DSP admitted interrupt is no longer eligible");
      }
      bus.validate_admitted_interrupt(*selected);
    } else if (!eligible) {
      selected.reset();
    }
    return selected;
  }

  void reset_reference() {
    auto next_owner = std::make_shared<const bool>(true);
    native.reference().clear();
    results = {};
    reference_owner = std::move(next_owner);
    generation = 0;
  }

  void restore_reference_base() {
    if (!results.base) { throw Fault("Native reference instruction lost its base state"); }
    const DspReferenceScope restoring(native.reference(), DspReferenceMode::restoring);
    cpu.restoreReferenceAttempt(*results.base);
  }

  void restore_visible_reference() {
    restore_reference_base();
    for (unsigned index = 0; index < results.addresses.published.size(); ++index) {
      if (const auto value = results.addresses.published.at(index)) {
        cpu.regs().r.at(index).var = static_cast<std::int32_t>(*value);
      }
    }
  }

  void capture_address_phase() {
    auto &reference = native.reference();
    if (reference.execution() != DspReferenceExecution::address_and_retirement_boundaries) {
      return;
    }
    const auto &pending = reference.pending();
    if (!pending) { throw Fault("Native address phase lost its real access"); }
    const auto *request = std::get_if<DspNativeAccess>(&*pending);
    if (request == nullptr || request->kind == DspNativeAccessKind::opcode ||
        request->kind == DspNativeAccessKind::extension) {
      return;
    }
    if (request->kind != DspNativeAccessKind::read && request->kind != DspNativeAccessKind::write) {
      throw Fault("Unqualified native address-phase access pc=" + hex(request->pc, 6) +
                  " address=" + hex(request->target.address, 6) + " omr=" + hex(request->omr, 6));
    }
    const auto opcode = reference.opcode();
    if (!opcode || !results.base) {
      throw Fault("Native address phase lost its fetched instruction base");
    }
    std::array<std::uint32_t, 8> before{};
    std::array<std::uint32_t, 8> partial{};
    for (unsigned index = 0; index < before.size(); ++index) {
      before.at(index) =
          static_cast<std::uint32_t>(results.base->registers.r.at(index)) & word_mask;
      partial.at(index) = word(cpu.regs().r.at(index));
    }
    results.addresses.capture(*opcode, *request, before, partial);
  }

  DspAddressUpdate publish_address_phase() {
    if (!results.addresses.pending) {
      throw Fault("Native address phase lost its held register value");
    }
    const auto result = *results.addresses.pending;
    cpu.regs().r.at(result.index).var = static_cast<std::int32_t>(result.value);
    results.addresses.published.at(result.index) = result.value;
    results.addresses.pending.reset();
    return result;
  }

  [[nodiscard]] std::pair<dsp56k::Instruction, std::uint32_t>
  fetch_instruction(const std::uint32_t opcode) const {
    auto first = dsp56k::Invalid;
    auto second = dsp56k::Invalid;
    if (opcodes.getInstructionTypes(opcode, first, second) == 0) {
      throw Fault("Undefined native program-fetch opcode=" + hex(opcode, 6));
    }
    return {first, dsp56k::Opcodes::getFlags(first, second)};
  }

  void check_fetch_opcode() const {
    const auto &reference = native.reference();
    const auto pending = reference.pending();
    if (!reference.fetch().active() || !pending) { return; }
    const auto *request = std::get_if<DspNativeAccess>(&*pending);
    if (request == nullptr || request->kind != DspNativeAccessKind::opcode) { return; }
    const auto value = reference.latched_program_word();
    if (!value) { return; }
    const auto [instruction, flags] = fetch_instruction(*value);
    if ((flags & (dsp56k::OpFlagLoop | dsp56k::OpFlagCacheMod)) != 0 ||
        instruction == dsp56k::Stop || instruction == dsp56k::Wait || instruction == dsp56k::Trap ||
        instruction == dsp56k::Trapcc || instruction == dsp56k::Debug ||
        instruction == dsp56k::Debugcc || instruction == dsp56k::Illegal ||
        instruction == dsp56k::Reset) {
      throw Fault("Unqualified native program-fetch control opcode at P:" +
                  hex(request->target.address, 6) + " pc=" + hex(request->pc, 6) +
                  " omr=" + hex(request->omr, 6));
    }
  }

  void finish_fetch(const bool redirected) {
    auto &reference = native.reference();
    auto &fetch = reference.fetch();
    if (!fetch.active()) { return; }
    const auto next = word(cpu.regs().pc);
    const auto omr = word(cpu.regs().omr);
    constexpr unsigned loop_cache_or_compatibility = 0x1a000;
    if ((word(cpu.regs().sr) & loop_cache_or_compatibility) != 0) {
      throw Fault("Unqualified native program-fetch mode change pc=" +
                  hex(reference.snapshot().context.pc, 6) + " omr=" + hex(omr, 6));
    }
    const auto view = fetch.view();
    if (!redirected && next != view.consume_address) {
      throw Fault("Unqualified native program-fetch PC transition pc=" +
                  hex(reference.snapshot().context.pc, 6) + " next=" + hex(next, 6) +
                  " expected=" + hex(view.consume_address, 6) + " omr=" + hex(omr, 6));
    }
    if (redirected || omr != view.omr) { fetch.redirect(next, omr); }
  }

  void finish_reference(const bool redirected) {
    bus.observe_omr(word(cpu.regs().omr));
    finish_fetch(redirected);
    native.reference().finish();
    results = {};
  }

  DspReferenceEffect retire_reference() {
    const auto &pending = native.reference().pending();
    if (!results.retirement || !pending ||
        !std::holds_alternative<DspInstructionRetire>(*pending)) {
      throw Fault("Native retirement lost its held instruction result");
    }
    const auto completed = *pending;
    const bool redirected = results.retirement->redirected;
    {
      const DspReferenceScope restoring(native.reference(), DspReferenceMode::restoring);
      cpu.restoreReferenceAttempt(results.retirement->cpu);
    }
    finish_reference(redirected);
    return completed;
  }

  void resume_reference() {
    restore_reference_base();
    auto &reference = native.reference();
    reference.rewind();
    try {
      const DspReferenceScope attempting(reference, DspReferenceMode::attempt);
      if (const auto &interrupt = reference.interrupt()) {
        access.arm_interrupt(interrupt->request, interrupt->address);
        cpu.execReferenceInterruptAttempt(interrupt->address, interrupt->request.level);
        access.verify_interrupt_fetch();
      } else {
        cpu.execReferenceAttempt();
      }
      reference.verify_complete();
    } catch (const DspReferenceYield &) {
      access.reset();
      try {
        capture_address_phase();
      } catch (...) {
        restore_visible_reference();
        throw;
      }
      restore_visible_reference();
      return;
    }
    // Validate native continuation constraints without publishing a partial step.
    const auto result = cpu.referenceAttemptState();
    const bool redirected = cpu.referenceRedirected();
    if (reference.execution() == DspReferenceExecution::access_and_retirement_boundaries ||
        reference.execution() == DspReferenceExecution::address_and_retirement_boundaries) {
      const auto before = reference.snapshot().context;
      reference.stage_retirement({.pc = before.pc,
                                  .next_pc = word(cpu.regs().pc),
                                  .omr = before.omr,
                                  .next_omr = word(cpu.regs().omr),
                                  .redirected = redirected});
      results.retirement = DspRetirementResult{.cpu = result, .redirected = redirected};
      restore_visible_reference();
    } else {
      finish_reference(redirected);
    }
  }

  DspBus &bus;
  DspNativeBus native;
  MemoryAccess access;
  dsp56k::Memory memory;
  Peripherals peripherals_x;
  Peripherals peripherals_y;
  dsp56k::DSP cpu;
  const dsp56k::Opcodes opcodes;
  bool faulted = false;
  bool active = false;
  RunHooks run;
  DspNativeResultState results;
  std::shared_ptr<const bool> reference_owner;
  std::uint64_t generation = 0;
};

DspCore::DspCore(DspBus &bus) : impl_(std::make_unique<Impl>(bus)) {
  reset_spi_boot();
}

DspCore::~DspCore() = default;

void DspCore::reset_spi_boot() {
  const ExecutionGuard guard(impl_->active);
  if (impl_->run.reset) { impl_->run.reset(); }
  impl_->faulted = true;
  impl_->reset_reference();
  impl_->bus.reset();
  impl_->cpu.resetHW();
  // DSP56300FM figure 5-3/table 5-2: hardware reset sets CDP1/CDP0;
  // mode pins select the low nibble. The generic core resets OMR to zero.
  impl_->cpu.regs().omr.var = reset_core_dma_priority | spi_mode;
  impl_->cpu.setPC(boot_entry);
  impl_->access.reset();
  impl_->faulted = false;
  impl_->invalidate();
}

void DspCore::set_run_hooks(RunHooks hooks) {
  if (impl_->active || impl_->native.reference().active()) {
    throw Fault("Cannot configure an active DSP run");
  }
  impl_->run = std::move(hooks);
}

bool DspCore::faulted() const {
  return impl_->faulted;
}

std::uint64_t DspCore::native_cycles() const {
  return impl_->cpu.getCycles();
}

DspCore::StepEstimate DspCore::next_step_cycles(const bool vector_estimates,
                                                const Admission *admission) const {
  if (impl_->active || impl_->faulted || impl_->native.reference().active()) {
    throw Fault("DSP cycle preview requires a healthy completed step");
  }
  const auto before = state();
  try {
    constexpr unsigned compatibility = 0x2000;
    const auto interrupt = impl_->select_interrupt(before, admission);
    if ((before.sr & compatibility) != 0 || (interrupt && !vector_estimates)) {
      throw Fault("Unqualified DSP interrupt or compatibility-mode timing preview");
    }
    if (interrupt) { return impl_->interrupt_estimate(before, *interrupt); }
    return {.units = impl_->cpu.referenceNextCycles(), .interrupt = std::nullopt};
  } catch (const std::exception &error) {
    throw Fault("DSP cycle preview pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + ": " +
                error.what());
  }
}

bool DspCore::executing() const {
  return impl_->active;
}

DspCoreCheckpoint DspCore::checkpoint() const {
  if (impl_->active || impl_->faulted) {
    throw Fault("DSP checkpoint requires a completed reference step or reset");
  }
  impl_->access.verify_interrupt_fetch();
  return DspCoreCheckpoint(
      std::make_shared<const DspCoreCheckpoint::Storage>(impl_->cpu.referenceState(),
                                                         impl_->native.reference().snapshot(),
                                                         impl_->generation,
                                                         impl_->results));
}

void DspCore::restore_checkpoint(const DspCoreCheckpoint &checkpoint) {
  const ExecutionGuard guard(impl_->active);
  if (!checkpoint.storage_) { throw Fault("Cannot restore an empty native DSP checkpoint"); }
  impl_->faulted = true;
  impl_->reset_reference();
  {
    const DspReferenceScope restoring(impl_->native.reference(), DspReferenceMode::restoring);
    impl_->cpu.restoreReferenceState(checkpoint.storage_->cpu);
  }
  impl_->native.reference().restore(checkpoint.storage_->reference);
  impl_->results = checkpoint.storage_->results;
  impl_->generation = checkpoint.storage_->generation;
  if (checkpoint.storage_->reference.active && !impl_->results.base) {
    throw Fault("Native reference checkpoint lost its original computation base");
  }
  impl_->access.reset();
  impl_->faulted = false;
}

std::optional<DspReferencePending> DspCore::reference_pending_unchecked() const {
  const auto &reference = impl_->native.reference();
  const auto &pending = reference.pending();
  if (!pending) { return std::nullopt; }
  const auto &address = impl_->results.addresses.pending;
  return DspReferencePending{.effect = address ? DspReferenceEffect{*address} : *pending,
                             .ticket = DspReferenceTicket(impl_->reference_owner,
                                                          {.generation = impl_->generation,
                                                           .record = reference.pending_index(),
                                                           .address = address.has_value()}),
                             .latched_program_word = reference.latched_program_word(),
                             .execution = reference.execution()};
}

std::optional<DspReferencePending> DspCore::reference_pending() const {
  if (impl_->active || impl_->faulted) {
    throw Fault("Native reference inspection requires a healthy suspension boundary");
  }
  return reference_pending_unchecked();
}

DspReferenceProgress
DspCore::reference_progress(std::optional<DspReferenceEffect> completed) const {
  return {.completed = completed,
          .pending = reference_pending_unchecked(),
          .retired = !impl_->native.reference().active()};
}

DspReferenceProgress DspCore::begin_reference_instruction(const DspReferenceExecution contract) {
  return begin_reference_step(contract, false);
}

DspReferenceProgress DspCore::begin_reference_interrupt(const DspReferenceExecution contract) {
  return begin_reference_step(contract, true);
}

DspReferenceProgress DspCore::begin_reference_step(const DspReferenceExecution contract,
                                                   const bool interrupt_entry) {
  if (impl_->active || impl_->faulted || impl_->native.reference().active() ||
      (contract != DspReferenceExecution::access_boundaries &&
       contract != DspReferenceExecution::access_and_retirement_boundaries &&
       contract != DspReferenceExecution::address_and_retirement_boundaries) ||
      impl_->run.begin || impl_->run.complete || impl_->run.failed || impl_->run.reset) {
    throw Fault(
        "Native access suspension requires an explicit healthy unclocked instruction boundary");
  }
  const auto before = state();
  if (contract == DspReferenceExecution::address_and_retirement_boundaries &&
      (interrupt_entry || impl_->cpu.referenceAttemptState().repeat.remaining != 0)) {
    throw Fault("Unqualified native address-phase interrupt/REP timeline pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
  if (impl_->native.reference().fetch().active() && interrupt_entry) {
    throw Fault("Unqualified native program-fetch interrupt timeline pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
  constexpr unsigned mask_shift = 8;
  constexpr unsigned interrupt_mask = 3;
  const auto interrupt = impl_->bus.interrupt_request((before.sr >> mask_shift) & interrupt_mask);
  const auto queued_opcode = impl_->native.reference().fetch().peek(before.pc);
  const bool accepted =
      interrupt && impl_->cpu.referenceInterruptible(
                       queued_opcode ? *queued_opcode : impl_->bus.peek_program(before.pc));
  if (interrupt_entry && !accepted) {
    throw Fault("No admissible native reference interrupt pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
  if (!interrupt_entry && accepted) {
    throw Fault("Unqualified native reference interrupt entry pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6) + " vector=" + hex(before.vba + interrupt->vector, 6) +
                " level=" + std::to_string(interrupt->level));
  }
  if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Native reference owner exhausted its instruction generations");
  }
  const ExecutionGuard guard(impl_->active);
  try {
    impl_->invalidate();
    impl_->access.verify_interrupt_fetch();
    impl_->results.base = impl_->cpu.referenceAttemptState();
    ++impl_->generation;
    std::optional<DspReferenceInterrupt> entry;
    if (interrupt_entry) {
      entry =
          DspReferenceInterrupt{.request = *interrupt, .address = before.vba + interrupt->vector};
    }
    impl_->native.reference().begin({.pc = before.pc, .omr = before.omr}, contract, entry);
    impl_->resume_reference();
    return reference_progress();
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("DSP reference instruction pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) +
                ": " + error.what());
  } catch (...) {
    impl_->faulted = true;
    throw Fault("Non-standard DSP reference instruction failure pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
}

void DspCore::validate_reference_ticket(const DspReferenceTicket &ticket) const {
  const auto &reference = impl_->native.reference();
  if (impl_->active || impl_->faulted || !reference.pending() ||
      ticket.owner_ != impl_->reference_owner || ticket.position_.generation != impl_->generation ||
      ticket.position_.record != reference.pending_index() ||
      ticket.position_.address != impl_->results.addresses.pending.has_value()) {
    throw Fault("Stale, foreign or unavailable native reference access ticket");
  }
}

DspReferencePending DspCore::latch_reference_program(const DspReferenceTicket &ticket) {
  validate_reference_ticket(ticket);
  static_cast<void>(impl_->native.reference().check_program_latch());
  const auto before = state();
  const ExecutionGuard guard(impl_->active);
  try {
    impl_->native.reference().latch_program();
    const auto pending = reference_pending_unchecked();
    if (!pending) { throw Fault("Program latch lost its native admission boundary"); }
    return *pending;
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("DSP program latch pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) + ": " +
                error.what());
  } catch (...) {
    impl_->faulted = true;
    throw Fault("Non-standard DSP program latch failure pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
}

DspProgramFetchView DspCore::start_reference_fetch(const DspReferenceTicket &ticket) {
  validate_reference_ticket(ticket);
  const auto &effect = impl_->native.reference().check_program_latch();
  const auto &access = std::get<DspNativeAccess>(effect);
  constexpr unsigned loop_cache_or_compatibility = 0x1a000;
  if (access.kind != DspNativeAccessKind::opcode || impl_->native.reference().interrupt() ||
      (state().sr & loop_cache_or_compatibility) != 0 ||
      impl_->cpu.referenceAttemptState().repeat.remaining != 0) {
    throw Fault("Program-fetch start requires an ordinary pending opcode P:" +
                hex(access.target.address, 6) + " pc=" + hex(access.pc, 6) +
                " omr=" + hex(access.omr, 6));
  }
  impl_->native.reference().fetch().start(access.target.address, access.omr);
  return reference_fetch();
}

DspProgramFetchView DspCore::reference_fetch() const {
  if (impl_->active || impl_->faulted) {
    throw Fault("Program-fetch inspection requires a healthy native boundary");
  }
  return impl_->native.reference().fetch().view();
}

DspProgramFetchView DspCore::advance_reference_fetch(const DspProgramFetchStage stage) {
  validate_reference_fetch(stage);
  auto &fetch = impl_->native.reference().fetch();
  const auto before = state();
  const auto view = fetch.view();
  const auto address = view.pending_address.value_or(view.next_address);
  const ExecutionGuard guard(impl_->active);
  try {
    if (stage == DspProgramFetchStage::address) {
      fetch.issue();
    } else {
      fetch.capture(impl_->bus);
    }
    return fetch.view();
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("DSP program-fetch read at P:" + hex(address, 6) + " pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6) + ": " + error.what());
  } catch (...) {
    impl_->faulted = true;
    throw Fault("Non-standard DSP program-fetch failure at P:" + hex(address, 6) +
                " pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6));
  }
}

void DspCore::validate_reference_fetch(const DspProgramFetchStage stage) const {
  if (impl_->active || impl_->faulted || impl_->run.begin) {
    throw Fault("Program fetching requires a healthy unclocked native boundary");
  }
  impl_->native.reference().fetch().check_stage(stage);
}

void DspCore::validate_reference_admission(const DspReferenceTicket &ticket) const {
  validate_reference_ticket(ticket);
  impl_->native.reference().check_admission();
  impl_->check_fetch_opcode();
}

void DspCore::fail_reference_cycle() noexcept {
  impl_->faulted = true;
}

DspReferenceProgress DspCore::advance_reference_access(const DspReferenceTicket &ticket) {
  validate_reference_admission(ticket);
  const auto before = state();
  const ExecutionGuard guard(impl_->active);
  try {
    if (impl_->results.addresses.pending) {
      return reference_progress(impl_->publish_address_phase());
    }
    const auto pending = impl_->native.reference().pending();
    if (pending && std::holds_alternative<DspInstructionRetire>(*pending)) {
      return reference_progress(impl_->retire_reference());
    }
    auto completed = impl_->native.reference().commit();
    impl_->resume_reference();
    return reference_progress(completed);
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("DSP reference admission pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) +
                ": " + error.what());
  } catch (...) {
    impl_->faulted = true;
    throw Fault("Non-standard DSP reference admission failure pc=" + hex(before.pc, 6) +
                " omr=" + hex(before.omr, 6));
  }
}

DspState DspCore::state() const {
  const auto &cpu = impl_->cpu;
  const auto &regs = cpu.regs();
  DspState result{.pc = word(regs.pc),
                  .sr = static_cast<std::uint32_t>(register_value(cpu, dsp56k::Reg_SR)) & word_mask,
                  .omr = word(regs.omr),
                  .sp = word(regs.sp),
                  .vba = word(regs.vba),
                  .stack_pc = word(dsp56k::hiword(regs.ss[cpu.ssIndex()])),
                  .stack_sr = word(dsp56k::loword(regs.ss[cpu.ssIndex()])),
                  .la = word(regs.la),
                  .lc = word(regs.lc),
                  .x = register_value(cpu, dsp56k::Reg_X) & data_mask,
                  .y = register_value(cpu, dsp56k::Reg_Y) & data_mask,
                  .a = register_value(cpu, dsp56k::Reg_A) & accumulator_mask,
                  .b = register_value(cpu, dsp56k::Reg_B) & accumulator_mask};
  for (std::size_t index = 0; index < result.r.size(); ++index) {
    result.r[index] = word(regs.r[index]);
    result.n[index] = word(regs.n[index]);
    result.m[index] = word(regs.m[index]);
  }
  return result;
}

DspDebugState DspCore::debug_state() const {
  const auto &regs = impl_->cpu.regs();
  DspDebugState result{
      .cpu = state(),
      .native_instructions = impl_->cpu.getInstructionCounter(),
      .native_cycles = impl_->cpu.getCycles(),
      .ep = word(regs.ep),
      .sz = word(regs.sz),
      .sc = static_cast<std::uint32_t>(register_value(impl_->cpu, dsp56k::Reg_SC)) & word_mask};
  for (std::size_t index = 0; index < result.stack_high.size(); ++index) {
    result.stack_high[index] = word(dsp56k::hiword(regs.ss[index]));
    result.stack_low[index] = word(dsp56k::loword(regs.ss[index]));
  }
  return result;
}

void DspCore::step() {
  step({});
}

void DspCore::step(DspNativeAccessObserver observer) {
  step_impl(std::move(observer), nullptr);
}

void DspCore::step_admitted(const std::optional<DspInterruptRequest> interrupt) {
  const Admission admission{.interrupt = interrupt};
  step_impl({}, &admission);
}

void DspCore::step_impl(DspNativeAccessObserver observer, const Admission *admission) {
  if (impl_->faulted) { throw Fault("DSP requires reset after a rejected reference step"); }
  if (impl_->native.reference().active() || impl_->native.reference().fetch().active()) {
    throw Fault("Cannot execute an atomic DSP step while a reference access is pending");
  }
  const ExecutionGuard guard(impl_->active);
  const auto before = state();
  const auto before_cycles = impl_->cpu.getCycles();
  std::optional<DspInterruptRequest> interrupt;
  StepEstimate estimate;
  try {
    impl_->invalidate();
    interrupt = impl_->select_interrupt(before, admission);
    impl_->begin_run(before, interrupt.has_value());
    if (interrupt && impl_->run.vector_estimates) {
      estimate = impl_->interrupt_estimate(before, *interrupt);
    }
    const DspNativeAccessScope observed(impl_->native,
                                        impl_->timing_observer(std::move(observer)),
                                        {.pc = before.pc, .omr = before.omr});
    if (interrupt) {
      const auto vector = before.vba + interrupt->vector;
      impl_->access.arm_interrupt(*interrupt,
                                  vector,
                                  estimate.interrupt ? std::optional(estimate.interrupt->words)
                                                     : std::nullopt,
                                  admission != nullptr);
      impl_->cpu.execReferenceInterrupt(vector, interrupt->level);
      impl_->access.verify_interrupt_fetch();
    } else {
      interrupt.reset();
      impl_->cpu.execReference();
    }
    if (impl_->run.complete) {
      const auto after_cycles = impl_->cpu.getCycles();
      if (after_cycles < before_cycles ||
          after_cycles - before_cycles > std::numeric_limits<unsigned>::max()) {
        throw Fault("Invalid native DSP cycle-estimate delta");
      }
      if (estimate.interrupt && after_cycles - before_cycles != estimate.units) {
        throw Fault("DSP native interrupt cost differs from its vector timing preview");
      }
      impl_->run.complete(static_cast<unsigned>(after_cycles - before_cycles));
    }
    impl_->bus.observe_omr(word(impl_->cpu.regs().omr));
  } catch (const std::exception &error) {
    impl_->faulted = true;
    if (impl_->run.failed) { impl_->run.failed(); }
    const auto context = interrupt ? " irq=" + hex(interrupt->vector, 2) +
                                         " level=" + std::to_string(interrupt->level)
                                   : std::string{};
    throw Fault("DSP reference step pc=" + hex(before.pc, 6) + " omr=" + hex(before.omr, 6) +
                context + ": " + error.what());
  }
}

}
