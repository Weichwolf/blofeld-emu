// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_core.hpp"

#include "common/bus.hpp"
#include "dsp/detail/dsp_native_access.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/detail/dsp_reference_execution.hpp"
#include "dsp/dsp_address_phase.hpp"
#include "dsp/dsp_address_stage.hpp"
#include "dsp/dsp_address_stages.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupt_timing.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_loop_source.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_move_writeback.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_peripheral_read.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"
#include "dsp/dsp_peripheral_read_stages.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_repeat_fetch.hpp"
#include "dsp56kEmu/disasm.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp56kEmu/peripherals.h"
#include "dsp56kEmu/registers.h"
#include "dsp56kEmu/types.h"

#include <algorithm>
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
constexpr unsigned ccr_mask = 0xff;
constexpr unsigned address_space = 0x1000000;
constexpr unsigned native_xy_words = 0x18000;
constexpr unsigned spi_mode = 5;
constexpr unsigned reset_core_dma_priority = 0x300;
constexpr unsigned boot_entry = 0xff0000;
constexpr std::uint64_t data_mask = 0xffffffffffff;
constexpr std::uint64_t accumulator_mask = 0xffffffffffffff;

// Only forms whose real native opening and F2 provenance can be retained here.
bool qualified_fetch_loop(const std::uint32_t opcode) {
  constexpr unsigned immediate_mask = 0xff00f0;
  constexpr unsigned immediate_do = 0x060080;
  constexpr unsigned register_mask = 0xffc0ff;
  constexpr unsigned register_do = 0x06c000;
  constexpr unsigned forever_do = 0x000203;
  constexpr unsigned source_mask = 0x3f;
  constexpr unsigned source_sp = 0x3b;
  constexpr unsigned source_ssh = 0x3c;
  constexpr unsigned source_ssl = 0x3d;
  const auto source = (opcode >> 8) & source_mask;
  return relative_loop(opcode) || (opcode & immediate_mask) == immediate_do ||
         opcode == forever_do ||
         ((opcode & register_mask) == register_do && source != source_sp && source != source_ssh &&
          source != source_ssl);
}

DspRepeatControl repeat_control(const dsp56k::DSP::ReferenceAttemptState &state) {
  const auto value = [](const auto field) { return static_cast<std::uint32_t>(field) & word_mask; };
  const auto &repeat = state.repeat;
  return {.pc = value(state.registers.pc),
          .omr = value(state.registers.omr),
          .lc = value(state.registers.lc),
          .remaining = repeat.remaining,
          .body = repeat.body,
          .opcode = repeat.opcode,
          .extension = repeat.extension,
          .next = repeat.next,
          .saved_lc = value(repeat.savedLC),
          .loaded = repeat.loaded};
}

DspLoopArchitecture loop_control(const dsp56k::DSP::ReferenceAttemptState &state) {
  const auto value = [](const auto field) { return static_cast<std::uint32_t>(field) & word_mask; };
  const auto &cpu = state.registers;
  return {.pc = value(cpu.pc),
          .omr = value(cpu.omr),
          .la = value(cpu.la),
          .lc = value(cpu.lc),
          .sp = value(cpu.sp),
          .sc = value(cpu.sc),
          .status = value(cpu.sr) & ~ccr_mask};
}

DspSubroutineControl forecast_subroutine(const DspLoopArchitecture before,
                                         const bool call,
                                         const std::uint32_t next_pc) {
  constexpr unsigned stack_count_mask = 31;
  auto after = before;
  after.pc = next_pc;
  after.sp = call ? before.sp + 1 : before.sp - 1;
  after.sc = (call ? before.sc + 1 : before.sc + stack_count_mask) & stack_count_mask;
  return {.before = before, .after = after};
}

struct StepContext {
  std::uint32_t pc, sr, omr, vba;
};

StepContext step_context(const DspState &state) {
  return {.pc = state.pc, .sr = state.sr, .omr = state.omr, .vba = state.vba};
}

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

struct MoveMerge {
  std::int64_t visible = 0;
  std::int64_t completed = 0;
  std::uint8_t fields = 0;
  bool accumulator = false;
};

std::int64_t merge_move_fields(const MoveMerge input) {
  constexpr std::uint64_t low = 0xffffff;
  constexpr std::uint64_t middle = low << 24U;
  constexpr std::uint64_t high = std::uint64_t{0xff} << 48U;
  // Native DspRegs stores A/B left-aligned; X/Y remain right-aligned.
  constexpr unsigned accumulator_offset = 8;
  const auto offset = input.accumulator ? accumulator_offset : 0;
  const auto mask =
      (((input.fields & 1U) != 0 ? low : 0) | ((input.fields & 2U) != 0 ? middle : 0) |
       ((input.fields & 4U) != 0 ? high : 0))
      << offset;
  return std::bit_cast<std::int64_t>((std::bit_cast<std::uint64_t>(input.visible) & ~mask) |
                                     (std::bit_cast<std::uint64_t>(input.completed) & mask));
}
}

struct DspRetirementResult {
  dsp56k::DSP::ReferenceAttemptState cpu;
  bool redirected = false;
};

struct DspPreviousResult {
  DspRetirementResult result;
  DspInstructionRetire effect;
  std::uint64_t generation = 0;
  std::size_t record = 0;
};

struct DspNativeResultState {
  std::optional<dsp56k::DSP::ReferenceAttemptState> base;
  std::optional<DspRetirementResult> retirement;
  std::optional<dsp56k::DSP::ReferenceAttemptState> visible;
  std::optional<DspPreviousResult> previous;
  DspAddressPhase addresses;
  DspMoveWritebackState writeback;
  std::array<std::optional<std::uint32_t>, 8> pipeline_addresses{};
};

struct DspCoreCheckpoint::Storage {
  Storage(dsp56k::DSP::ReferenceState state,
          DspReferenceState reference_state,
          const std::uint64_t serial,
          const DspNativeResultState &result,
          const DspAddressStages &address_stages,
          DspPeripheralReadStages read_stages,
          const bool held)
      : cpu(std::move(state)),
        reference(std::move(reference_state)),
        generation(serial),
        results(result),
        stages(address_stages),
        peripheral_reads(std::move(read_stages)),
        reset_held(held) {}

  const dsp56k::DSP::ReferenceState cpu;
  const DspReferenceState reference;
  const std::uint64_t generation;
  const DspNativeResultState results;
  const DspAddressStages stages;
  const DspPeripheralReadStages peripheral_reads;
  const bool reset_held;
};

DspCoreCheckpoint::DspCoreCheckpoint(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

DspCoreCheckpoint::~DspCoreCheckpoint() = default;

namespace {
DspFetchedWord capture_read_word(const DspProgramFetch &fetch,
                                 const DspPeripheralReadPoint point,
                                 const std::uint64_t occurrence,
                                 const bool automatic) {
  if (automatic) { return fetch.select(point.address, occurrence); }
  const auto head = fetch.queued(0);
  if (!head || head->word.address != point.address) {
    throw Fault("Explicit peripheral capture lost its next actual F2 occurrence");
  }
  return *head;
}

template <typename Stages>
auto &peripheral_record(Stages &stages, const DspPeripheralReadPosition &position) {
  if (position.slot >= stages.records.size()) { throw Fault("Invalid peripheral response slot"); }
  auto &record = stages.records.at(position.slot);
  if (!record || record->serial != position.serial || record->origin != position.origin) {
    throw Fault("Stale, foreign or discarded-future peripheral response");
  }
  return *record;
}
}

class DspCore::Impl {
public:
  explicit Impl(DspBus &board)
      : bus(board),
        native(board),
        access(board, native),
        memory(access, dsp56k::Memory::BusOwned{}, address_space, native_xy_words),
        peripherals_x(native, DspSpace::x),
        peripherals_y(native, DspSpace::y),
        cpu(memory, &peripherals_x, &peripherals_y) {
    static_assert(!dsp56k::g_useJIT);
    static_assert(std::is_trivially_copyable_v<dsp56k::DSP::ReferenceAttemptState>);
    static_assert(sizeof(dsp56k::DSP::ReferenceAttemptState) <= 1024);
  }

  [[nodiscard]] bool has_peripheral_reads() const {
    return std::ranges::any_of(peripheral_reads.records,
                               [](const auto &record) { return record.has_value(); });
  }

  [[nodiscard]] std::uint64_t peripheral_epoch() const {
    const auto epoch = bus.last_dma_cycle();
    if (!epoch || bus.has_active_dma()) {
      throw Fault("Unqualified peripheral-read cycle/DMA state pc=" + hex(word(cpu.regs().pc), 6));
    }
    return *epoch;
  }

  void validate_peripheral_capture(const DspPeripheralReadPoint point) const {
    const auto &reference = native.reference();
    const auto &pending_effect = reference.pending();
    if (reference.fetch().view().consume_address != point.address || !pending_effect) {
      throw Fault("Peripheral capture is not the next actual F2 boundary pc=" +
                  hex(point.address, 6));
    }
    const auto &pending = *pending_effect;
    if (const auto *retirement = std::get_if<DspInstructionRetire>(&pending)) {
      if (!retirement->redirected && retirement->next_pc == point.address &&
          retirement->omr == retirement->next_omr) {
        return;
      }
    } else if (const auto *request = std::get_if<DspNativeAccess>(&pending)) {
      if (request->kind == DspNativeAccessKind::opcode && request->pc == point.address) { return; }
      const auto opcode = reference.committed_access(DspNativeAccessKind::opcode);
      const auto form =
          opcode && opcode->value ? decode_memory_stage(*opcode->value) : std::nullopt;
      if (request->kind == DspNativeAccessKind::write && form && form->peripheral) {
        const auto words = form->peripheral_register ? 1U : 2U;
        if (((request->pc + words) & word_mask) == point.address) { return; }
      }
    }
    throw Fault("Unqualified unresolved control flow before peripheral capture pc=" +
                hex(point.address, 6));
  }

  void validate_peripheral_admission(const std::optional<std::uint64_t> cycle) const {
    if (!has_peripheral_reads()) { return; }
    const auto &pending = native.reference().pending();
    const auto *request = pending ? std::get_if<DspNativeAccess>(&*pending) : nullptr;
    if (request == nullptr) { return; }
    if (request->kind == DspNativeAccessKind::read || request->kind == DspNativeAccessKind::bit) {
      for (const auto &record : peripheral_reads.records) {
        if (record && record->point.address == request->pc) {
          throw Fault("Native peripheral read requires its owned sampled response at " +
                      hex(request->target.address, 6) + " pc=" + hex(request->pc, 6));
        }
      }
      return;
    }
    constexpr unsigned short_io_begin = 0xffff80;
    if (request->kind != DspNativeAccessKind::write || request->target.space == DspSpace::p ||
        request->target.address < short_io_begin) {
      return;
    }
    const auto epoch = cycle ? *cycle : peripheral_epoch();
    const auto opcode = native.reference().committed_access(DspNativeAccessKind::opcode);
    const auto form = opcode && opcode->value ? decode_memory_stage(*opcode->value) : std::nullopt;
    if (!form || !form->peripheral ||
        (peripheral_reads.last_store_cycle && epoch <= *peripheral_reads.last_store_cycle)) {
      throw Fault("Unqualified native store/response overlap pc=" + hex(request->pc, 6));
    }
    constexpr unsigned read_to_execute = 2;
    for (const auto &record : peripheral_reads.records) {
      if (record &&
          (epoch < record->point.epoch || epoch - record->point.epoch > read_to_execute)) {
        throw Fault("Expired peripheral sample before native write pc=" + hex(request->pc, 6));
      }
    }
  }

  void observe_peripheral_store(const DspReferenceEffect &completed) {
    const auto *event = std::get_if<DspNativeAccess>(&completed);
    constexpr unsigned short_io_begin = 0xffff80;
    if (event == nullptr || event->kind != DspNativeAccessKind::write ||
        event->target.space == DspSpace::p || event->target.address < short_io_begin) {
      return;
    }
    const auto epoch = bus.last_dma_cycle();
    if (!epoch) { return; }
    if (has_peripheral_reads()) {
      const auto opcode = native.reference().committed_access(DspNativeAccessKind::opcode);
      if (!opcode || !opcode->value) { throw Fault("Native store lost its actual opcode receipt"); }
      for (auto &record : peripheral_reads.records) {
        if (record) {
          static_cast<void>(record->sample.forward(
              {.address = opcode->target.address, .value = *opcode->value}, *event, *epoch));
        }
      }
    }
    peripheral_reads.last_store_cycle = epoch;
  }

  void invalidate() {
    const auto changes = bus.take_code_changes();
    if (changes.all) {
      cpu.clearOpcodeCache();
    } else {
      for (const auto address : changes.addresses) { cpu.clearOpcodeCache(address); }
    }
  }

  [[nodiscard]] StepContext current_step_context() const {
    const auto &regs = cpu.regs();
    return {.pc = word(regs.pc),
            .sr = static_cast<std::uint32_t>(register_value(cpu, dsp56k::Reg_SR)) & word_mask,
            .omr = word(regs.omr),
            .vba = word(regs.vba)};
  }

  void begin_run(const DspState before, const bool interrupt) const {
    if (run.begin) {
      const auto units = run.fetched && !interrupt ? cpu.referenceNextCycles() : 0;
      const auto opcode =
          run.timing_opcode && !interrupt ? std::optional(cpu.referenceNextOpcode()) : std::nullopt;
      run.begin(before, interrupt, units, opcode);
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

  [[nodiscard]] StepEstimate interrupt_estimate(const StepContext &before,
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
  select_interrupt(const StepContext &before, const Admission *admission) const {
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
    stages = {};
    peripheral_reads = {};
    reference_owner = std::move(next_owner);
    generation = 0;
  }

  void restore_reference_base() {
    if (!results.base) { throw Fault("Native reference instruction lost its base state"); }
    const DspReferenceScope restoring(native.reference(), DspReferenceMode::restoring);
    cpu.restoreReferenceAttempt(*results.base);
  }

  void restore_visible_reference() {
    if (results.visible) {
      const DspReferenceScope restoring(native.reference(), DspReferenceMode::restoring);
      cpu.restoreReferenceAttempt(*results.visible);
    } else {
      restore_reference_base();
    }
    for (unsigned index = 0; index < results.addresses.published.size(); ++index) {
      if (const auto value = results.addresses.published.at(index)) {
        cpu.regs().r.at(index).var = static_cast<std::int32_t>(*value);
      }
    }
    apply_move_fields(results.writeback.published);
    apply_pipeline_addresses();
  }

  void apply_pipeline_addresses() {
    for (unsigned index = 0; index < results.pipeline_addresses.size(); ++index) {
      if (const auto value = results.pipeline_addresses.at(index)) {
        cpu.regs().r.at(index).var = static_cast<std::int32_t>(*value);
      }
    }
  }

  DspRepeatControl repeat_entry(const DspReferenceExecution contract,
                                const DspAddressRecord *const address) const {
    const auto repeat = repeat_control(cpu.referenceAttemptState());
    if (repeat.remaining != 0 && contract == DspReferenceExecution::staged_register_boundaries) {
      native.reference().fetch().verify_repeat(repeat);
      if (repeat.loaded && decode_memory_stage(repeat.opcode) && address == nullptr) {
        throw Fault("Native repeated memory E1 requires its actual AG1 record read P:" +
                    hex(repeat.body, 6) + " pc=" + hex(repeat.pc, 6) +
                    " omr=" + hex(repeat.omr, 6));
      }
    }
    return repeat;
  }

  void bind_memory_address(const DspAddressRecord &record) {
    const auto visible = cpu.referenceAttemptState();
    results.visible = visible;
    if (!results.previous) { results.base = visible; }
    if (!results.base) { throw Fault("Native memory E1 lost its actual computation base"); }
    auto &base = results.base->registers;
    const auto &source = record.source.registers;
    const auto bind = [&](const DspAddressUpdate update) {
      const auto index = update.index;
      base.r.at(index) = source.r.at(index);
      base.n.at(index) = source.n.at(index);
      base.m.at(index) = source.m.at(index);
      base.mMask.at(index) = source.mMask.at(index);
      base.mModulo.at(index) = source.mModulo.at(index);
      results.addresses.published.at(index) = update.value;
    };
    if (record.view.kind == DspAddressStageKind::register_agu) {
      bind(record.view.update);
      if (record.view.secondary) { bind(record.view.secondary->update); }
    }
    for (unsigned reg = 0; reg < visible.registers.r.size(); ++reg) {
      results.pipeline_addresses.at(reg) =
          static_cast<std::uint32_t>(visible.registers.r.at(reg)) & word_mask;
    }
  }

  DspReferenceProgress memory_entry(DspCore &core, const DspAddressRecord &record) {
    const auto &view = record.view;
    const auto repeat = repeat_control(cpu.referenceAttemptState());
    const bool loaded = repeat.remaining != 0 && repeat.loaded;
    auto pending = core.reference_pending();
    if (!pending) {
      pending = core.begin_reference_step(
                        DspReferenceExecution::staged_register_boundaries, false, &record)
                    .pending;
    }
    if (!pending) { throw Fault("Native memory E1 lost its actual boundary"); }
    record.fetched.check_front(native.reference().fetch());
    DspReferenceProgress progress{.pending = pending};
    if (!loaded) {
      const auto *opcode = std::get_if<DspNativeAccess>(&pending->effect);
      if (opcode == nullptr || opcode->kind != DspNativeAccessKind::opcode ||
          opcode->target.address != view.word.address ||
          pending->latched_program_word != view.word.value) {
        throw Fault("Native memory E1 does not match its captured F2 word");
      }
      bind_memory_address(record);
      progress = core.advance_reference_access(pending->ticket);
    } else if (native.reference().opcode() != view.word.value) {
      throw Fault("Repeated memory E1 differs from its native-owned opcode latch");
    }
    return progress;
  }

  static DspNativeAccess commit_addressed_data(DspCore &core,
                                               DspReferenceProgress &progress,
                                               const DspMemoryAccess expected) {
    if (!progress.pending) { throw Fault("Native memory E1 lost its actual data access"); }
    const auto *data = std::get_if<DspNativeAccess>(&progress.pending->effect);
    const auto kind = expected.operation == DspMemoryOperation::read ? DspNativeAccessKind::read
                                                                     : DspNativeAccessKind::write;
    if (data == nullptr || data->kind != kind || data->target.space != expected.target.space ||
        data->target.address != expected.target.address) {
      throw Fault("Native memory E1 differs from the actual AG1 address");
    }
    progress = core.advance_reference_access(progress.pending->ticket);
    if (!progress.completed) { throw Fault("Native memory E1 lost its committed data effect"); }
    return std::get<DspNativeAccess>(*progress.completed);
  }

  void apply_move_fields(const DspMoveFields fields) {
    if (fields == DspMoveFields{}) { return; }
    if (!results.retirement) {
      throw Fault("Native writeback lost its actual completed CPU result");
    }
    const auto &completed = results.retirement->cpu.registers;
    auto &regs = cpu.regs();
    regs.x.var =
        merge_move_fields({.visible = regs.x.var, .completed = completed.x, .fields = fields.x});
    regs.y.var =
        merge_move_fields({.visible = regs.y.var, .completed = completed.y, .fields = fields.y});
    regs.a.var = merge_move_fields(
        {.visible = regs.a.var, .completed = completed.a, .fields = fields.a, .accumulator = true});
    regs.b.var = merge_move_fields(
        {.visible = regs.b.var, .completed = completed.b, .fields = fields.b, .accumulator = true});
    for (unsigned index = 0; index < regs.r.size(); ++index) {
      const auto bit = 1U << index;
      if ((fields.r & bit) != 0) { regs.r.at(index).var = completed.r.at(index); }
      if ((fields.n & bit) != 0) { regs.n.at(index).var = completed.n.at(index); }
      if ((fields.m & bit) != 0) {
        regs.m.at(index).var = completed.m.at(index);
        regs.mMask.at(index) = completed.mMask.at(index);
        regs.mModulo.at(index) = completed.mModulo.at(index);
      }
    }
  }

  DspMoveWriteback publish_move_writeback() {
    if (!results.writeback.pending) { throw Fault("Native move-writeback lost its held fields"); }
    const auto completed = *results.writeback.pending;
    apply_move_fields(completed.fields);
    for (unsigned index = 0; index < results.pipeline_addresses.size(); ++index) {
      if ((completed.fields.r & (1U << index)) != 0 && results.pipeline_addresses.at(index)) {
        results.pipeline_addresses.at(index) = word(cpu.regs().r.at(index));
      }
    }
    results.writeback.published = completed.fields;
    results.writeback.pending.reset();
    return completed;
  }

  DspMoveFields move_writeback_fields(const DspNativeAccess *request = nullptr,
                                      const DspMoveDecode scope = DspMoveDecode::all) const {
    const auto &reference = native.reference();
    if (reference.execution() != DspReferenceExecution::staged_register_boundaries) { return {}; }
    const auto opcode = reference.opcode();
    const auto context = reference.snapshot().context;
    try {
      if (!opcode) { throw Fault("Native move-writeback lost its admitted opcode"); }
      // Control writes remain in the actual native retirement result. They do
      // not publish a fictitious ordinary register field at the move boundary.
      if (decode_omr_move(*opcode)) { return {}; }
      return decode_move_writeback(*opcode, scope);
    } catch (const std::exception &error) {
      std::string operation;
      if (request != nullptr) {
        constexpr std::array spaces{"P:", "X:", "Y:"};
        operation = std::string(request->kind == DspNativeAccessKind::write ? " before write "
                                                                            : " before read ") +
                    spaces.at(static_cast<unsigned>(request->target.space)) +
                    hex(request->target.address, 6);
      }
      throw Fault("Native move-writeback" + operation + " pc=" + hex(context.pc, 6) +
                  " omr=" + hex(context.omr, 6) + ": " + error.what());
    }
  }

  void validate_peripheral_address_owner(const DspNativeAccess &request) const {
    const bool owned = std::ranges::any_of(peripheral_reads.records, [&](const auto &record) {
      return record && record->automatic && record->point.address == request.pc;
    });
    if (!owned) {
      throw Fault("Unqualified native memory-address phase read " +
                  std::string(request.target.space == DspSpace::x ? "X:" : "Y:") +
                  hex(request.target.address, 6) + " pc=" + hex(request.pc, 6) +
                  " omr=" + hex(request.omr, 6));
    }
  }

  void capture_address_phase() {
    auto &reference = native.reference();
    if (reference.execution() != DspReferenceExecution::address_and_retirement_boundaries &&
        reference.execution() != DspReferenceExecution::staged_register_boundaries) {
      return;
    }
    const auto &pending = reference.pending();
    if (!pending) { throw Fault("Native address phase lost its real access"); }
    const auto *request = std::get_if<DspNativeAccess>(&*pending);
    if (request == nullptr || request->kind == DspNativeAccessKind::opcode ||
        request->kind == DspNativeAccessKind::extension) {
      return;
    }
    const auto admitted_opcode = reference.opcode();
    const auto port = admitted_opcode ? decode_peripheral_read(*admitted_opcode) : std::nullopt;
    if (port) { validate_peripheral_address_owner(*request); }
    const bool port_bit = request->kind == DspNativeAccessKind::bit && port && port->bit &&
                          request->bit && request->bit->operation == DspBitOperation::test;
    if (request->kind != DspNativeAccessKind::read && request->kind != DspNativeAccessKind::write &&
        !port_bit) {
      throw Fault("Unqualified native address-phase access pc=" + hex(request->pc, 6) +
                  " address=" + hex(request->target.address, 6) + " omr=" + hex(request->omr, 6));
    }
    const auto opcode = reference.opcode();
    if (!opcode || !results.base) {
      throw Fault("Native address phase lost its fetched instruction base");
    }
    static_cast<void>(move_writeback_fields(request));
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

  void check_overlap() const {
    const auto &reference = native.reference();
    const auto &pending = reference.pending();
    if (reference.execution() != DspReferenceExecution::staged_register_boundaries ||
        !results.retirement || !results.base || results.previous || results.addresses.pending ||
        results.writeback.pending || !pending ||
        !std::holds_alternative<DspInstructionRetire>(*pending)) {
      throw Fault("Native overlap requires one actual completed staged result");
    }
    const auto effect = std::get<DspInstructionRetire>(*pending);
    const auto &next = results.retirement->cpu;
    constexpr unsigned forbidden_status = 0x1a000;
    constexpr unsigned condition_bits = 0xff;
    const auto status = static_cast<std::uint32_t>(next.registers.sr);
    const auto before_status = static_cast<std::uint32_t>(results.base->registers.sr);
    if (reference.fetch().pending_context() || effect.redirected || effect.omr != effect.next_omr ||
        next.repeat.remaining != 0 || (status & forbidden_status) != 0 ||
        ((status ^ before_status) & ~condition_bits) != 0) {
      throw Fault("Unqualified native overlap control/mode/REP transition pc=" + hex(effect.pc, 6) +
                  " omr=" + hex(effect.omr, 6));
    }
    const auto fetch = reference.fetch().view();
    if (reference.fetch().active() &&
        (fetch.consume_address != effect.next_pc || fetch.omr != effect.next_omr)) {
      throw Fault("Native overlap lost its actual sequential fetch cursor");
    }
    constexpr unsigned mask_shift = 8;
    constexpr unsigned interrupt_mask = 3;
    if (bus.interrupt_request((status >> mask_shift) & interrupt_mask)) {
      throw Fault("Unqualified native overlap interrupt pc=" + hex(effect.next_pc, 6) +
                  " omr=" + hex(effect.next_omr, 6));
    }
  }

  void begin_overlap() {
    auto &reference = native.reference();
    const auto pending = reference.pending();
    if (!pending || !results.retirement) {
      throw Fault("Native overlap lost its actual predecessor result");
    }
    const auto effect = std::get<DspInstructionRetire>(*pending);
    const auto computation = results.retirement->cpu;
    const auto visible = cpu.referenceAttemptState();
    const DspPreviousResult previous{.result = *results.retirement,
                                     .effect = effect,
                                     .generation = generation,
                                     .record = reference.pending_index()};
    reference.finish();
    results = {};
    results.base = computation;
    results.visible = visible;
    results.previous = previous;
    ++generation;
    reference.begin({.pc = effect.next_pc, .omr = effect.next_omr},
                    DspReferenceExecution::staged_register_boundaries);
    resume_reference();
  }

  DspInstructionRetire publish_previous() {
    if (!results.previous) { throw Fault("Native overlap lost its prior result"); }
    const auto previous = *results.previous;
    results.visible = previous.result.cpu;
    results.previous.reset();
    restore_visible_reference();
    // The younger computation already contains this exact completed predecessor.
    bus.observe_omr(previous.effect.next_omr);
    return previous.effect;
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
    const bool repeat =
        fetch_repeat_prefix(*value) &&
        (reference.execution() == DspReferenceExecution::access_and_retirement_boundaries ||
         reference.execution() == DspReferenceExecution::staged_register_boundaries) &&
        (word(cpu.regs().sr) & 0x1a000U) == 0 &&
        fetch_repeat_count(
            {.opcode = *value,
             .register_count =
                 *value == dsp_repeat_from_a1
                     ? static_cast<std::uint32_t>(register_value(cpu, dsp56k::Reg_A1)) & word_mask
                     : static_cast<std::uint32_t>(cpu.regs().x.var) & word_mask}) != 0;
    if (reference.fetch().repeating() && !fetch_repeat_body(*value)) {
      throw Fault("Unqualified REP body read P:" + hex(request->target.address, 6) + " opcode=" +
                  hex(*value, 6) + " pc=" + hex(request->pc, 6) + " omr=" + hex(request->omr, 6));
    }
    const bool loop = qualified_fetch_loop(*value) &&
                      reference.execution() != DspReferenceExecution::access_boundaries;
    if (((flags & dsp56k::OpFlagLoop) != 0 && !loop && !repeat) ||
        (flags & dsp56k::OpFlagCacheMod) != 0 || instruction == dsp56k::Stop ||
        instruction == dsp56k::Wait || instruction == dsp56k::Trap ||
        instruction == dsp56k::Trapcc || instruction == dsp56k::Debug ||
        instruction == dsp56k::Debugcc || instruction == dsp56k::Illegal ||
        instruction == dsp56k::Reset) {
      throw Fault("Unqualified native program-fetch control opcode at P:" +
                  hex(request->target.address, 6) + " pc=" + hex(request->pc, 6) +
                  " omr=" + hex(request->omr, 6));
    }
  }

  void capture_repeat_opening() {
    auto &reference = native.reference();
    const auto opcode = reference.opcode();
    if (!reference.fetch().active() || !opcode || !fetch_repeat_prefix(*opcode)) { return; }
    if (!results.base || !results.retirement ||
        (reference.execution() != DspReferenceExecution::access_and_retirement_boundaries &&
         reference.execution() != DspReferenceExecution::staged_register_boundaries)) {
      throw Fault("Native REP opening lost its actual held result");
    }
    reference.fetch().stage_repeat(repeat_control(*results.base),
                                   repeat_control(results.retirement->cpu));
  }

  void capture_omr_context() {
    auto &reference = native.reference();
    auto &fetch = reference.fetch();
    const auto opcode = reference.opcode();
    if (!fetch.active() || !opcode || !decode_omr_move(*opcode) ||
        reference.execution() != DspReferenceExecution::staged_register_boundaries) {
      return;
    }
    const auto &pending = reference.pending();
    const auto *effect = pending ? std::get_if<DspInstructionRetire>(&*pending) : nullptr;
    if (effect == nullptr) { throw Fault("OMR context lost its actual native result"); }
    fetch.stage_context(*effect, bus);
    const auto &context = fetch.context();
    if (!context) { throw Fault("OMR context lost its F2 writer"); }
    for (auto &record : peripheral_reads.records) {
      if (record) { record->sample.follow_context(*context); }
    }
  }

  void finish_fetch(const bool redirected) {
    auto &reference = native.reference();
    auto &fetch = reference.fetch();
    if (!fetch.active()) { return; }
    const auto next = word(cpu.regs().pc);
    const auto omr = word(cpu.regs().omr);
    const auto before = reference.snapshot().context;
    if (fetch.confirm_context(
            {.pc = before.pc, .next_pc = next, .omr = omr, .redirected = redirected}, bus)) {
      return;
    }
    if (fetch.confirm_repeat(
            {.pc = before.pc, .next_pc = next, .omr = omr, .redirected = redirected},
            repeat_control(cpu.referenceAttemptState()))) {
      return;
    }
    const auto control = loop_control(cpu.referenceAttemptState());
    if (fetch.confirm_opening(control, redirected) || fetch.confirm_loop(control, redirected)) {
      return;
    }
    if (fetch.confirm_subroutine(
            {.pc = before.pc, .next_pc = next, .omr = omr, .redirected = redirected}, control)) {
      return;
    }
    constexpr unsigned loop_cache_or_compatibility = 0x1a000;
    if ((word(cpu.regs().sr) & loop_cache_or_compatibility) != 0 &&
        !fetch.accepts_loop_control(control)) {
      throw Fault("Unqualified native program-fetch mode change pc=" +
                  hex(reference.snapshot().context.pc, 6) + " omr=" + hex(omr, 6));
    }
    if (fetch.confirm_branch(reference.snapshot().context.pc, next, omr, redirected)) { return; }
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
    apply_pipeline_addresses();
    finish_reference(redirected);
    return completed;
  }

  void capture_loop_opening() {
    auto &reference = native.reference();
    const auto opcode_word = reference.opcode();
    if (!reference.fetch().active() || !opcode_word || !qualified_fetch_loop(*opcode_word)) {
      return;
    }
    if (!results.base || !results.retirement) {
      throw Fault("Native DO lost its actual held CPU result");
    }
    const auto opcode = reference.committed_access(DspNativeAccessKind::opcode);
    const auto extension = reference.committed_access(DspNativeAccessKind::extension);
    const auto pair = reference.fetch().consumed_pair();
    if (!opcode || !extension || !opcode->value || !extension->value || opcode->ordinal != 0 ||
        extension->ordinal != 1 || pair[0].word.address != opcode->target.address ||
        pair[0].word.value != *opcode->value || pair[1].word.address != extension->target.address ||
        pair[1].word.value != *extension->value || pair[0].occurrence == 0 ||
        pair[1].occurrence == 0) {
      throw Fault("Native DO differs from its actual consumed F2 pair");
    }
    reference.fetch().open_loop({.word = pair[0].word,
                                 .extension = pair[1].word,
                                 .before = loop_control(*results.base),
                                 .after = loop_control(results.retirement->cpu)},
                                pair[0],
                                pair[1]);
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
        reference.execution() == DspReferenceExecution::address_and_retirement_boundaries ||
        reference.execution() == DspReferenceExecution::staged_register_boundaries) {
      const auto before = reference.snapshot().context;
      DspMoveFields fields;
      try {
        fields = move_writeback_fields();
      } catch (...) {
        restore_visible_reference();
        throw;
      }
      reference.stage_retirement({.pc = before.pc,
                                  .next_pc = word(cpu.regs().pc),
                                  .omr = before.omr,
                                  .next_omr = word(cpu.regs().omr),
                                  .redirected = redirected});
      results.retirement = DspRetirementResult{.cpu = result, .redirected = redirected};
      if (fields != DspMoveFields{}) {
        const auto opcode = reference.opcode();
        if (!opcode) { throw Fault("Native move-writeback lost its captured opcode"); }
        results.writeback.pending = DspMoveWriteback{
            .fields = fields, .opcode = *opcode, .pc = before.pc, .omr = before.omr};
      }
      try {
        capture_repeat_opening();
        capture_omr_context();
        capture_loop_opening();
      } catch (...) {
        restore_visible_reference();
        throw;
      }
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
  bool reference_reset = false;
  bool active = false;
  RunHooks run;
  DspNativeResultState results;
  DspAddressStages stages;
  DspPeripheralReadStages peripheral_reads;
  std::shared_ptr<const bool> reference_owner;
  std::uint64_t generation = 0;
};

DspCore::DspCore(DspBus &bus) : impl_(std::make_unique<Impl>(bus)) {
  reset_spi_boot();
}

DspCore::~DspCore() = default;

void DspCore::reset_spi_boot() {
  require_reset_released();
  const ExecutionGuard guard(impl_->active);
  if (impl_->run.reset) { impl_->run.reset(); }
  impl_->faulted = true;
  impl_->reset_reference();
  impl_->bus.reset();
  impl_->cpu.resetHW();
  // DSP56300FM2.3.3 clears SR[7:0]; the SDK reset leaves CCR[7:4] behind.
  impl_->cpu.regs().sr.var = static_cast<std::int32_t>(word(impl_->cpu.regs().sr) & ~ccr_mask);
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

bool DspCore::reference_reset_active() const {
  return impl_->reference_reset;
}

void DspCore::set_reference_reset(const bool held) {
  if (impl_->active || held == impl_->reference_reset) {
    throw Fault("Native reference reset requires an inactive gate transition");
  }
  impl_->reference_reset = held;
}

void DspCore::require_reset_released() const {
  if (impl_->reference_reset) {
    const auto context = state();
    throw Fault("DSP operation during hardware reset pc=" + hex(context.pc, 6) +
                " omr=" + hex(context.omr, 6));
  }
}

bool DspCore::faulted() const {
  return impl_->faulted;
}

std::uint64_t DspCore::native_cycles() const {
  return impl_->cpu.getCycles();
}

DspCore::StepEstimate DspCore::next_step_cycles(const bool vector_estimates,
                                                const Admission *admission,
                                                const bool timing_opcode) const {
  if (impl_->active || impl_->faulted || impl_->native.reference().active()) {
    throw Fault("DSP cycle preview requires a healthy completed step");
  }
  const auto before = state();
  try {
    constexpr unsigned compatibility = 0x2000;
    const auto interrupt = impl_->select_interrupt(step_context(before), admission);
    if ((before.sr & compatibility) != 0 || (interrupt && !vector_estimates)) {
      throw Fault("Unqualified DSP interrupt or compatibility-mode timing preview");
    }
    if (interrupt) { return impl_->interrupt_estimate(step_context(before), *interrupt); }
    const auto word =
        timing_opcode ? std::optional(impl_->cpu.referenceNextOpcode()) : std::nullopt;
    return {.units = impl_->cpu.referenceNextCycles(), .opcode = word, .interrupt = std::nullopt};
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
                                                         impl_->results,
                                                         impl_->stages,
                                                         impl_->peripheral_reads,
                                                         impl_->reference_reset));
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
  impl_->stages = checkpoint.storage_->stages;
  impl_->peripheral_reads = checkpoint.storage_->peripheral_reads;
  impl_->generation = checkpoint.storage_->generation;
  impl_->reference_reset = checkpoint.storage_->reset_held;
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
  const auto &move = impl_->results.writeback.pending;
  auto effect = *pending;
  if (address) {
    effect = *address;
  } else if (move) {
    effect = *move;
  }
  return DspReferencePending{.effect = effect,
                             .ticket = DspReferenceTicket(impl_->reference_owner,
                                                          {.generation = impl_->generation,
                                                           .record = reference.pending_index(),
                                                           .address = address.has_value(),
                                                           .move = move.has_value()}),
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

void DspCore::validate_reference_overlap(const DspReferenceTicket &ticket) const {
  validate_reference_ticket(ticket);
  impl_->check_overlap();
  if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("Native overlap exhausted its instruction generations");
  }
}

DspReferenceProgress DspCore::overlap_reference_instruction(const DspReferenceTicket &ticket) {
  validate_reference_overlap(ticket);
  const ExecutionGuard guard(impl_->active);
  try {
    impl_->invalidate();
    impl_->begin_overlap();
    return reference_progress();
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("Native overlap: " + std::string(error.what()));
  }
}

std::optional<DspReferencePending> DspCore::reference_retirement() const {
  if (impl_->active || impl_->faulted) { throw Fault("Native overlap owner is unavailable"); }
  const auto &previous = impl_->results.previous;
  if (!previous) { return std::nullopt; }
  return DspReferencePending{
      .effect = previous->effect,
      .ticket = DspReferenceTicket(
          impl_->reference_owner,
          {.generation = previous->generation, .record = previous->record, .previous = true}),
      .execution = DspReferenceExecution::staged_register_boundaries};
}

void DspCore::validate_reference_retirement(const DspReferenceTicket &ticket) const {
  const auto pending = reference_retirement();
  if (!pending || ticket != pending->ticket) {
    throw Fault("Stale, foreign or unavailable previous native retirement ticket");
  }
}

DspInstructionRetire DspCore::retire_reference_overlap(const DspReferenceTicket &ticket) {
  validate_reference_retirement(ticket);
  const ExecutionGuard guard(impl_->active);
  try {
    return impl_->publish_previous();
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault("Native prior retirement: " + std::string(error.what()));
  }
}

DspReferenceProgress DspCore::begin_reference_interrupt(const DspReferenceExecution contract) {
  return begin_reference_step(contract, true);
}

DspReferenceProgress DspCore::begin_reference_step(const DspReferenceExecution contract,
                                                   const bool interrupt_entry,
                                                   const DspAddressRecord *const address) {
  require_reset_released();
  if (impl_->active || impl_->faulted || impl_->native.reference().active() ||
      (contract != DspReferenceExecution::access_boundaries &&
       contract != DspReferenceExecution::access_and_retirement_boundaries &&
       contract != DspReferenceExecution::address_and_retirement_boundaries &&
       contract != DspReferenceExecution::staged_register_boundaries) ||
      impl_->run.begin || impl_->run.complete || impl_->run.failed || impl_->run.reset ||
      impl_->run.ready) {
    throw Fault(
        "Native access suspension requires an explicit healthy unclocked instruction boundary");
  }
  const auto before = state();
  const auto repeat = impl_->repeat_entry(contract, address);
  if ((contract == DspReferenceExecution::address_and_retirement_boundaries ||
       contract == DspReferenceExecution::staged_register_boundaries) &&
      (interrupt_entry ||
       (repeat.remaining != 0 && contract != DspReferenceExecution::staged_register_boundaries))) {
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
    if (address != nullptr) { impl_->bind_memory_address(*address); }
    if (repeat.remaining != 0 && repeat.loaded && impl_->native.reference().fetch().active()) {
      const auto captured = impl_->native.reference().fetch().repeat_body();
      if (!captured) { throw Fault("Loaded REP lost its actual opcode receipt"); }
      impl_->native.reference().bind_repeat_opcode(*captured);
    }
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
      ticket.position_.address != impl_->results.addresses.pending.has_value() ||
      ticket.position_.move != impl_->results.writeback.pending.has_value() ||
      ticket.position_.previous) {
    throw Fault("Stale, foreign or unavailable native reference access ticket");
  }
}

DspReferencePending DspCore::latch_reference_program(const DspReferenceTicket &ticket) {
  validate_reference_ticket(ticket);
  static_cast<void>(impl_->native.reference().check_program_latch());
  const auto before = impl_->native.reference().snapshot().context;
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

std::optional<DspFetchedWord> DspCore::pipeline_fetched_word(const std::size_t index) const {
  if (impl_->active || impl_->faulted || impl_->run.begin) {
    throw Fault("Unavailable actual pipeline F2 receipt pc=" + hex(state().pc, 6));
  }
  return impl_->native.reference().fetch().queued(index);
}

DspProgramFetchView DspCore::advance_reference_fetch(const DspProgramFetchStage stage) {
  validate_reference_fetch(stage);
  auto &fetch = impl_->native.reference().fetch();
  const auto visible = state();
  const auto before = impl_->native.reference().active()
                          ? impl_->native.reference().snapshot().context
                          : DspNativeAccess{.pc = visible.pc, .omr = visible.omr};
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

bool DspCore::pipeline_interrupt_deferred() const {
  static_cast<void>(pipeline_fetched_word(0));
  const auto native = repeat_control(impl_->cpu.referenceAttemptState());
  const auto &fetch = impl_->native.reference().fetch();
  if (native.remaining != 0) { fetch.verify_repeat(native); }
  const auto opcode = fetch.peek(native.pc);
  if (opcode && !impl_->cpu.referenceInterruptible(*opcode)) { return true; }
  const auto held = pipeline_repeat_result();
  return held && held->remaining != 0;
}

std::optional<DspRepeatControl> DspCore::pipeline_repeat_control() const {
  static_cast<void>(pipeline_fetched_word(0));
  return impl_->native.reference().fetch().repeat_control();
}

std::optional<DspFetchedWord> DspCore::pipeline_repeat_body() const {
  static_cast<void>(pipeline_fetched_word(0));
  return impl_->native.reference().fetch().repeat_body();
}

std::optional<DspRepeatControl> DspCore::pipeline_repeat_result() const {
  static_cast<void>(pipeline_fetched_word(0));
  const auto &reference = impl_->native.reference();
  const auto &result = impl_->results.retirement;
  const auto pending = reference.pending();
  const auto *effect = pending ? std::get_if<DspInstructionRetire>(&*pending) : nullptr;
  if (!result || effect == nullptr) { return std::nullopt; }
  const auto projection = repeat_control(result->cpu);
  return reference.fetch().owns_repeat_result({.pc = effect->pc,
                                               .next_pc = effect->next_pc,
                                               .omr = effect->next_omr,
                                               .redirected = effect->redirected},
                                              projection)
             ? std::optional(projection)
             : std::nullopt;
}

bool DspCore::pipeline_loop_control() const {
  if (impl_->active || impl_->faulted || impl_->run.begin) {
    throw Fault("Native loop control requires a healthy suspension boundary");
  }
  const auto &fetch = impl_->native.reference().fetch();
  return fetch.active() &&
         fetch.accepts_loop_control(loop_control(impl_->cpu.referenceAttemptState()));
}

bool DspCore::pipeline_loop_result() const {
  if (impl_->active || impl_->faulted || impl_->run.begin) {
    throw Fault("Native loop result requires a healthy suspension boundary");
  }
  const auto &result = impl_->results.retirement;
  return result && impl_->native.reference().fetch().owns_loop_result(loop_control(result->cpu),
                                                                      result->redirected);
}

std::optional<std::uint32_t> DspCore::pipeline_loop_last(const DspFetchedWord word) const {
  static_cast<void>(pipeline_fetched_word(0));
  return impl_->native.reference().fetch().loop_last(word);
}

std::uint32_t DspCore::pipeline_continuation(const DspFetchedWord word,
                                             const unsigned words) const {
  static_cast<void>(pipeline_fetched_word(0));
  const auto &fetch = impl_->native.reference().fetch();
  return fetch.continuation_after(fetch.instruction(word, words));
}

void DspCore::hold_pipeline_loop(const DspFetchedWord word) {
  static_cast<void>(pipeline_fetched_word(0));
  if (!automatic_loop_source(word.word.value)) {
    throw Fault("Unqualified automatic DO source read P:" + hex(word.word.address, 6) +
                " pc=" + hex(state().pc, 6) + " omr=" + hex(state().omr, 6));
  }
  auto &fetch = impl_->native.reference().fetch();
  fetch.hold_after(fetch.instruction(word, 2));
}

void DspCore::hold_pipeline_branch(const DspFetchedWord word) {
  static_cast<void>(pipeline_fetched_word(0));
  const auto form = decode_branch_form(word.word.value);
  if (!form || form->displacement || form->address_register || form->absolute_extension) {
    throw Fault("OMR branch barrier requires its actual two-word F2 instruction");
  }
  auto &fetch = impl_->native.reference().fetch();
  fetch.hold_after(fetch.instruction(word, 2));
}

void DspCore::validate_reference_fetch(const DspProgramFetchStage stage) const {
  if (impl_->active || impl_->faulted || impl_->run.begin) {
    throw Fault("Program fetching requires a healthy unclocked native boundary");
  }
  impl_->native.reference().fetch().check_stage(stage);
}

void DspCore::validate_reference_admission(const DspReferenceTicket &ticket,
                                           const bool retiring_previous,
                                           const std::optional<std::uint64_t> cycle) const {
  validate_reference_ticket(ticket);
  const auto pending = reference_pending_unchecked();
  if (impl_->results.previous && !retiring_previous && pending &&
      std::holds_alternative<DspInstructionRetire>(pending->effect)) {
    throw Fault("Younger native result cannot retire before its actual predecessor");
  }
  impl_->native.reference().check_admission();
  impl_->check_fetch_opcode();
  impl_->validate_peripheral_admission(cycle);
}

void DspCore::fail_reference_cycle() noexcept {
  impl_->faulted = true;
}

DspReferenceProgress DspCore::advance_reference_access(const DspReferenceTicket &ticket) {
  validate_reference_admission(ticket);
  const auto before = impl_->native.reference().snapshot().context;
  const ExecutionGuard guard(impl_->active);
  try {
    if (impl_->results.addresses.pending) {
      return reference_progress(impl_->publish_address_phase());
    }
    if (impl_->results.writeback.pending) {
      return reference_progress(impl_->publish_move_writeback());
    }
    const auto pending = impl_->native.reference().pending();
    if (pending && std::holds_alternative<DspInstructionRetire>(*pending)) {
      return reference_progress(impl_->retire_reference());
    }
    auto completed = impl_->native.reference().commit();
    impl_->observe_peripheral_store(completed);
    if (const auto *event = std::get_if<DspNativeAccess>(&completed);
        event != nullptr && event->kind == DspNativeAccessKind::opcode) {
      static_cast<void>(impl_->move_writeback_fields(nullptr, DspMoveDecode::nonmemory));
    }
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

DspPeripheralReadTicket
DspCore::capture_reference_peripheral_read(const DspPeripheralReadPoint point) {
  return capture_peripheral_read(point, false);
}

DspMemoryAccess DspCore::pipeline_peripheral_access(const std::uint32_t address,
                                                    const std::uint64_t occurrence) const {
  if (impl_->active || impl_->faulted || impl_->run.begin ||
      !impl_->native.reference().fetch().active()) {
    throw Fault("Unavailable pipeline peripheral AG1 pc=" + hex(address, 6));
  }
  const auto &fetch = impl_->native.reference().fetch();
  if (occurrence == 0) { throw Fault("Automatic peripheral AG1 requires its actual F2 identity"); }
  const auto opcode = fetch.select(address, occurrence);
  const auto form = decode_peripheral_read(opcode.word.value);
  if (!form || fetch.view().omr != state().omr) {
    throw Fault("Unqualified actual peripheral AG1/F2 word pc=" + hex(address, 6));
  }
  if (!form->bit && decode_move_writeback(opcode.word.value) == DspMoveFields{}) {
    throw Fault("Unqualified native peripheral destination pc=" + hex(address, 6));
  }
  static_cast<void>(fetch.instruction(opcode, form->words));
  return {.target = form->target, .phase = DspMemoryPhase::peripheral_sample};
}

DspPeripheralReadTicket
DspCore::capture_pipeline_peripheral_read(const DspPeripheralReadPoint point,
                                          const std::uint64_t occurrence) {
  static_cast<void>(pipeline_peripheral_access(point.address, occurrence));
  return capture_peripheral_read(point, true, occurrence);
}

DspPeripheralReadTicket DspCore::capture_peripheral_read(const DspPeripheralReadPoint point,
                                                         const bool automatic,
                                                         const std::uint64_t occurrence) {
  const auto &reference = impl_->native.reference();
  if (impl_->active || impl_->faulted || impl_->run.begin || !reference.fetch().active() ||
      impl_->peripheral_epoch() != point.epoch) {
    throw Fault("Unqualified peripheral capture pc=" + hex(point.address, 6));
  }
  if (automatic) {
    if (reference.active() &&
        reference.execution() != DspReferenceExecution::staged_register_boundaries) {
      throw Fault("Automatic peripheral capture has an incompatible native owner");
    }
  } else {
    if (!reference.active() ||
        reference.execution() != DspReferenceExecution::access_and_retirement_boundaries) {
      throw Fault("Unqualified explicit peripheral capture pc=" + hex(point.address, 6));
    }
    impl_->validate_peripheral_capture(point);
  }
  const auto selected = capture_read_word(reference.fetch(), point, occurrence, automatic);
  auto &stages = impl_->peripheral_reads;
  if (stages.serial == std::numeric_limits<std::uint64_t>::max() ||
      (stages.last_store_cycle && point.epoch <= *stages.last_store_cycle)) {
    throw Fault("Peripheral capture requires a fresh pre-write phase pc=" + hex(point.address, 6));
  }
  for (const auto &record : stages.records) {
    if (record && record->sample.fetched().opcode.occurrence == selected.occurrence) {
      throw Fault("Duplicate owned peripheral capture pc=" + hex(point.address, 6));
    }
  }
  for (std::size_t slot = 0; slot < stages.records.size(); ++slot) {
    if (stages.records.at(slot)) { continue; }
    const ExecutionGuard guard(impl_->active);
    try {
      const auto sample =
          DspPeripheralRead::capture(impl_->bus, reference.fetch(), point, selected.occurrence);
      stages.records.at(slot) = DspPeripheralReadRecord{.sample = sample,
                                                        .point = point,
                                                        .origin = impl_->reference_owner,
                                                        .serial = ++stages.serial,
                                                        .automatic = automatic};
      DspPeripheralReadTicket ticket;
      ticket.owner_ = impl_->reference_owner;
      ticket.origin_ = impl_->reference_owner;
      ticket.slot_ = slot;
      ticket.serial_ = stages.serial;
      return ticket;
    } catch (...) {
      impl_->faulted = true;
      throw;
    }
  }
  throw Fault("Peripheral response capacity exhausted pc=" + hex(point.address, 6));
}

void DspCore::validate_reference_peripheral_read(const DspPeripheralReadTicket &ticket) const {
  if (impl_->active || impl_->faulted || ticket.owner_ != impl_->reference_owner) {
    throw Fault("Stale, foreign or unavailable native peripheral response owner");
  }
  static_cast<void>(peripheral_record(
      impl_->peripheral_reads,
      {.origin = ticket.origin_, .slot = ticket.slot_, .serial = ticket.serial_}));
}

DspPeripheralReadTicket
DspCore::rebind_reference_peripheral_read(const DspPeripheralReadTicket &ticket) const {
  if (impl_->active || impl_->faulted) { throw Fault("Unavailable peripheral response rebind"); }
  static_cast<void>(peripheral_record(
      impl_->peripheral_reads,
      {.origin = ticket.origin_, .slot = ticket.slot_, .serial = ticket.serial_}));
  auto rebound = ticket;
  rebound.owner_ = impl_->reference_owner;
  return rebound;
}

bool DspCore::has_reference_peripheral_reads() const {
  return impl_->has_peripheral_reads();
}

DspBranchDecision DspCore::preview_pipeline_peripheral_branch(const DspPeripheralReadTicket &read) {
  validate_reference_peripheral_read(read);
  const auto &record =
      peripheral_record(impl_->peripheral_reads,
                        {.origin = read.origin_, .slot = read.slot_, .serial = read.serial_});
  if (!record.automatic) { throw Fault("Branch preview requires the pipeline read owner"); }
  const auto decision = record.sample.preview_branch(impl_->peripheral_epoch());
  impl_->native.reference().fetch().predict_branch(decision,
                                                   record.sample.fetched().opcode.occurrence);
  return decision;
}

DspReferenceProgress DspCore::advance_reference_peripheral_read(const DspPeripheralReadTicket &read,
                                                                const DspReferenceTicket &native) {
  return advance_peripheral_read(
      read, native, DspReferenceExecution::access_and_retirement_boundaries);
}

DspReferenceProgress DspCore::advance_pipeline_peripheral_read(const DspPeripheralReadTicket &read,
                                                               const DspReferenceTicket &native) {
  return advance_peripheral_read(read, native, DspReferenceExecution::staged_register_boundaries);
}

DspReferenceProgress DspCore::advance_peripheral_read(const DspPeripheralReadTicket &read,
                                                      const DspReferenceTicket &native,
                                                      const DspReferenceExecution execution) {
  validate_reference_ticket(native);
  validate_reference_peripheral_read(read);
  auto &reference = impl_->native.reference();
  const auto pending = reference_pending_unchecked();
  const auto opcode = reference.committed_access(DspNativeAccessKind::opcode);
  const auto extension = reference.committed_access(DspNativeAccessKind::extension);
  const auto *access = pending ? std::get_if<DspNativeAccess>(&pending->effect) : nullptr;
  if (!opcode || access == nullptr || reference.execution() != execution) {
    throw Fault("Owned peripheral admission requires a real pending native data read");
  }
  const auto epoch = impl_->peripheral_epoch();
  const ExecutionGuard guard(impl_->active);
  try {
    auto &record =
        peripheral_record(impl_->peripheral_reads,
                          {.origin = read.origin_, .slot = read.slot_, .serial = read.serial_});
    if (record.automatic != (execution == DspReferenceExecution::staged_register_boundaries)) {
      throw Fault("Peripheral response has a different execution owner");
    }
    record.sample.fetched().check_consumed(reference.fetch().consumed_pair(), state().omr);
    const auto value = record.sample.consume(*opcode, extension, *access, epoch);
    const auto completed = reference.commit_owned_read(*access, value);
    impl_->peripheral_reads.records.at(read.slot_).reset();
    impl_->resume_reference();
    return reference_progress(completed);
  } catch (...) {
    impl_->faulted = true;
    throw;
  }
}

DspBranchDecision DspCore::capture_reference_branch(const std::uint32_t address,
                                                    const std::uint64_t occurrence) {
  if (impl_->active || impl_->faulted || impl_->run.begin ||
      !impl_->native.reference().fetch().active()) {
    throw Fault("Native branch AG1 requires a healthy active frontend at P:" + hex(address, 6));
  }
  auto &fetch = impl_->native.reference().fetch();
  const auto sample = fetch.select(address, occurrence);
  const auto opcode = sample.word;
  const auto form = decode_branch_form(opcode.value);
  const auto omr = fetch.execution_omr(sample);
  if (!form || fetch.view().omr != state().omr) {
    throw Fault("Unqualified native branch AG1/F2 instruction at P:" + hex(address, 6) +
                " pc=" + hex(address, 6) + " omr=" + hex(omr, 6));
  }
  const auto captured = fetch.instruction(sample, form->words());
  // The admitted forms cannot write OMR. The actual native E1/E2 must confirm
  // this occurrence-bound fetch decision without supplying architectural state.
  DspBranchDecision decision{.word = opcode, .omr = omr};
  const auto cpu = state();
  constexpr unsigned last_internal_frame = 15;
  if ((form->subroutine_call && cpu.sp >= last_internal_frame) ||
      (form->subroutine_return && (cpu.sp == 0 || cpu.sp > last_internal_frame))) {
    throw Fault("Unqualified native subroutine stack read P:" + hex(address, 6) +
                " pc=" + hex(cpu.pc, 6) + " sp=" + hex(cpu.sp, 6) + " omr=" + hex(omr, 6));
  }
  if (form->subroutine_return) {
    // Non-destructive observation only. Genuine native RTS must pop the frame
    // and independently confirm this occurrence-bound target in E1/E2.
    decision.next_pc = cpu.stack_pc;
    decision.taken = true;
  } else if (form->displacement) {
    decision.next_pc = (opcode.address + *form->displacement) & word_mask;
    decision.taken = true;
  } else if (form->address_register) {
    decision.next_pc = state().r.at(*form->address_register);
    decision.taken = true;
  } else {
    if (!captured.extension) { throw Fault("Native branch lost its actual F2 extension"); }
    const auto extension = captured.extension->word;
    const bool taken = form->absolute_extension || form->relative_extension ||
                       ((((omr >> form->bit) & 1U) != 0) == form->set);
    decision.extension = extension;
    const auto target =
        form->relative_extension ? (opcode.address + extension.value) & word_mask : extension.value;
    decision.next_pc = taken ? target : ((extension.address + 1) & word_mask);
    decision.taken = taken;
  }
  if (form->subroutine_call || form->subroutine_return) {
    decision.subroutine = forecast_subroutine(
        loop_control(impl_->cpu.referenceAttemptState()), form->subroutine_call, decision.next_pc);
  }
  fetch.predict_branch(decision, captured.opcode.occurrence);
  return decision;
}

DspAddressStageTicket DspCore::capture_reference_address(const std::uint32_t address,
                                                         const std::uint64_t occurrence) {
  if (impl_->active || impl_->faulted || impl_->run.begin ||
      !impl_->native.reference().fetch().active()) {
    throw Fault("Native AG1 requires a healthy unclocked frontend");
  }
  const auto &fetch = impl_->native.reference().fetch();
  const auto opcode = fetch.select(address, occurrence);
  const auto form = decode_memory_stage(opcode.word.value);
  const bool immediate_port =
      form && form->peripheral && !form->peripheral_register && !is_program_transfer(*form);
  const auto words = immediate_port ? 2U : 1U;
  const auto captured = fetch.instruction(opcode, words);
  const ExecutionGuard guard(impl_->active);
  const auto slot = impl_->stages.capture(impl_->cpu, captured);
  DspAddressStageTicket ticket;
  ticket.owner_ = impl_->reference_owner;
  ticket.slot_ = slot;
  const auto &record = impl_->stages.records.at(slot);
  if (!record) { throw Fault("Native AG1 lost its captured record"); }
  ticket.serial_ = record->serial;
  return ticket;
}

DspAddressStageView DspCore::address_stage(const DspAddressStageTicket &ticket) const {
  if (impl_->active || impl_->faulted || ticket.owner_ != impl_->reference_owner) {
    throw Fault("Stale, foreign or unavailable native address-stage owner");
  }
  return impl_->stages.get({.slot = ticket.slot_, .serial = ticket.serial_}).view;
}

DspAddressStageTicket DspCore::rebind_address_stage(const DspAddressStageTicket &ticket) const {
  static_cast<void>(impl_->stages.get({.slot = ticket.slot_, .serial = ticket.serial_}));
  auto rebound = ticket;
  rebound.owner_ = impl_->reference_owner;
  return rebound;
}

void DspCore::validate_reference_address(const DspAddressStageTicket &ticket,
                                         const bool published) const {
  const auto view = address_stage(ticket);
  if (view.published != published) {
    throw Fault("Native address-stage phase mismatch pc=" + hex(view.word.address, 6));
  }
}

DspAddressStageView DspCore::publish_reference_address(const DspAddressStageTicket &ticket) {
  validate_reference_address(ticket, false);
  auto &record = impl_->stages.get({.slot = ticket.slot_, .serial = ticket.serial_});
  const auto publish = [&](const DspAddressUpdate update, const bool pointer_update) {
    if (!pointer_update) { return; }
    impl_->cpu.regs().r.at(update.index).var = static_cast<std::int32_t>(update.value);
    impl_->results.pipeline_addresses.at(update.index) = update.value;
  };
  publish(record.view.update, record.view.pointer_update);
  if (record.view.secondary) {
    const auto second = *record.view.secondary;
    publish(second.update, second.pointer_update);
  }
  record.view.published = true;
  return record.view;
}

DspMemoryStageResult DspCore::execute_reference_memory(const DspAddressStageTicket &ticket) {
  validate_reference_address(ticket, true);
  const auto record = impl_->stages.get({.slot = ticket.slot_, .serial = ticket.serial_});
  const auto &view = record.view;
  auto active_access = view.data_accesses[0];
  try {
    DspMemoryStageResult result{.address = view, .inputs = state(), .data = {}, .progress = {}};
    auto progress = impl_->memory_entry(*this, record);
    if (view.extension) {
      if (!progress.pending) { throw Fault("Native peripheral E1 lost its actual extension"); }
      const auto *actual = std::get_if<DspNativeAccess>(&progress.pending->effect);
      if (actual == nullptr || actual->kind != DspNativeAccessKind::extension ||
          actual->target.address != view.extension->address ||
          progress.pending->latched_program_word != view.extension->value) {
        throw Fault("Native peripheral E1 extension differs from its actual AG1/F2 word");
      }
      progress = advance_reference_access(progress.pending->ticket);
    }
    record.fetched.check_consumed(impl_->native.reference().fetch().consumed_pair(),
                                  view.update.omr);
    for (std::size_t index = 0; index < view.data_access_count; ++index) {
      active_access = view.data_accesses.at(index);
      const auto actual = Impl::commit_addressed_data(*this, progress, active_access);
      if (index == 0) {
        result.data = actual;
      } else {
        result.additional_data = actual;
      }
    }
    if (progress.pending && std::holds_alternative<DspMoveWriteback>(progress.pending->effect)) {
      progress = advance_reference_access(progress.pending->ticket);
      if (!progress.completed) { throw Fault("Native memory E1 lost its real load publication"); }
      result.move = std::get<DspMoveWriteback>(*progress.completed);
    }
    if (!progress.pending ||
        !std::holds_alternative<DspInstructionRetire>(progress.pending->effect)) {
      throw Fault("Native memory E1 did not reach actual retirement");
    }
    result.progress = progress;
    impl_->stages.records.at(ticket.slot_).reset();
    return result;
  } catch (const std::exception &error) {
    impl_->faulted = true;
    throw Fault(
        "Native memory E1 " +
        std::string(active_access.operation == DspMemoryOperation::read ? "read " : "write ") +
        (active_access.target.space == DspSpace::x ? "X:" : "Y:") +
        hex(active_access.target.address, 6) + " pc=" + hex(view.word.address, 6) +
        " omr=" + hex(view.update.omr, 6) + ": " + error.what());
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
  require_reset_released();
  if (impl_->faulted) { throw Fault("DSP requires reset after a rejected reference step"); }
  if (impl_->native.reference().active() || impl_->native.reference().fetch().active()) {
    throw Fault("Cannot execute an atomic DSP step while a reference access is pending");
  }
  if (impl_->run.ready) { impl_->run.ready(); }
  const ExecutionGuard guard(impl_->active);
  // Timing/diagnostic hooks retain the same complete pre-step observation.
  // Unhooked admission needs only PC/SR/OMR/VBA, including actual lazy CCR materialization.
  const auto snapshot = impl_->run.begin ? std::optional(state()) : std::nullopt;
  const auto before = snapshot ? step_context(*snapshot) : impl_->current_step_context();
  const auto before_cycles = impl_->cpu.getCycles();
  std::optional<DspInterruptRequest> interrupt;
  StepEstimate estimate;
  try {
    impl_->invalidate();
    interrupt = impl_->select_interrupt(before, admission);
    if (snapshot) { impl_->begin_run(*snapshot, interrupt.has_value()); }
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
