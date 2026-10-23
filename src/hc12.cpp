// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12.hpp"

#include "address.hh"
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "emulate.hh"
#include "error.hh"
#include "globalcontext.hh"
#include "hc12_decimal.hpp"
#include "loadimage.hh"
#include "memstate.hh"
#include "opbehavior.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "space.hh"
#include "translate.hh"
#include "types.h"
#include "xml.hh"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace blofeld {

namespace {
constexpr unsigned first_maskable_vector = 0xff00;
constexpr unsigned last_maskable_vector = 0xfff2;

class CodeImage final : public ghidra::LoadImage {
public:
  explicit CodeImage(Bus &bus) : LoadImage("Blofeld MCU memory"), bus_(bus) {}

  void
  loadFill(ghidra::uint1 *bytes, const ghidra::int4 size, const ghidra::Address &address) override {
    for (ghidra::int4 index = 0; index < size; ++index) {
      const auto offset = address.getOffset() + static_cast<ghidra::uintb>(index);
      // Decoder lookahead has no bus side effects. Actual instruction
      // bytes are separately checked before execution.
      const auto target = static_cast<std::uint16_t>(offset & hc12::address_mask);
      bytes[index] = bus_.peek(target);
      bus_.note_code_byte(target);
    }
  }

  [[nodiscard]] std::string getArchType() const override { return "HC-12:BE:16:default"; }

  void adjustVma(long /*adjustment*/) override { throw Fault("MCU memory cannot be relocated"); }

private:
  Bus &bus_;
};

class BusBank final : public ghidra::MemoryBank {
public:
  BusBank(ghidra::AddrSpace *space, Bus &bus) : MemoryBank(space, 1, 256), bus_(bus) {}

protected:
  void insert(const ghidra::uintb address, const ghidra::uintb value) override {
    if (address > hc12::address_mask) { throw Fault("HC12 write address exceeds 16 bits"); }
    bus_.write(static_cast<std::uint16_t>(address), static_cast<std::uint8_t>(value));
  }

  [[nodiscard]] ghidra::uintb find(const ghidra::uintb address) const override {
    if (address > hc12::address_mask) { throw Fault("HC12 read address exceeds 16 bits"); }
    return bus_.read(static_cast<std::uint16_t>(address));
  }

private:
  Bus &bus_;
};

class Assembly final : public ghidra::AssemblyEmit {
public:
  void dump(const ghidra::Address & /*address*/,
            const std::string &mnemonic,
            const std::string &operands) override {
    text = mnemonic + (operands.empty() ? "" : " " + operands);
  }

  std::string text;
};

std::string xml_text(const std::string &text) {
  std::string result;
  for (const char character : text) {
    switch (character) {
      case '&': result += "&amp;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      default: result += character; break;
    }
  }
  return result;
}

class DecodedInstruction {
public:
  DecodedInstruction() = default;

  ~DecodedInstruction() {
    // PcodeEmitCache transfers ownership of these objects to the receiving vectors.
    for (auto *operation : operations) { delete operation; }
    for (auto *varnode : varnodes) { delete varnode; }
  }

  DecodedInstruction(const DecodedInstruction &) = delete;
  DecodedInstruction &operator=(const DecodedInstruction &) = delete;
  std::vector<ghidra::PcodeOpRaw *> operations;
  std::vector<ghidra::VarnodeData *> varnodes;
  ghidra::int4 length = 0;
};

class ExecutionGuard {
public:
  explicit ExecutionGuard(bool &active) : active_(active) {
    if (active_) { throw Fault("Re-entrant HC12 instruction execution"); }
    active_ = true;
  }

  ~ExecutionGuard() { active_ = false; }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;

private:
  bool &active_;
};

struct RegisterLayout {
  ghidra::VarnodeData d;
  ghidra::VarnodeData x;
  ghidra::VarnodeData y;
  ghidra::VarnodeData sp;
  ghidra::VarnodeData ccr;
  ghidra::VarnodeData pc;
};

RegisterLayout register_layout(const ghidra::Translate &translator) {
  return {.d = translator.getRegister("D"),
          .x = translator.getRegister("IX"),
          .y = translator.getRegister("IY"),
          .sp = translator.getRegister("SP"),
          .ccr = translator.getRegister("CCR"),
          .pc = translator.getRegister("PC")};
}

// Execute exactly one cached translation. Ghidra's EmulatePcodeCache eagerly decodes
// the following instruction, which can report a fault after the current one has
// already committed or retain stale instructions after RAM code is modified.
class InstructionExecutor final : public ghidra::EmulateMemory {
public:
  InstructionExecutor(Bus &bus,
                      ghidra::Sleigh &translator,
                      ghidra::MemoryState &memory,
                      const ghidra::VarnodeData &program_counter)
      : EmulateMemory(&memory),
        bus_(bus),
        translator_(translator),
        program_counter_(program_counter) {
    ghidra::OpBehavior::registerInstructions(behaviors_, &translator_);
    translator_.getUserOpNames(user_operations_);
  }

  ~InstructionExecutor() override {
    discard_translations();
    for (auto *behavior : behaviors_) { delete behavior; }
  }

  InstructionExecutor(const InstructionExecutor &) = delete;
  InstructionExecutor &operator=(const InstructionExecutor &) = delete;

  void setExecuteAddress(const ghidra::Address &address) override {
    address_ = address;
    finished_ = true;
  }

  [[nodiscard]] ghidra::Address getExecuteAddress() const override { return address_; }

  [[nodiscard]] bool waiting() const { return waiting_; }

  void cancel_wait() { waiting_ = false; }

  void restore_wait(const bool waiting) { waiting_ = waiting; }

  void clear_translations() {
    if (executing_) { throw Fault("Cannot invalidate an active HC12 translation"); }
    discard_translations();
  }

  void execute() {
    const ExecutionGuard guard(executing_);
    bus_.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()), .length = 1});
    const auto &decoded = translation();
    bus_.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()),
                        .length = static_cast<unsigned>(decoded.length)});
    bus_.begin_execution({.address = static_cast<std::uint16_t>(address_.getOffset()),
                          .length = static_cast<unsigned>(decoded.length)});
    operations_ = decoded.operations;
    next_ = address_ + decoded.length;
    memstate->setValue(&program_counter_, next_.getOffset());
    index_ = 0;
    finished_ = false;
    for (unsigned steps = 0; !finished_; ++steps) {
      if (steps == operation_limit) { throw Fault("HC12 p-code operation limit exceeded"); }
      if (index_ == operations_.size()) {
        setExecuteAddress(next_);
        break;
      }
      currentOp = operations_[index_];
      currentBehave = currentOp->getBehavior();
      executeCurrentOp();
    }
    memstate->setValue(&program_counter_, address_.getOffset());
  }

protected:
  void fallthruOp() override { ++index_; }

  void executeUnary() override {
    const auto *input = currentOp->getInput(0);
    const auto *output = currentOp->getOutput();
    write_value(
        output,
        currentBehave->evaluateUnary(operand_size(output), operand_size(input), read_value(input)));
  }

  void executeBinary() override {
    const auto *left = currentOp->getInput(0);
    const auto *right = currentOp->getInput(1);
    const auto *output = currentOp->getOutput();
    const auto first = read_value(left);
    const auto second = read_value(right);
    write_value(
        output,
        currentBehave->evaluateBinary(operand_size(output), operand_size(left), first, second));
  }

  void executeLoad() override {
    const auto access = memory_access(false);
    write_value(currentOp->getOutput(), bus_.read_memory(access));
  }

  void executeStore() override {
    const auto access = memory_access(true);
    bus_.write_memory({.address = access.address,
                       .size = access.size,
                       .value = read_value(currentOp->getInput(2))});
  }

  void executeCallother() override {
    const auto identifier = memstate->getValue(currentOp->getInput(0));
    if (identifier >= user_operations_.size()) {
      throw Fault("Unknown HC12 p-code user operation");
    }
    const auto &name = user_operations_.at(static_cast<std::size_t>(identifier));
    if (name == "WaitForInterrupt") {
      if (currentOp->numInput() != 1 || currentOp->getOutput() != nullptr) {
        throw Fault("Unexpected HC12 WaitForInterrupt semantics");
      }
      waiting_ = true;
      fallthruOp();
      return;
    }
    if (name == "decimalAdjustAccumulator" || name == "decimalAdjustCarry") {
      execute_decimal_adjust(name == "decimalAdjustCarry");
      fallthruOp();
      return;
    }
    if (name != "LoadStack") { throw Fault("Unimplemented HC12 operation: " + name); }
    if (currentOp->numInput() != 2 || currentOp->getOutput() == nullptr ||
        currentOp->getOutput()->size != 2) {
      throw Fault("Unexpected HC12 LoadStack semantics");
    }
    // CPU12 LDS loads its operand into SP. Ghidra represents that assignment
    // through an intrinsic so decompiler stack analysis can recognize it.
    write_value(currentOp->getOutput(), read_value(currentOp->getInput(1)));
    fallthruOp();
  }

  void executeBranch() override {
    const auto destination = currentOp->getInput(0)->getAddr();
    if (!destination.isConstant()) {
      setExecuteAddress(destination);
      return;
    }
    // Relative p-code branches use signed 32-bit operation offsets, not MCU addresses.
    const auto displacement =
        std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(destination.getOffset()));
    const auto target = static_cast<std::int64_t>(index_) + displacement;
    if (target < 0 || static_cast<std::uint64_t>(target) > operations_.size()) {
      throw Fault("HC12 p-code branch outside current instruction");
    }
    index_ = static_cast<std::size_t>(target);
  }

private:
  void execute_decimal_adjust(const bool carry_result) {
    constexpr ghidra::int4 arguments = 4;
    constexpr unsigned byte_mask = 0xff;
    if (currentOp->numInput() != arguments || currentOp->getOutput() == nullptr ||
        currentOp->getOutput()->size != 1) {
      throw Fault("Unexpected HC12 decimal-adjust intrinsic shape");
    }
    for (ghidra::int4 index = 1; index < arguments; ++index) {
      if (currentOp->getInput(index)->size != 1) {
        throw Fault("Unexpected HC12 decimal-adjust operand width");
      }
    }
    const auto accumulator = read_value(currentOp->getInput(1));
    const auto carry = read_value(currentOp->getInput(2));
    const auto half = read_value(currentOp->getInput(3));
    if (accumulator > byte_mask || carry > 1 || half > 1) {
      throw Fault("Unexpected HC12 decimal-adjust operand value");
    }
    try {
      const auto result = detail::hc12_decimal_adjust(
          {.a = static_cast<std::uint8_t>(accumulator), .carry = carry != 0, .half = half != 0});
      const ghidra::uintb carry_value = result.carry ? 1U : 0U;
      write_value(currentOp->getOutput(), carry_result ? carry_value : result.a);
    } catch (const Fault &fault) {
      throw Fault("HC12 DAA pc=" + hex(address_.getOffset()) + ": " + fault.what());
    }
  }

  [[nodiscard]] static ghidra::int4 operand_size(const ghidra::VarnodeData *node) {
    if (node->size == 0 || node->size > sizeof(ghidra::uintb)) {
      throw Fault("Unsupported HC12 p-code operand width");
    }
    return static_cast<ghidra::int4>(node->size);
  }

  [[nodiscard]] ghidra::uintb read_value(const ghidra::VarnodeData *node) const {
    if (node->space != translator_.getDefaultCodeSpace()) { return memstate->getValue(node); }
    if (node->offset > hc12::address_mask) { throw Fault("HC12 constant operand exceeds 16 bits"); }
    return bus_.read_memory({.address = static_cast<std::uint16_t>(node->offset),
                             .size = static_cast<unsigned>(node->size)});
  }

  void write_value(const ghidra::VarnodeData *node, const ghidra::uintb value) {
    if (node->space != translator_.getDefaultCodeSpace()) {
      memstate->setValue(node, value);
      return;
    }
    if (node->offset > hc12::address_mask) { throw Fault("HC12 constant operand exceeds 16 bits"); }
    bus_.write_memory({.address = static_cast<std::uint16_t>(node->offset),
                       .size = static_cast<unsigned>(node->size),
                       .value = value});
  }

  [[nodiscard]] MemoryRead memory_access(const bool write) const {
    const auto *space = currentOp->getInput(0)->getSpaceFromConst();
    if (space != translator_.getDefaultCodeSpace() || space->getWordSize() != 1) {
      throw Fault("Unsupported HC12 load/store address space");
    }
    const auto offset = read_value(currentOp->getInput(1));
    if (offset > hc12::address_mask) { throw Fault("HC12 operand address exceeds 16 bits"); }
    const auto size = write ? currentOp->getInput(2)->size : currentOp->getOutput()->size;
    return {.address = static_cast<std::uint16_t>(offset), .size = static_cast<unsigned>(size)};
  }

  [[nodiscard]] const DecodedInstruction &translation() {
    const auto pc = static_cast<std::uint16_t>(address_.getOffset());
    if (const auto found = translations_.find(pc); found != translations_.end()) {
      return *found->second;
    }
    if (translations_.size() == translation_limit) { discard_translations(); }
    auto decoded = std::make_unique<DecodedInstruction>();
    ghidra::PcodeEmitCache emitter(decoded->operations, decoded->varnodes, behaviors_, 0);
    decoded->length = translator_.oneInstruction(emitter, address_);
    return *translations_.emplace(pc, std::move(decoded)).first->second;
  }

  void discard_translations() {
    operations_ = {};
    currentOp = nullptr;
    currentBehave = nullptr;
    translations_.clear();
  }

  static constexpr unsigned operation_limit = 100000;
  static constexpr unsigned translation_limit = 128;
  Bus &bus_;
  ghidra::Sleigh &translator_;
  const ghidra::VarnodeData &program_counter_;
  ghidra::Address address_;
  ghidra::Address next_;
  std::span<ghidra::PcodeOpRaw *const> operations_;
  std::unordered_map<std::uint16_t, std::unique_ptr<DecodedInstruction>> translations_;
  std::vector<ghidra::OpBehavior *> behaviors_;
  std::vector<std::string> user_operations_;
  std::size_t index_ = 0;
  bool finished_ = true;
  bool executing_ = false;
  bool waiting_ = false;
};

} // namespace

class Hc12::Impl {
public:
  Impl(Bus &bus, const std::filesystem::path &specification)
      : bus(bus), image(bus), translator(&image, &context), memory(&translator) {
    std::istringstream configuration("<sleigh>" + xml_text(specification.string()) + "</sleigh>");
    ghidra::DocumentStorage documents;
    auto *root = documents.parseDocument(configuration)->getRoot();
    documents.registerTag(root);
    translator.initialize(documents);
    layout = register_layout(translator);
    ram = std::make_unique<BusBank>(translator.getDefaultCodeSpace(), bus);
    registers = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getSpaceByName("register"), 8, 256, 1024, nullptr);
    temporaries = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getUniqueSpace(), 8, 4096, 4096, nullptr);
    memory.setMemoryBank(ram.get());
    memory.setMemoryBank(registers.get());
    memory.setMemoryBank(temporaries.get());
    executor = std::make_unique<InstructionExecutor>(bus, translator, memory, layout.pc);
  }

  void refresh_decoder(const bool force = false) {
    const auto revision = bus.code_revision();
    if (force || decoded_revision != revision) {
      executor->clear_translations();
      translator.reset(&image, &context);
      ghidra::DocumentStorage documents;
      translator.initialize(documents);
      layout = register_layout(translator);
      decoded_revision = revision;
    }
  }

  void apply_state(const CpuState &state) {
    executor->cancel_wait();
    defer_irq = false;
    memory.setValue(&layout.d, state.d);
    memory.setValue(&layout.x, state.x);
    memory.setValue(&layout.y, state.y);
    memory.setValue(&layout.sp, state.sp);
    memory.setValue(&layout.ccr, state.ccr);
    memory.setValue(&layout.pc, state.pc);
    executor->setExecuteAddress(ghidra::Address(translator.getDefaultCodeSpace(), state.pc));
  }

  void push_word(const std::uint16_t value) {
    constexpr unsigned address_space = 65536;
    const auto old = memory.getValue(&layout.sp);
    const auto stack = static_cast<std::uint16_t>((old + address_space - 2) & hc12::address_mask);
    memory.setValue(&layout.sp, stack);
    bus.write_memory({.address = stack, .size = 2, .value = value});
  }

  void push_byte(const std::uint8_t value) {
    constexpr unsigned address_space = 65536;
    const auto stack = static_cast<std::uint16_t>(
        (memory.getValue(&layout.sp) + address_space - 1) & hc12::address_mask);
    memory.setValue(&layout.sp, stack);
    bus.write_memory({.address = stack, .value = value});
  }

  std::uint64_t decoded_revision = 0;
  bool defer_irq = false;
  bool active = false;
  bool checkpointable = false;
  bool wait_clock_failed = false;
  Bus &bus;
  CodeImage image;
  ghidra::ContextInternal context;
  ghidra::Sleigh translator;
  ghidra::MemoryState memory;
  RegisterLayout layout{};
  std::unique_ptr<BusBank> ram;
  std::unique_ptr<ghidra::MemoryHashOverlay> registers;
  std::unique_ptr<ghidra::MemoryHashOverlay> temporaries;
  std::unique_ptr<InstructionExecutor> executor;
};

Hc12::Hc12(Bus &bus, const std::filesystem::path &specification) try
    : impl_(std::make_unique<Impl>(bus, specification)) {
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot initialize HC12 semantics: " + error.explain);
}

Hc12::~Hc12() = default;

void Hc12::reset(const Hc12ResetSource source) {
  if (source != Hc12ResetSource::external && source != Hc12ResetSource::cop) {
    throw Fault("Invalid HC12 reset source");
  }
  const auto vector = source == Hc12ResetSource::cop ? hc12::cop_reset_vector : hc12::reset_vector;
  const ExecutionGuard guard(impl_->active);
  impl_->checkpointable = false;
  impl_->executor->cancel_wait();
  impl_->defer_irq = false;
  CpuState initial;
  initial.sp = hc12::reset_sp;   // S12CPU15UG section 3.1.3, reset programming model.
  initial.ccr = hc12::reset_ccr; // S, X and I are set by CPU12 reset.
  impl_->bus.begin_reset({.pc = 0, .ccr = initial.ccr});
  initial.pc = static_cast<std::uint16_t>(impl_->bus.read_memory({.address = vector, .size = 2}));
  impl_->apply_state(initial);
  impl_->bus.end_reset();
  impl_->wait_clock_failed = false;
  impl_->checkpointable = true;
}

void Hc12::set_state(const CpuState &state) {
  const ExecutionGuard guard(impl_->active);
  impl_->checkpointable = false;
  impl_->apply_state(state);
  impl_->checkpointable = true;
}

CpuState Hc12::state() const {
  return {.pc = static_cast<std::uint16_t>(impl_->executor->getExecuteAddress().getOffset()),
          .d = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.d)),
          .x = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.x)),
          .y = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.y)),
          .sp = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.sp)),
          .ccr = static_cast<std::uint8_t>(impl_->memory.getValue(&impl_->layout.ccr))};
}

bool Hc12::waiting() const {
  return impl_->executor->waiting();
}

bool Hc12::executing() const {
  return impl_->active;
}

void Hc12::begin_wait_clock() {
  if (impl_->active || !impl_->checkpointable || impl_->wait_clock_failed || !waiting()) {
    throw Fault("HC12 wait clocks require a healthy completed WAI boundary");
  }
  impl_->active = true;
  impl_->checkpointable = false;
}

void Hc12::end_wait_clock(const bool healthy) {
  impl_->active = false;
  impl_->checkpointable = healthy;
  impl_->wait_clock_failed = !healthy;
}

Hc12Checkpoint Hc12::checkpoint() const {
  if (impl_->active || !impl_->checkpointable || impl_->wait_clock_failed) {
    throw Fault("HC12 checkpoint requires a completed instruction, reset or register replacement");
  }
  Hc12Checkpoint result;
  std::map<ghidra::VarnodeData, std::string> registers;
  impl_->translator.getAllRegisters(registers);
  const auto *space = impl_->translator.getSpaceByName("register");
  for (const auto &[reg, name] : registers) {
    if (reg.space == space &&
        (reg.offset >= result.registers_.size() ||
         static_cast<ghidra::uintb>(reg.size) > result.registers_.size() - reg.offset)) {
      throw Fault("HC12 checkpoint cannot represent native register " + name);
    }
  }
  impl_->registers->getChunk(
      0, static_cast<ghidra::int4>(result.registers_.size()), result.registers_.data());
  result.pc_ = state().pc;
  result.waiting_ = waiting();
  result.defer_irq_ = impl_->defer_irq;
  return result;
}

void Hc12::restore_checkpoint(const Hc12Checkpoint &checkpoint) {
  const ExecutionGuard guard(impl_->active);
  impl_->checkpointable = false;
  impl_->refresh_decoder(true);
  impl_->registers->setChunk(
      0, static_cast<ghidra::int4>(checkpoint.registers_.size()), checkpoint.registers_.data());
  impl_->executor->setExecuteAddress(
      ghidra::Address(impl_->translator.getDefaultCodeSpace(), checkpoint.pc_));
  impl_->executor->restore_wait(checkpoint.waiting_);
  impl_->wait_clock_failed = false;
  impl_->defer_irq = checkpoint.defer_irq_;
  impl_->checkpointable = true;
}

std::string Hc12::disassemble() const try {
  impl_->refresh_decoder();
  Assembly assembly;
  impl_->translator.printAssembly(assembly, impl_->executor->getExecuteAddress());
  return assembly.text;
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot disassemble HC12 at " + hex(state().pc) + ": " + error.explain);
}

std::optional<InterruptRequest> Hc12::pending_interrupt() const {
  const auto ccr =
      static_cast<std::uint8_t>(state().ccr | (impl_->defer_irq ? hc12::interrupt_mask : 0));
  if ((ccr & (hc12::interrupt_mask | hc12::xirq_mask)) ==
      (hc12::interrupt_mask | hc12::xirq_mask)) {
    return {};
  }
  const auto request = impl_->bus.interrupt_request(ccr);
  if (!request) { return {}; }
  switch (request->kind) {
    case InterruptKind::maskable:
      if ((ccr & hc12::interrupt_mask) != 0) { return {}; }
      if ((request->vector & 1U) != 0 || request->vector < first_maskable_vector ||
          request->vector > last_maskable_vector) {
        throw Fault("Invalid CPU maskable interrupt vector " + hex(request->vector));
      }
      break;
    case InterruptKind::xirq:
      if ((ccr & hc12::xirq_mask) != 0) { return {}; }
      if (request->vector != hc12::xirq_vector) { throw Fault("Invalid CPU XIRQ vector"); }
      break;
    default: throw Fault("Unknown CPU interrupt kind");
  }
  return request;
}

void Hc12::enter_interrupt(const InterruptRequest request,
                           const std::optional<std::uint16_t> reuse_stack) {
  impl_->bus.begin_interrupt(request, reuse_stack.has_value());
  auto next = state();
  const auto target =
      static_cast<std::uint16_t>(impl_->bus.read_memory({.address = request.vector, .size = 2}));
  impl_->bus.acknowledge_interrupt(request.vector);
  if (reuse_stack) {
    impl_->memory.setValue(&impl_->layout.sp, *reuse_stack);
  } else {
    impl_->push_word(next.pc);
    impl_->push_word(next.y);
    impl_->push_word(next.x);
    constexpr unsigned byte_mask = 0xff;
    const auto ba = static_cast<std::uint16_t>(((next.d & byte_mask) << 8U) | (next.d >> 8U));
    impl_->push_word(ba);
    impl_->push_byte(next.ccr);
  }
  next.sp = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.sp));
  next.ccr = static_cast<std::uint8_t>(next.ccr | hc12::interrupt_mask |
                                       (request.kind == InterruptKind::xirq ? hc12::xirq_mask : 0));
  next.pc = target;
  impl_->apply_state(next);
}

CpuStep Hc12::step() {
  if (impl_->wait_clock_failed) {
    throw Fault("HC12 wait clock failure requires coordinated recovery");
  }
  const ExecutionGuard guard(impl_->active);
  if (!impl_->checkpointable) {
    throw Fault("HC12 incomplete execution at " + hex(state().pc) +
                " requires coordinated recovery before stepping");
  }
  impl_->checkpointable = false;
  try {
    const auto before = state();
    impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
    if (const auto request = pending_interrupt()) {
      const bool reuse = waiting();
      enter_interrupt(*request, reuse ? std::optional(before.sp) : std::nullopt);
      impl_->bus.retire_instruction();
      impl_->checkpointable = true;
      return {.instruction_executed = false,
              .interrupt_vector = request->vector,
              .stack_reused = reuse};
    }
    if (waiting()) {
      impl_->checkpointable = true;
      return {.instruction_executed = false};
    }
    const auto opcode = impl_->bus.peek(before.pc);
    constexpr unsigned andcc_opcode = 0x10;
    constexpr unsigned transfer_opcode = 0xb7;
    constexpr unsigned tap_operand = 2;
    const bool clearing_instruction =
        opcode == andcc_opcode ||
        (opcode == transfer_opcode && impl_->bus.peek(static_cast<std::uint16_t>(
                                          (before.pc + 1U) & hc12::address_mask)) == tap_operand);
    impl_->refresh_decoder();
    impl_->executor->execute();
    // S12CPUV2 CLI/ANDCC and TAP guarantee execution of the following
    // instruction when they clear a previously set I bit. No elapsed cycles
    // are inferred by this instruction-boundary delivery rule.
    impl_->defer_irq = clearing_instruction && (before.ccr & hc12::interrupt_mask) != 0 &&
                       (state().ccr & hc12::interrupt_mask) == 0;
    constexpr unsigned rti_opcode = 0x0b;
    constexpr unsigned swi_opcode = 0x3f;
    CpuStep result;
    if (opcode == swi_opcode) { result.interrupt_vector = hc12::swi_vector; }
    if (opcode == rti_opcode) {
      if (const auto request = pending_interrupt()) {
        enter_interrupt(*request, before.sp);
        result.interrupt_vector = request->vector;
        result.stack_reused = true;
      }
    }
    impl_->bus.retire_instruction();
    impl_->checkpointable = true;
    return result;
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HC12 at " + hex(state().pc) + ": " + error.explain);
  }
}

} // namespace blofeld
