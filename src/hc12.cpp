// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hc12.hpp"

#include "address.hh"
#include "blofeld/bus.hpp"
#include "blofeld/hc12_registers.hpp"
#include "emulate.hh"
#include "error.hh"
#include "globalcontext.hh"
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
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace blofeld {

namespace {

class CodeImage final : public ghidra::LoadImage {
public:
  explicit CodeImage(Bus &bus) : LoadImage("Blofeld MCU memory"), bus_(bus) {}

  void
  loadFill(ghidra::uint1 *bytes, const ghidra::int4 size, const ghidra::Address &address) override {
    for (ghidra::int4 index = 0; index < size; ++index) {
      const auto offset = address.getOffset() + static_cast<ghidra::uintb>(index);
      // Decoder lookahead has no bus side effects. Actual instruction
      // bytes are separately checked before execution.
      bytes[index] = bus_.peek(static_cast<std::uint16_t>(offset & hc12::address_mask));
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

// Execute exactly one translation. Ghidra's EmulatePcodeCache eagerly decodes
// the following instruction, which can report a fault after the current one has
// already committed or retain stale instructions after RAM code is modified.
class InstructionExecutor final : public ghidra::EmulateMemory {
public:
  InstructionExecutor(ghidra::Sleigh &translator, ghidra::MemoryState &memory)
      : EmulateMemory(&memory), translator_(translator) {
    ghidra::OpBehavior::registerInstructions(behaviors_, &translator_);
    translator_.getUserOpNames(user_operations_);
  }

  ~InstructionExecutor() override {
    clear();
    for (auto *behavior : behaviors_) { delete behavior; }
  }

  InstructionExecutor(const InstructionExecutor &) = delete;
  InstructionExecutor &operator=(const InstructionExecutor &) = delete;

  void setExecuteAddress(const ghidra::Address &address) override {
    address_ = address;
    finished_ = true;
  }

  [[nodiscard]] ghidra::Address getExecuteAddress() const override { return address_; }

  void execute(Bus &bus) {
    bus.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()), .length = 1});
    clear();
    ghidra::PcodeEmitCache emitter(operations_, varnodes_, behaviors_, 0);
    const auto length = translator_.oneInstruction(emitter, address_);
    bus.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()),
                       .length = static_cast<unsigned>(length)});
    next_ = address_ + length;
    memstate->setValue("PC", next_.getOffset());
    index_ = 0;
    finished_ = false;
    for (unsigned steps = 0; !finished_; ++steps) {
      if (steps == operation_limit) { throw Fault("HC12 p-code operation limit exceeded"); }
      if (index_ == operations_.size()) {
        setExecuteAddress(next_);
        break;
      }
      currentOp = operations_.at(index_);
      currentBehave = currentOp->getBehavior();
      executeCurrentOp();
    }
    memstate->setValue("PC", address_.getOffset());
  }

protected:
  void fallthruOp() override { ++index_; }

  void executeCallother() override {
    const auto identifier = memstate->getValue(currentOp->getInput(0));
    if (identifier >= user_operations_.size()) {
      throw Fault("Unknown HC12 p-code user operation");
    }
    const auto &name = user_operations_.at(static_cast<std::size_t>(identifier));
    if (name != "LoadStack") { throw Fault("Unimplemented HC12 operation: " + name); }
    if (currentOp->numInput() != 2 || currentOp->getOutput() == nullptr ||
        currentOp->getOutput()->size != 2) {
      throw Fault("Unexpected HC12 LoadStack semantics");
    }
    // CPU12 LDS loads its operand into SP. Ghidra represents that assignment
    // through an intrinsic so decompiler stack analysis can recognize it.
    memstate->setValue(currentOp->getOutput(), memstate->getValue(currentOp->getInput(1)));
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
  void clear() {
    // These objects are allocated and transferred by the native PcodeEmitCache API.
    for (auto *operation : operations_) { delete operation; }
    for (auto *varnode : varnodes_) { delete varnode; }
    operations_.clear();
    varnodes_.clear();
  }

  static constexpr unsigned operation_limit = 100000;
  ghidra::Sleigh &translator_;
  ghidra::Address address_;
  ghidra::Address next_;
  std::vector<ghidra::PcodeOpRaw *> operations_;
  std::vector<ghidra::VarnodeData *> varnodes_;
  std::vector<ghidra::OpBehavior *> behaviors_;
  std::vector<std::string> user_operations_;
  std::size_t index_ = 0;
  bool finished_ = true;
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
    ram = std::make_unique<BusBank>(translator.getDefaultCodeSpace(), bus);
    registers = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getSpaceByName("register"), 8, 256, 1024, nullptr);
    temporaries = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getUniqueSpace(), 8, 4096, 4096, nullptr);
    memory.setMemoryBank(ram.get());
    memory.setMemoryBank(registers.get());
    memory.setMemoryBank(temporaries.get());
    executor = std::make_unique<InstructionExecutor>(translator, memory);
  }

  void refresh_decoder() {
    if (decoded_revision != bus.revision()) {
      translator.reset(&image, &context);
      ghidra::DocumentStorage documents;
      translator.initialize(documents);
      decoded_revision = bus.revision();
    }
  }

  std::uint64_t decoded_revision = 0;
  Bus &bus;
  CodeImage image;
  ghidra::ContextInternal context;
  ghidra::Sleigh translator;
  ghidra::MemoryState memory;
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

void Hc12::reset() {
  CpuState initial;
  initial.sp = hc12::reset_sp;   // S12CPU15UG section 3.1.3, reset programming model.
  initial.ccr = hc12::reset_ccr; // S, X and I are set by CPU12 reset.
  initial.pc = static_cast<std::uint16_t>(
      (static_cast<unsigned>(impl_->bus.read(hc12::reset_vector)) << 8U) |
      impl_->bus.read(hc12::reset_vector + 1));
  set_state(initial);
}

void Hc12::set_state(const CpuState &state) {
  impl_->memory.setValue("D", state.d);
  impl_->memory.setValue("IX", state.x);
  impl_->memory.setValue("IY", state.y);
  impl_->memory.setValue("SP", state.sp);
  impl_->memory.setValue("CCR", state.ccr);
  impl_->memory.setValue("PC", state.pc);
  impl_->executor->setExecuteAddress(
      ghidra::Address(impl_->translator.getDefaultCodeSpace(), state.pc));
}

CpuState Hc12::state() const {
  return {.pc = static_cast<std::uint16_t>(impl_->executor->getExecuteAddress().getOffset()),
          .d = static_cast<std::uint16_t>(impl_->memory.getValue("D")),
          .x = static_cast<std::uint16_t>(impl_->memory.getValue("IX")),
          .y = static_cast<std::uint16_t>(impl_->memory.getValue("IY")),
          .sp = static_cast<std::uint16_t>(impl_->memory.getValue("SP")),
          .ccr = static_cast<std::uint8_t>(impl_->memory.getValue("CCR"))};
}

std::string Hc12::disassemble() const try {
  impl_->refresh_decoder();
  Assembly assembly;
  impl_->translator.printAssembly(assembly, impl_->executor->getExecuteAddress());
  return assembly.text;
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot disassemble HC12 at " + hex(state().pc) + ": " + error.explain);
}

void Hc12::step() {
  try {
    impl_->refresh_decoder();
    impl_->executor->execute(impl_->bus);
    impl_->bus.retire_instruction();
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HC12 at " + hex(state().pc) + ": " + error.explain);
  }
}

} // namespace blofeld
