// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "address.hh"
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
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
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace blofeld::detail::hcs08 {
constexpr unsigned address_mask = 0xffff;

class CodeImage final : public ghidra::LoadImage {
public:
  explicit CodeImage(Bus &bus) : LoadImage("Blofeld HCS08 target memory"), bus_(bus) {}

  void
  loadFill(ghidra::uint1 *bytes, const ghidra::int4 size, const ghidra::Address &address) override {
    for (ghidra::int4 index = 0; index < size; ++index) {
      const auto offset = address.getOffset() + static_cast<ghidra::uintb>(index);
      // Decoder lookahead has no bus side effects. Actual instruction
      // bytes are separately checked before execution.
      const auto target = static_cast<std::uint16_t>(offset & address_mask);
      bytes[index] = bus_.peek(target);
      bus_.note_code_byte(target);
    }
  }

  [[nodiscard]] std::string getArchType() const override { return "HCS08:BE:16:default"; }

  void adjustVma(long /*adjustment*/) override { throw Fault("MCU memory cannot be relocated"); }

private:
  Bus &bus_;
};

class BusBank final : public ghidra::MemoryBank {
public:
  BusBank(ghidra::AddrSpace *space, Bus &bus) : MemoryBank(space, 1, 256), bus_(bus) {}

protected:
  void insert(const ghidra::uintb address, const ghidra::uintb value) override {
    if (address > address_mask) { throw Fault("HCS08 write address exceeds 16 bits"); }
    bus_.write(static_cast<std::uint16_t>(address), static_cast<std::uint8_t>(value));
  }

  [[nodiscard]] ghidra::uintb find(const ghidra::uintb address) const override {
    if (address > address_mask) { throw Fault("HCS08 read address exceeds 16 bits"); }
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

inline std::string xml_text(const std::string &text) {
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
    if (active_) { throw Fault("Re-entrant HCS08 instruction execution"); }
    active_ = true;
  }

  ~ExecutionGuard() { active_ = false; }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;

private:
  bool &active_;
};

struct RegisterLayout {
  ghidra::VarnodeData a;
  ghidra::VarnodeData hx;
  ghidra::VarnodeData sp;
  ghidra::VarnodeData ccr;
  ghidra::VarnodeData pc;
};

inline RegisterLayout register_layout(const ghidra::Translate &translator) {
  return {.a = translator.getRegister("A"),
          .hx = translator.getRegister("HIX"),
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

  [[nodiscard]] bool background_requested() const { return background_; }

  void clear_background_request() { background_ = false; }

  void clear_translations() {
    if (executing_) { throw Fault("Cannot invalidate an active HCS08 translation"); }
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
      if (steps == operation_limit) { throw Fault("HCS08 p-code operation limit exceeded"); }
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
      throw Fault("Unknown HCS08 p-code user operation");
    }
    const auto &name = user_operations_.at(static_cast<std::size_t>(identifier));
    if (name != "backgroundDebugMode" || currentOp->numInput() != 1 ||
        currentOp->getOutput() != nullptr) {
      throw Fault("Unsupported HCS08 intrinsic: " + name);
    }
    background_ = true;
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
      throw Fault("HCS08 p-code branch outside current instruction");
    }
    index_ = static_cast<std::size_t>(target);
  }

private:
  [[nodiscard]] static ghidra::int4 operand_size(const ghidra::VarnodeData *node) {
    if (node->size == 0 || node->size > sizeof(ghidra::uintb)) {
      throw Fault("Unsupported HCS08 p-code operand width");
    }
    return static_cast<ghidra::int4>(node->size);
  }

  [[nodiscard]] ghidra::uintb read_value(const ghidra::VarnodeData *node) const {
    if (node->space != translator_.getDefaultCodeSpace()) { return memstate->getValue(node); }
    if (node->offset > address_mask) { throw Fault("HCS08 constant operand exceeds 16 bits"); }
    return bus_.read_memory({.address = static_cast<std::uint16_t>(node->offset),
                             .size = static_cast<unsigned>(node->size)});
  }

  void write_value(const ghidra::VarnodeData *node, const ghidra::uintb value) {
    if (node->space != translator_.getDefaultCodeSpace()) {
      memstate->setValue(node, value);
      return;
    }
    if (node->offset > address_mask) { throw Fault("HCS08 constant operand exceeds 16 bits"); }
    bus_.write_memory({.address = static_cast<std::uint16_t>(node->offset),
                       .size = static_cast<unsigned>(node->size),
                       .value = value});
  }

  [[nodiscard]] MemoryRead memory_access(const bool write) const {
    const auto *space = currentOp->getInput(0)->getSpaceFromConst();
    if (space != translator_.getDefaultCodeSpace() || space->getWordSize() != 1) {
      throw Fault("Unsupported HCS08 load/store address space");
    }
    const auto offset = read_value(currentOp->getInput(1));
    if (offset > address_mask) { throw Fault("HCS08 operand address exceeds 16 bits"); }
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
  bool background_ = false;
};

} // namespace blofeld::detail::hcs08
