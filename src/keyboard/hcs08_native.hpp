// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "address.hh"
#include "common/bus.hpp"
#include "emulate.hh"
#include "error.hh"
#include "globalcontext.hh"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_write_state.hpp"
#include "loadimage.hh"
#include "memstate.hh"
#include "opbehavior.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "space.hh"
#include "translate.hh"
#include "types.h"
#include "xml.hh"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
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

  void program(const Bus *view) { program_ = view; }

  void
  loadFill(ghidra::uint1 *bytes, const ghidra::int4 size, const ghidra::Address &address) override {
    for (ghidra::int4 index = 0; index < size; ++index) {
      const auto offset = address.getOffset() + static_cast<ghidra::uintb>(index);
      // Decoder lookahead has no bus side effects. Actual instruction
      // bytes are separately checked before execution.
      const auto target = static_cast<std::uint16_t>(offset & address_mask);
      bytes[index] = program_ != nullptr ? program_->peek(target) : bus_.peek(target);
      bus_.note_code_byte(target);
    }
  }

  [[nodiscard]] std::string getArchType() const override { return "HCS08:BE:16:default"; }

  void adjustVma(long /*adjustment*/) override { throw Fault("MCU memory cannot be relocated"); }

private:
  Bus &bus_;
  const Bus *program_ = nullptr;
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
  ghidra::VarnodeData x;
  ghidra::VarnodeData sp;
  ghidra::VarnodeData ccr;
  ghidra::VarnodeData pc;
};

inline RegisterLayout register_layout(const ghidra::Translate &translator) {
  return {.a = translator.getRegister("A"),
          .hx = translator.getRegister("HIX"),
          .x = translator.getRegister("X"),
          .sp = translator.getRegister("SP"),
          .ccr = translator.getRegister("CCR"),
          .pc = translator.getRegister("PC")};
}

struct ReturnKnowledge {
  std::uint8_t &ccr;
  std::uint8_t &a;
  std::uint16_t &hx;
};

// Execute exactly one cached translation. Ghidra's EmulatePcodeCache eagerly decodes
// the following instruction, which can report a fault after the current one has
// already committed or retain stale instructions after RAM code is modified.
class InstructionExecutor final : public ghidra::EmulateMemory {
public:
  InstructionExecutor(Bus &bus,
                      ghidra::Sleigh &translator,
                      ghidra::MemoryState &memory,
                      const RegisterLayout &layout,
                      const ReturnKnowledge knowledge)
      : EmulateMemory(&memory),
        bus_(bus),
        translator_(translator),
        program_counter_(layout.pc),
        layout_(layout),
        knowledge_(knowledge),
        known_memory_(dynamic_cast<Hcs08KnownMemory *>(&bus)) {
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
    pending_write_.reset();
  }

  [[nodiscard]] ghidra::Address getExecuteAddress() const override { return address_; }

  [[nodiscard]] bool background_requested() const { return background_; }

  void clear_background_request() { background_ = false; }

  void restore_background_request(const bool value) { background_ = value; }

  void clear_translations() {
    if (executing_) { throw Fault("Cannot invalidate an active HCS08 translation"); }
    if (pending_write_) { throw Fault("Cannot invalidate a suspended HCS08 native write"); }
    discard_translations();
  }

  [[nodiscard]] bool suspended() const { return pending_write_.has_value(); }

  [[nodiscard]] std::optional<NativeInstructionState> checkpoint_write() const {
    if (!pending_write_) { return std::nullopt; }
    NativeInstructionState result{.address = address_.getOffset(),
                                  .next = next_.getOffset(),
                                  .index = index_,
                                  .steps = steps_,
                                  .write = *pending_write_,
                                  .temporaries = {}};
    for (const auto *operation : operations_) {
      if (const auto *output = operation->getOutput()) { capture_temporary(*output, result); }
      for (ghidra::int4 n = 0; n < operation->numInput(); ++n) {
        capture_temporary(*operation->getInput(n), result);
      }
    }
    return result;
  }

  void restore_write(const NativeInstructionState &saved, const std::uint8_t opcode) {
    if (executing_ || pending_write_) {
      throw Fault("HCS08 native restore requires an empty owner");
    }
    address_ = ghidra::Address(translator_.getDefaultCodeSpace(), saved.address);
    const auto &decoded = translation();
    const bool word = saved.write.access.size == 2 && saved.write.completed == 1;
    const bool move = saved.write.access.size == 1 && saved.write.completed == 0;
    if (saved.index >= decoded.operations.size() || saved.steps > operation_limit ||
        !(word || move) || saved.next != (address_ + decoded.length).getOffset()) {
      throw Fault("Invalid trusted HCS08 suspended native write");
    }
    operations_ = decoded.operations;
    next_ = ghidra::Address(translator_.getDefaultCodeSpace(), saved.next);
    index_ = saved.index;
    steps_ = saved.steps;
    pending_write_ = saved.write;
    word_write_ = word;
    raw_load_ = false;
    raw_store_ = false;
    loaded_operand_.reset();
    byte_move_ = move;
    stack_call_ = subroutine_call(opcode);
    stack_return_ = false;
    finished_ = false;
    for (const auto &temporary : saved.temporaries) {
      memstate->setChunk(temporary.bytes.data(),
                         translator_.getUniqueSpace(),
                         temporary.address,
                         static_cast<ghidra::int4>(temporary.bytes.size()));
    }
  }

  void captured_read(Hcs08DataRead *operand) { operand_ = operand; }

  void captured_return(std::array<Hcs08StackRead, 5> *reads) { return_reads_ = reads; }

  void prepare_return_frame(const std::optional<std::uint16_t> stack) {
    return_frame_.reset();
    if (stack) { return_frame_ = ReturnFrame{.stack = *stack, .pending = std::nullopt}; }
  }

  [[nodiscard]] std::array<std::uint8_t, 3> return_masks() const {
    constexpr unsigned all_reads = 7;
    if (!return_frame_ || return_frame_->seen != all_reads || return_frame_->pending) {
      throw Fault("HCS08 RTI did not read its actual CCR/A/X frame fields");
    }
    return return_frame_->masks;
  }

  [[nodiscard]] std::optional<Hcs08KnownByte> loaded_operand() const {
    if (raw_load_ && !loaded_operand_) {
      throw Fault("HCS08 qualified LDA did not consume its actual byte operand");
    }
    return loaded_operand_;
  }

  bool execute(const Bus *program = nullptr,
               const bool word_write = false,
               const bool byte_move = false) {
    const ExecutionGuard guard(executing_);
    if (pending_write_) {
      if (word_write != word_write_ || byte_move != byte_move_) {
        throw Fault("HCS08 suspended write requires its original bus-period owner");
      }
      commit_write_byte();
      pending_write_.reset();
      ++index_;
      return continue_execution();
    }
    const auto &code = program != nullptr ? *program : bus_;
    code.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()), .length = 1});
    const auto opcode = code.peek(static_cast<std::uint16_t>(address_.getOffset()));
    constexpr unsigned lda_direct = 0xb6;
    constexpr unsigned lda_extended = 0xc6;
    constexpr unsigned sta_direct = 0xb7;
    constexpr unsigned sta_extended = 0xc7;
    raw_load_ = known_memory_ != nullptr && (opcode == lda_direct || opcode == lda_extended);
    raw_store_ =
        (opcode == sta_direct || opcode == sta_extended) && knowledge_.a != Hcs08KnownByte::all;
    loaded_operand_.reset();
    constexpr unsigned rts = 0x81;
    stack_call_ = subroutine_call(opcode);
    stack_return_ = opcode == rts;
    const auto &decoded = translation();
    code.check_execute({.address = static_cast<std::uint16_t>(address_.getOffset()),
                        .length = static_cast<unsigned>(decoded.length)});
    bus_.begin_execution({.address = static_cast<std::uint16_t>(address_.getOffset()),
                          .length = static_cast<unsigned>(decoded.length)});
    operations_ = decoded.operations;
    next_ = address_ + decoded.length;
    memstate->setValue(&program_counter_, next_.getOffset());
    index_ = 0;
    steps_ = 0;
    finished_ = false;
    word_write_ = word_write;
    byte_move_ = byte_move;
    return continue_execution();
  }

private:
  [[nodiscard]] static bool subroutine_call(const unsigned opcode) {
    constexpr unsigned bsr = 0xad;
    constexpr unsigned first_jsr = 0xbd;
    constexpr unsigned last_jsr = 0xfd;
    constexpr unsigned call_nibble = 0xd;
    return opcode == bsr ||
           (opcode >= first_jsr && opcode <= last_jsr && (opcode & 15U) == call_nibble);
  }

  bool continue_execution() {
    while (!finished_ && !pending_write_) {
      if (steps_ == operation_limit) { throw Fault("HCS08 p-code operation limit exceeded"); }
      if (index_ == operations_.size()) {
        setExecuteAddress(next_);
        break;
      }
      currentOp = operations_[index_];
      currentBehave = currentOp->getBehavior();
      executeCurrentOp();
      ++steps_;
    }
    if (finished_) { memstate->setValue(&program_counter_, address_.getOffset()); }
    return finished_;
  }

  void capture_temporary(const ghidra::VarnodeData &node, NativeInstructionState &saved) const {
    if (node.space != translator_.getUniqueSpace()) { return; }
    if (node.size == 0 || node.size > sizeof(ghidra::uintb)) {
      throw Fault("Unqualified HCS08 native temporary width");
    }
    if (std::ranges::any_of(saved.temporaries, [&](const auto &cell) {
          return cell.address == node.offset && cell.bytes.size() == node.size;
        })) {
      return;
    }
    NativeTemporary cell{.address = node.offset, .bytes = std::vector<std::uint8_t>(node.size)};
    memstate->getChunk(
        cell.bytes.data(), node.space, node.offset, static_cast<ghidra::int4>(node.size));
    saved.temporaries.push_back(std::move(cell));
  }

  void commit_write_byte() {
    if (!pending_write_ || pending_write_->access.size == 0 || pending_write_->access.size > 2 ||
        pending_write_->completed >= pending_write_->access.size) {
      throw Fault("HCS08 native word write lost its pending byte");
    }
    auto &write = *pending_write_;
    constexpr unsigned byte_bits = 8;
    constexpr unsigned byte_mask = 0xff;
    const unsigned offset = stack_call_ ? 1U - write.completed : write.completed;
    const auto shift = (write.access.size - 1U - offset) * byte_bits;
    bus_.write_memory({.address = static_cast<std::uint16_t>(write.access.address + offset),
                       .size = 1,
                       .value = (write.access.value >> shift) & byte_mask});
    ++write.completed;
  }

protected:
  void fallthruOp() override {
    if (!pending_write_) { ++index_; }
  }

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
    write_value(currentOp->getOutput(), read_operand(access));
  }

  void executeStore() override {
    const auto access = memory_access(true);
    write_operand({.address = access.address,
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
    return read_operand({.address = static_cast<std::uint16_t>(node->offset),
                         .size = static_cast<unsigned>(node->size)});
  }

  [[nodiscard]] Hcs08KnownByte return_byte(const unsigned index) const {
    if (return_reads_ == nullptr || (!return_frame_ && !stack_return_)) {
      throw Fault("HCS08 captured return read lacks its owned frame");
    }
    if (stack_return_ && index >= 2) { throw Fault("HCS08 RTS has only two stack reads"); }
    auto &read = return_reads_->at(index);
    const auto stack = stack_return_ ? memstate->getValue(&layout_.sp) : return_frame_->stack + 1U;
    const auto address = static_cast<std::uint16_t>(stack + index);
    const unsigned raw_fields = stack_return_ ? 0U : 3U;
    if (!read.value || read.in_flight || read.consumed || read.address != address ||
        (index >= raw_fields && read.known != Hcs08KnownByte::all)) {
      throw Fault("HCS08 captured RTI read address=" + hex(address) +
                  ": missing, immature, repeated, mismatched or unknown PC byte");
    }
    read.consumed = true;
    return {.value = *read.value, .known = read.known};
  }

  [[nodiscard]] Hcs08KnownByte raw_return_byte(const MemoryRead access,
                                               const unsigned index) const {
    if (return_reads_ != nullptr) { return return_byte(index); }
    if (known_memory_ != nullptr) { return known_memory_->read_known_byte(access.address); }
    return {.value = static_cast<std::uint8_t>(bus_.read_memory(access))};
  }

  [[nodiscard]] ghidra::uintb read_captured_operand(const MemoryRead access) const {
    if (access.size != 1 || !operand_->value || operand_->in_flight || operand_->consumed ||
        operand_->address != access.address) {
      throw Fault("HCS08 captured operand read address=" + hex(access.address) +
                  ": missing, immature, repeated or mismatched actual byte read");
    }
    if (raw_load_) {
      if (loaded_operand_) { throw Fault("HCS08 captured LDA repeats its native byte copy"); }
      loaded_operand_ = Hcs08KnownByte{.value = *operand_->value, .known = operand_->known};
    } else if (operand_->known != Hcs08KnownByte::all) {
      throw Fault("HCS08 captured operand address=" + hex(access.address) +
                  " pc=" + hex(address_.getOffset()) + ": unqualified partial-byte consumer");
    }
    operand_->consumed = true;
    return static_cast<ghidra::uintb>(*operand_->value & operand_->known);
  }

  [[nodiscard]] ghidra::uintb read_operand(const MemoryRead access) const {
    if (operand_ != nullptr) { return read_captured_operand(access); }
    if (return_frame_ && access.size == 1) {
      for (unsigned index = 0; index < return_frame_->masks.size(); ++index) {
        const auto address = static_cast<std::uint16_t>(return_frame_->stack + 1U + index);
        if (access.address != address) { continue; }
        if ((return_frame_->seen & (1U << index)) != 0) {
          throw Fault("HCS08 RTI repeats a qualified raw frame-byte read");
        }
        const auto value = raw_return_byte(access, index);
        if (return_frame_->pending) {
          throw Fault("HCS08 RTI read precedes completion of its previous raw register copy");
        }
        return_frame_->masks[index] = value.known;
        return_frame_->pending = index;
        return static_cast<ghidra::uintb>(value.value & value.known);
      }
    }
    if (return_reads_ != nullptr) { return read_captured_return(access); }
    if (return_frame_ && access.size == 2 && access.address == address_mask &&
        static_cast<std::uint16_t>(return_frame_->stack + 4U) == access.address) {
      const auto high = bus_.read_memory({.address = access.address});
      return (high << 8U) | bus_.read_memory({.address = 0});
    }
    if (stack_return_ && access.address == address_mask) { return read_wrapped_return(access); }
    if (raw_load_) { return read_raw_load(access); }
    return bus_.read_memory(access);
  }

  [[nodiscard]] ghidra::uintb read_raw_load(const MemoryRead access) const {
    if (access.size != 1 || loaded_operand_ || known_memory_ == nullptr) {
      throw Fault("HCS08 LDA read address=" + hex(access.address) + " pc=" +
                  hex(address_.getOffset()) + " ccr=" + hex(memstate->getValue(&layout_.ccr)) +
                  " agent=CPU: unqualified raw operand transaction");
    }
    loaded_operand_ = known_memory_->read_known_byte(access.address);
    return static_cast<ghidra::uintb>(loaded_operand_->value & loaded_operand_->known);
  }

  [[nodiscard]] ghidra::uintb read_captured_return(const MemoryRead access) const {
    if (!stack_return_ && !return_frame_) { throw Fault("HCS08 captured return lost its frame"); }
    const auto expected = stack_return_ ? memstate->getValue(&layout_.sp)
                                        : static_cast<std::uint16_t>(return_frame_->stack + 4U);
    if (access.size != 2 || access.address != expected) {
      throw Fault("HCS08 captured return unexpected native read address=" + hex(access.address));
    }
    const unsigned first = stack_return_ ? 0U : 3U;
    const auto high = return_byte(first);
    const auto low = return_byte(first + 1U);
    return (static_cast<ghidra::uintb>(high.value) << 8U) | low.value;
  }

  [[nodiscard]] ghidra::uintb read_wrapped_return(const MemoryRead access) const {
    if (access.size != 2 || access.address != memstate->getValue(&layout_.sp)) {
      throw Fault("Unqualified HCS08 RTS word read address=" + hex(access.address) +
                  " pc=" + hex(address_.getOffset()));
    }
    const auto high = bus_.read_memory({.address = access.address});
    return (high << 8U) | bus_.read_memory({.address = 0});
  }

  void write_register(const ghidra::VarnodeData *node, ghidra::uintb value) {
    constexpr unsigned fixed = 0x60;
    constexpr unsigned high = 0xff00;
    const auto index = return_frame_ ? return_frame_->pending : std::nullopt;
    if (index) {
      const std::array<const ghidra::VarnodeData *, 3> fields{&layout_.ccr, &layout_.a, &layout_.x};
      const auto *expected = fields.at(*index);
      if (node->space != expected->space || node->offset != expected->offset ||
          node->size != expected->size) {
        throw Fault("HCS08 RTI raw frame copy has an unqualified register destination");
      }
      if (*index == 0) { value |= fixed; }
    }
    memstate->setValue(node, value);
    if (!index) { return; }
    if (!return_frame_) { throw Fault("HCS08 RTI frame lost during its raw register copy"); }
    const auto mask = return_frame_->masks.at(*index);
    switch (*index) {
      case 0: knowledge_.ccr = static_cast<std::uint8_t>(mask | fixed); break;
      case 1: knowledge_.a = mask; break;
      case 2: knowledge_.hx = static_cast<std::uint16_t>((knowledge_.hx & high) | mask); break;
      default: throw Fault("HCS08 RTI raw frame field exceeds CCR/A/X");
    }
    return_frame_->seen |= 1U << *index;
    return_frame_->pending.reset();
  }

  void write_value(const ghidra::VarnodeData *node, const ghidra::uintb value) {
    if (node->space != translator_.getDefaultCodeSpace()) {
      write_register(node, value);
      return;
    }
    if (node->offset > address_mask) { throw Fault("HCS08 constant operand exceeds 16 bits"); }
    write_operand({.address = static_cast<std::uint16_t>(node->offset),
                   .size = static_cast<unsigned>(node->size),
                   .value = value});
  }

  void write_operand(const MemoryWrite access) {
    if (operand_ != nullptr || return_reads_ != nullptr) {
      throw Fault("HCS08 captured operand unexpected write address=" + hex(access.address));
    }
    if (raw_store_) {
      if (access.size != 1 || known_memory_ == nullptr ||
          access.value != memstate->getValue(&layout_.a)) {
        throw Fault("HCS08 STA write address=" + hex(access.address) + " pc=" +
                    hex(address_.getOffset()) + " ccr=" + hex(memstate->getValue(&layout_.ccr)) +
                    " agent=CPU: unqualified masked operand transaction");
      }
      known_memory_->write_known_byte(
          {.address = access.address,
           .value = static_cast<std::uint8_t>(access.value & knowledge_.a)},
          knowledge_.a);
      return;
    }
    if (!word_write_ && !byte_move_) {
      if (stack_call_) {
        write_call_word(access);
      } else {
        bus_.write_memory(access);
      }
      return;
    }
    constexpr unsigned word_mask = 0xffff;
    constexpr unsigned byte_mask = 0xff;
    const bool word = word_write_ && !byte_move_ && access.size == 2 && access.value <= word_mask;
    const bool move = byte_move_ && !word_write_ && access.size == 1 && access.value <= byte_mask;
    if (pending_write_ || !(word || move)) {
      throw Fault("Unqualified HCS08 native bus-period write address=" + hex(access.address));
    }
    if (stack_call_ && (!word || access.address != memstate->getValue(&layout_.sp))) {
      throw Fault("HCS08 native call write lost its actual stack pointer");
    }
    pending_write_ = PendingNativeWrite{.access = access};
    if (word) { commit_write_byte(); }
  }

  void write_call_word(const MemoryWrite access) {
    constexpr unsigned word_mask = 0xffff;
    constexpr unsigned byte_mask = 0xff;
    constexpr unsigned byte_bits = 8;
    if (access.size != 2 || access.value > word_mask ||
        access.address != memstate->getValue(&layout_.sp)) {
      throw Fault("Unqualified HCS08 call word write address=" + hex(access.address) +
                  " pc=" + hex(address_.getOffset()));
    }
    // Family Rev2 p237/254: genuine native return PCL precedes PCH, including stack wrap.
    bus_.write_memory({.address = static_cast<std::uint16_t>(access.address + 1U),
                       .size = 1,
                       .value = access.value & byte_mask});
    bus_.write_memory({.address = access.address, .size = 1, .value = access.value >> byte_bits});
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
  const RegisterLayout &layout_;
  ReturnKnowledge knowledge_;
  Hcs08KnownMemory *known_memory_;
  Hcs08DataRead *operand_ = nullptr;
  std::array<Hcs08StackRead, 5> *return_reads_ = nullptr;

  struct ReturnFrame {
    std::uint16_t stack;
    std::array<std::uint8_t, 3> masks{};
    unsigned seen = 0;
    std::optional<unsigned> pending;
  };

  mutable std::optional<ReturnFrame> return_frame_;
  mutable std::optional<Hcs08KnownByte> loaded_operand_;
  ghidra::Address address_;
  ghidra::Address next_;
  std::span<ghidra::PcodeOpRaw *const> operations_;
  std::unordered_map<std::uint16_t, std::unique_ptr<DecodedInstruction>> translations_;
  std::vector<ghidra::OpBehavior *> behaviors_;
  std::vector<std::string> user_operations_;
  std::size_t index_ = 0;
  unsigned steps_ = 0;
  std::optional<PendingNativeWrite> pending_write_;
  bool word_write_ = false, byte_move_ = false;
  bool stack_call_ = false, stack_return_ = false;
  bool raw_load_ = false, raw_store_ = false;
  bool finished_ = true;
  bool executing_ = false;
  bool background_ = false;
};

} // namespace blofeld::detail::hcs08
