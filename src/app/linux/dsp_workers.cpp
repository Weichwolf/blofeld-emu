// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_workers.hpp"

#include "aot/artifact.hpp"
#include "aot/code.hpp"
#include "aot/dsp.hpp"
#include "aot/llvm.hpp"
#include "aot/semantic.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/foreground.hpp"
#include "dsp/io.hpp"
#include "dsp/memory.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "dsp_memory.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <llvm-c/Core.h>
#include <llvm-c/Types.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/IR/Argument.h>
#include <llvm/IR/Attributes.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/GlobalValue.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/InstrTypes.h>
#include <llvm/IR/Instruction.h>
#include <llvm/Support/Casting.h>
#include <span>
#include <stop_token>
#include <utility>
#if defined(__AVX512F__)
#include <immintrin.h>
#endif
#include <bitset>
#include <limits>
#include <llvm-c/BitReader.h>
#include <llvm-c/Error.h>
#include <llvm-c/Transforms/PassBuilder.h>
#include <llvm/ADT/StringExtras.h>
#include <llvm/Analysis/ValueTracking.h>
#include <llvm/Bitcode/BitcodeWriter.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IntrinsicInst.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/JSON.h>
#include <llvm/Support/SHA256.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/Utils/BasicBlockUtils.h>
#include <memory>
#include <optional>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using namespace dsp56300;
constexpr Word next_setup = 0x43c, voice_mode = 0x600389;
constexpr std::uint64_t outside_capture = 7;

namespace geometry {
constexpr unsigned capacity = 25;
constexpr std::size_t physical_words = blofeld::vi::dsp::Memory::physical_words;
constexpr Word high_first = 0xc000, list_base = 1746, voice_base = 50432;
constexpr Word stride = 262, amplifier = 62, idle_callback = 5646, channel = 10;
constexpr auto banks = blofeld::vi::dsp::Memory::bank_words;
constexpr Word low_x = static_cast<Word>(banks[0]);
constexpr Word high_x = static_cast<Word>(banks[0] + banks[1] + banks[2]);
constexpr Word low_y = static_cast<Word>(banks[0] + banks[1]);
constexpr Word high_y = static_cast<Word>(banks[0] + banks[1] + banks[2] + banks[3]);
constexpr Word program_low = static_cast<Word>(banks[0]);
constexpr Word high_x_words = 0x4000;
constexpr Word program_high = high_x + high_x_words;
constexpr Word program_end = static_cast<Word>(program_low + banks[3] - high_x_words);
constexpr auto voice_mapping = blofeld::vi::dsp::Memory::mapping_index(voice_mode);
}

using Ram = std::array<Word, geometry::physical_words>;

void check(const bool result, const char *message) {
  if (!result) { throw std::runtime_error(message); }
}

struct Program {
  std::uint64_t variant;
  std::vector<std::vector<Word>> code;
  std::vector<CodeSpan> spans;
  std::vector<Word> entries;
  std::vector<CodeTransition> transitions;
};

constexpr std::array mix{Word{1592}, Word{1936}};

struct WorkerPartition {
  Program program;
  std::vector<CodeSpan> exit_spans;
  std::vector<Word> exit_entries;
  std::vector<Word> failed;
  std::uint64_t instruction_slack;
};

const llvm::json::Array &array(const llvm::json::Object &input, const llvm::StringRef name) {
  const auto *value = input.getArray(name);
  check(value != nullptr, "The worker plan lacks a required array");
  return *value;
}

Word address(const llvm::json::Value &input) {
  const auto value = input.getAsInteger();
  if (!value || *value < 0 || *value > word_mask) {
    throw std::invalid_argument("The worker plan has an invalid address");
  }
  return static_cast<Word>(*value);
}

std::vector<Word> addresses(const llvm::json::Array &input) {
  std::vector<Word> result;
  for (const auto &value : input) { result.push_back(address(value)); }
  return result;
}

std::string code_identity(const WorkerPartition &partition) {
  llvm::SHA256 sha;
  const auto word = [&sha](const std::uint32_t value) {
    const std::array bytes{static_cast<std::uint8_t>(value >> 24),
                           static_cast<std::uint8_t>(value >> 16),
                           static_cast<std::uint8_t>(value >> 8),
                           static_cast<std::uint8_t>(value)};
    sha.update(llvm::ArrayRef(bytes));
  };
  const auto section = [&word](const std::span<const CodeSpan> code) {
    word(static_cast<std::uint32_t>(code.size()));
    for (const auto span : code) {
      word(span.address);
      word(static_cast<std::uint32_t>(span.words.size()));
      for (const auto opcode : span.words) { word(opcode); }
    }
  };
  section(partition.program.spans);
  section(partition.exit_spans);
  return llvm::toHex(llvm::ArrayRef(sha.final()), true);
}

WorkerPartition partition(const CodeVariant &source, const llvm::json::Object &plan) {
  check(plan.getInteger("variant") == static_cast<std::int64_t>(source.id),
        "The worker plan must target the prepared source variant");
  const auto limit = plan.getInteger("instruction_cycle_bound");
  constexpr std::int64_t instruction_cycle_ceiling = 192;
  if (!limit || *limit <= 0 || *limit >= instruction_cycle_ceiling) {
    throw std::invalid_argument("Worker plan cannot bound original instruction overshoot");
  }
  WorkerPartition result{.program = {.variant = source.id,
                                     .code = {},
                                     .spans = {},
                                     .entries = addresses(array(plan, "entries")),
                                     .transitions = {}},
                         .exit_spans = {},
                         .exit_entries = addresses(array(plan, "exit_entries")),
                         .failed = addresses(array(plan, "failed_fetches")),
                         .instruction_slack = static_cast<std::uint64_t>(*limit)};
  auto &program = result.program;
  const auto copy = [&program](const CodeSpan span) {
    const auto &words = program.code.emplace_back(span.words.begin(), span.words.end());
    return CodeSpan{.address = span.address, .words = words};
  };
  Word last = 0;
  for (const auto &region : array(plan, "worker_ranges")) {
    const auto *pair = region.getAsArray();
    check(pair != nullptr && pair->size() == 2, "The worker plan needs a start and end");
    auto cursor = address((*pair)[0]);
    const auto end = address((*pair)[1]);
    check(cursor >= last && cursor < end, "Worker ranges must be sorted and disjoint");
    // NOLINTNEXTLINE(misc-include-cleaner) The public ranges algorithm is declared by <algorithm>.
    auto span = std::ranges::lower_bound(source.code, cursor, {}, &CodeSpan::address);
    while (cursor < end) {
      check(span != source.code.end() && span->address == cursor && !span->words.empty() &&
                span->words.size() <= end - cursor,
            "The worker range must preserve the admitted instruction boundaries");
      program.spans.push_back(copy(*span));
      cursor += static_cast<Word>(span++->words.size());
    }
    last = end;
  }
  const auto count = plan.getInteger("instruction_count");
  check(count && *count == static_cast<std::int64_t>(program.spans.size()),
        "The worker plan must select every admitted instruction");
  for (const auto start : addresses(array(plan, "exit_spans"))) {
    const auto span = std::ranges::find(source.code, start, &CodeSpan::address);
    check(span != source.code.end(), "The worker's boundary must be original prepared code");
    result.exit_spans.push_back(copy(*span));
  }
  for (const auto entry : addresses(array(plan, "transitions"))) {
    program.transitions.push_back({.from = source.id, .to = outside_capture, .entry = entry});
  }
  const auto expected = plan.getString("code_sha256");
  check(expected && *expected == code_identity(result),
        "The worker plan must attest the exact original instruction spans");
  return result;
}

constexpr auto dsp_word_bits = std::bit_width(word_mask);
constexpr auto accumulator_shift =
    std::numeric_limits<std::uint64_t>::digits - std::bit_width(accumulator_mask);

class ProgramInspection final : public Io {
public:
  ProgramInspection(std::span<const Word> mapped_program, blofeld::vi::dsp::BootRom boot)
      : program_(mapped_program), boot_(boot) {}

  std::optional<Word> inspect_program(const Word at) const noexcept override {
    if (at < geometry::program_end) { return program_[at]; }
    return boot_.read(at);
  }

  [[nodiscard]] bool interrupt_pending(Word /*status*/) const noexcept override {
    ++irq_queries_;
    return false;
  }

  void clear_irq_queries() noexcept { irq_queries_ = 0; }

  [[nodiscard]] auto irq_queries() const noexcept { return irq_queries_; }

  bool read(Space /*space*/, Word /*address*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool write(Space /*space*/, Word /*address*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*address*/,
           Word /*pc*/,
           Word /*index*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

private:
  std::span<const Word> program_;
  blofeld::vi::dsp::BootRom boot_;
  mutable unsigned irq_queries_ = 0;
};

// Both tags fit in the high byte of an otherwise 24-bit private DSP word.
constexpr Word word_written = 1U << 31, word_read = 1U << 30;

constexpr std::size_t page_bytes = 4096;
constexpr std::size_t page_words = page_bytes / sizeof(Word);
constexpr std::size_t bucket_count = 64;
constexpr std::size_t page_capacity = 256;

struct Touches {
  std::array<std::uintptr_t, bucket_count> buckets{};
  std::array<std::uintptr_t, page_capacity> pages{};
  std::uint64_t used = 0;
  std::uintptr_t architectural = 0;
  static constexpr auto mask_bits = std::numeric_limits<std::uint64_t>::digits;
  using Mask = std::array<std::uint64_t, (sizeof(Registers) + mask_bits - 1) / mask_bits>;
  Mask register_inputs{};
  Mask register_writes{};
};

constexpr Touches::Mask register_bytes(std::size_t first, const std::size_t end) noexcept {
  Touches::Mask mask{};
  for (; first < end; ++first) { mask[first / 64] |= std::uint64_t{1} << (first % 64); }
  return mask;
}

struct RegisterInputs {
  const Registers &predicted;
  const Registers &canonical;
};

bool different_registers(const RegisterInputs inputs,
                         const Touches::Mask &reads,
                         const bool overwritten_b) noexcept {
  constexpr auto valid = register_bytes(0, sizeof(Registers));
  constexpr auto b = register_bytes(offsetof(Registers, b), offsetof(Registers, r));
  const auto first = std::as_bytes(std::span(&inputs.predicted, 1));
  const auto second = std::as_bytes(std::span(&inputs.canonical, 1));
  for (std::size_t chunk = 0; chunk < reads.size(); ++chunk) {
    auto mask = reads[chunk] & valid[chunk] & ~(overwritten_b ? b[chunk] : 0);
    if (mask == 0) { continue; }
    const auto offset = chunk * 64;
#if defined(__AVX512BW__)
    const auto left = _mm512_maskz_loadu_epi8(mask, first.data() + offset);
    const auto right = _mm512_maskz_loadu_epi8(mask, second.data() + offset);
    if (_mm512_mask_cmpneq_epi8_mask(mask, left, right) != 0) { return true; }
#else
    while (mask != 0) {
      const auto byte = offset + static_cast<std::size_t>(std::countr_zero(mask));
      if (first[byte] != second[byte]) { return true; }
      mask &= mask - 1;
    }
#endif
  }
  return false;
}

void preserve_registers(Registers &speculative,
                        const Registers &canonical,
                        const Touches::Mask &writes) noexcept {
  constexpr auto valid = register_bytes(0, sizeof(Registers));
  auto destination = std::as_writable_bytes(std::span(&speculative, 1));
  const auto source = std::as_bytes(std::span(&canonical, 1));
  for (std::size_t chunk = 0; chunk < writes.size(); ++chunk) {
    auto mask = valid[chunk] & ~writes[chunk];
    const auto offset = chunk * 64;
#if defined(__AVX512BW__)
    const auto saved = _mm512_maskz_loadu_epi8(mask, source.data() + offset);
    _mm512_mask_storeu_epi8(destination.data() + offset, mask, saved);
#else
    while (mask != 0) {
      const auto byte = offset + static_cast<std::size_t>(std::countr_zero(mask));
      destination[byte] = source[byte];
      mask &= mask - 1;
    }
#endif
  }
}

thread_local Touches *active_touches = nullptr;

std::int64_t local_slot() {
  const auto pointer = reinterpret_cast<std::uintptr_t>(&active_touches);
  const auto thread = reinterpret_cast<std::uintptr_t>(__builtin_thread_pointer());
  if (pointer <= thread) { return -static_cast<std::int64_t>(thread - pointer); }
  return static_cast<std::int64_t>(pointer - thread);
}

llvm::ConstantInt *metadata_offset(llvm::IRBuilder<> &ir) {
  // The process owns the initial executable TLS slot. This signed displacement,
  // never an address, is part of the instrumented module's cache inputs.
  return ir.getInt64(static_cast<std::uint64_t>(local_slot()));
}

llvm::Function *emit_touched_page(llvm::Module &module) {
  auto &abi = module.getContext();
  llvm::IRBuilder<> ir(abi);
  auto *const i64 = ir.getInt64Ty();
  auto *const ptr = ir.getPtrTy();
  auto *const record = llvm::Function::Create(llvm::FunctionType::get(ir.getVoidTy(), {ptr}, false),
                                              llvm::GlobalValue::InternalLinkage,
                                              "blofeld_private_record_page",
                                              module);
  record->addFnAttr(llvm::Attribute::AlwaysInline);
  record->addFnAttr(llvm::Attribute::NoUnwind);
  auto *entry = llvm::BasicBlock::Create(abi, "entry", record);
  auto *insert = llvm::BasicBlock::Create(abi, "insert", record);
  auto *save = llvm::BasicBlock::Create(abi, "save", record);
  auto *done = llvm::BasicBlock::Create(abi, "done", record);
  ir.SetInsertPoint(entry);
  auto *thread =
      ir.CreateCall(llvm::Intrinsic::getDeclaration(&module, llvm::Intrinsic::thread_pointer));
  auto *owner = ir.CreateLoad(ptr, ir.CreateGEP(ir.getInt8Ty(), thread, metadata_offset(ir)));
  auto *address = ir.CreatePtrToInt(record->getArg(0), i64);
  auto *page = ir.CreateAnd(address, ir.getInt64(~(page_bytes - 1)));
  auto *id = ir.CreateAnd(ir.CreateLShr(address, ir.getInt64(12)), ir.getInt64(bucket_count - 1));
  auto *bucket = ir.CreateGEP(i64, owner, id);
  auto *previous = ir.CreateLoad(i64, bucket);
  ir.CreateCondBr(ir.CreateICmpNE(previous, page), insert, done);
  ir.SetInsertPoint(insert);
  ir.CreateStore(page, bucket);
  auto *count = ir.CreateGEP(ir.getInt8Ty(), owner, ir.getInt64(offsetof(Touches, used)));
  auto *used = ir.CreateLoad(i64, count);
  ir.CreateStore(ir.CreateAdd(used, ir.getInt64(1)), count);
  ir.CreateCondBr(ir.CreateICmpULT(used, ir.getInt64(page_capacity)), save, done);
  ir.SetInsertPoint(save);
  auto *base = ir.CreateGEP(ir.getInt8Ty(), owner, ir.getInt64(offsetof(Touches, pages)));
  ir.CreateStore(page, ir.CreateGEP(i64, base, used));
  ir.CreateBr(done);
  ir.SetInsertPoint(done);
  ir.CreateRetVoid();
  return record;
}

llvm::Function *emit_register_access(llvm::Module &module) {
  auto &context = module.getContext();
  llvm::IRBuilder<> ir(context);
  auto *const i64 = ir.getInt64Ty();
  auto *const ptr = ir.getPtrTy();
  auto *const size = ir.getInt64(sizeof(Registers));
  auto *fn = llvm::Function::Create(
      llvm::FunctionType::get(ir.getVoidTy(), {ptr, i64, ir.getInt1Ty()}, false),
      llvm::GlobalValue::InternalLinkage,
      "blofeld_probe_register_access",
      module);
  fn->addFnAttr(llvm::Attribute::AlwaysInline);
  auto *entry = llvm::BasicBlock::Create(context, "entry", fn);
  auto *loop = llvm::BasicBlock::Create(context, "bucket", fn);
  auto *write = llvm::BasicBlock::Create(context, "write", fn);
  auto *read = llvm::BasicBlock::Create(context, "read", fn);
  auto *next = llvm::BasicBlock::Create(context, "next", fn);
  auto *done = llvm::BasicBlock::Create(context, "done", fn);
  ir.SetInsertPoint(entry);
  auto *thread =
      ir.CreateCall(llvm::Intrinsic::getDeclaration(&module, llvm::Intrinsic::thread_pointer));
  auto *owner = ir.CreateLoad(ptr, ir.CreateGEP(ir.getInt8Ty(), thread, metadata_offset(ir)));
  const auto field = [&](const std::size_t at) {
    return ir.CreateGEP(ir.getInt8Ty(), owner, ir.getInt64(at));
  };
  auto *base = ir.CreateLoad(i64, field(offsetof(Touches, architectural)));
  auto *address = ir.CreatePtrToInt(fn->getArg(0), i64);
  auto *right = ir.CreateAdd(address, fn->getArg(1));
  auto *beyond = ir.CreateAdd(base, size);
  auto *lo = ir.CreateSelect(ir.CreateICmpULT(address, base), base, address);
  auto *hi = ir.CreateSelect(ir.CreateICmpULT(right, beyond), right, beyond);
  auto *const offset = ir.CreateSub(lo, base);
  auto *const limit = ir.CreateSub(hi, base);
  auto *reads = field(offsetof(Touches, register_inputs));
  auto *writes = field(offsetof(Touches, register_writes));
  ir.CreateCondBr(ir.CreateICmpULT(lo, hi), loop, done);
  ir.SetInsertPoint(loop);
  auto *at = ir.CreatePHI(i64, 2);
  at->addIncoming(offset, entry);
  auto *chunk = ir.CreateLShr(at, ir.getInt64(6));
  auto *bit = ir.CreateAnd(at, ir.getInt64(Touches::mask_bits - 1));
  auto *room = ir.CreateSub(ir.getInt64(64), bit);
  auto *left = ir.CreateSub(limit, at);
  auto *bits = ir.CreateSelect(ir.CreateICmpULT(left, room), left, room);
  auto *mask = ir.CreateShl(ir.CreateLShr(llvm::ConstantInt::getAllOnesValue(i64),
                                          ir.CreateAnd(ir.CreateSub(ir.getInt64(64), bits),
                                                       ir.getInt64(Touches::mask_bits - 1))),
                            bit);
  auto *write_slot = ir.CreateGEP(i64, writes, chunk);
  auto *written = ir.CreateLoad(i64, write_slot);
  ir.CreateCondBr(fn->getArg(2), write, read);
  ir.SetInsertPoint(write);
  ir.CreateStore(ir.CreateOr(mask, written), write_slot);
  ir.CreateBr(next);
  ir.SetInsertPoint(read);
  auto *read_slot = ir.CreateGEP(i64, reads, chunk);
  auto *observed = ir.CreateLoad(i64, read_slot);
  ir.CreateStore(ir.CreateOr(observed, ir.CreateAnd(mask, ir.CreateNot(written))), read_slot);
  ir.CreateBr(next);
  ir.SetInsertPoint(next);
  auto *following = ir.CreateAdd(at, bits);
  ir.CreateCondBr(ir.CreateICmpULT(following, limit), loop, done);
  at->addIncoming(following, next);
  ir.SetInsertPoint(done);
  ir.CreateRetVoid();
  return fn;
}

bool accumulator_projection(const llvm::StoreInst &write,
                            const llvm::Argument &output,
                            const llvm::DataLayout &layout) {
  // The observer compares public Register snapshots in the lifted owner's
  // coordinates. The standard accessor certifies precisely the A/B projection.
  static_assert(offsetof(Registers, b) == offsetof(Registers, a) + sizeof(std::uint64_t));
  const auto *shift = llvm::dyn_cast<llvm::BinaryOperator>(write.getValueOperand());
  if (shift == nullptr || shift->getOpcode() != llvm::Instruction::LShr) { return false; }
  const auto *type = llvm::dyn_cast<llvm::FixedVectorType>(shift->getType());
  if (type == nullptr || type->getNumElements() != 2 || !type->getElementType()->isIntegerTy(64)) {
    return false;
  }
  const auto *input = llvm::dyn_cast<llvm::LoadInst>(shift->getOperand(0));
  const auto *amount = llvm::dyn_cast<llvm::Constant>(shift->getOperand(1));
  if (input == nullptr || amount == nullptr) { return false; }
  const auto *bits = llvm::dyn_cast_or_null<llvm::ConstantInt>(amount->getSplatValue());
  if (bits == nullptr || bits->getZExtValue() != accumulator_shift) { return false; }
  const auto on_accumulators = [&](const llvm::Value *pointer) {
    std::int64_t offset = 0;
    return llvm::GetPointerBaseWithConstantOffset(pointer, offset, layout) == &output &&
           offset == offsetof(Registers, a);
  };
  return on_accumulators(input->getPointerOperand()) && on_accumulators(write.getPointerOperand());
}

std::int64_t register_origin(const llvm::Module &module) {
  constexpr auto accessor = "_ZNK8dsp563004Core9registersEv";
  const auto *function = module.getFunction(accessor);
  check(function != nullptr && function->hasFnAttribute("dsp56300-core-interface") &&
            !function->isDeclaration() && function->arg_size() == 2,
        "The published architectural register accessor must be present");
  const auto *result = function->getArg(0);
  const auto *owner = function->getArg(1);
  const auto &layout = module.getDataLayout();
  check(result->hasStructRetAttr() && result->getParamStructRetType() != nullptr &&
            layout.getTypeAllocSize(result->getParamStructRetType()) == sizeof(Registers),
        "The published accessor must return the architectural register type");

  std::optional<std::int64_t> address;
  bool normalized = false;
  for (const auto &block : *function) {
    for (const auto &instruction : block) {
      if (const auto *write = llvm::dyn_cast<llvm::StoreInst>(&instruction)) {
        check(address && !normalized && accumulator_projection(*write, *result, layout),
              "Register accessor changed its A/B projection");
        normalized = true;
        continue;
      }
      const auto *copy = llvm::dyn_cast<llvm::MemCpyInst>(&instruction);
      if (copy == nullptr) {
        check(!instruction.mayWriteToMemory(), "Unrecognized register accessor side effect");
        continue;
      }
      const auto *length = llvm::dyn_cast<llvm::ConstantInt>(copy->getLength());
      std::int64_t destination = 0;
      check(!normalized && length != nullptr && length->getZExtValue() == sizeof(Registers) &&
                llvm::GetPointerBaseWithConstantOffset(copy->getRawDest(), destination, layout) ==
                    result &&
                destination == 0,
            "The accessor must copy exactly the public register shape");
      std::int64_t displacement = 0;
      const auto *reference = llvm::dyn_cast<llvm::LoadInst>(
          llvm::GetPointerBaseWithConstantOffset(copy->getRawSource(), displacement, layout));
      std::int64_t member = 0;
      check(!address && displacement >= 0 && reference != nullptr &&
                llvm::GetPointerBaseWithConstantOffset(
                    reference->getPointerOperand(), member, layout) == owner &&
                reference->getType()->isPointerTy(),
            "The architectural copy must name one native owner range");
      address = displacement;
    }
  }
  if (!address || !normalized) {
    throw std::invalid_argument("Cannot certify the prepared register span and normalization");
  }
  return *address;
}

void bind_register_origin(llvm::Module &module,
                          const std::span<const SemanticExport> exports,
                          const std::int64_t begin) {
  for (const auto &entry : exports) {
    auto *function = module.getFunction(llvm::StringRef(entry.name));
    check(function != nullptr && !function->isDeclaration() &&
              function->hasFnAttribute("dsp56300-image") && function->arg_size() == 4 &&
              function->getArg(0)->getType()->isPointerTy(),
          "The worker must enter through the standard native owner argument");
    llvm::IRBuilder<> ir(&*function->getEntryBlock().getFirstInsertionPt());
    auto *thread =
        ir.CreateCall(llvm::Intrinsic::getDeclaration(&module, llvm::Intrinsic::thread_pointer));
    auto *slot = ir.CreateGEP(ir.getInt8Ty(), thread, metadata_offset(ir));
    auto *trace = ir.CreateLoad(ir.getPtrTy(), slot);
    auto *origin = ir.CreateGEP(
        ir.getInt8Ty(), function->getArg(0), ir.getInt64(static_cast<std::uint64_t>(begin)));
    auto *field =
        ir.CreateGEP(ir.getInt8Ty(), trace, ir.getInt64(offsetof(Touches, architectural)));
    ir.CreateStore(ir.CreatePtrToInt(origin, ir.getInt64Ty()), field);
  }
}

enum class Access : std::uint8_t { read, write };

struct RegisterAccess {
  llvm::Value *address;
  llvm::Value *width;
  Access operation;
};

llvm::Value *image_owner(llvm::Function &function) {
  if (!function.hasFnAttribute("dsp56300-image-part")) { return nullptr; }
  check(!function.arg_empty() && function.getArg(0)->getType()->isPointerTy(),
        "Image part's owner argument");
  return function.getArg(0);
}

class RegisterBatch {
public:
  RegisterBatch(llvm::Function &hook, llvm::Value *owner, const std::int64_t origin)
      : module_(*hook.getParent()), hook_(hook), origin_(origin), owner_(owner) {}

  void observe(llvm::Instruction &operation) {
    if (llvm::isa<llvm::PHINode>(operation)) { return; }
    llvm::IRBuilder<> ir(&operation);
    ir.SetCurrentDebugLocation(operation.getDebugLoc());
    if (auto *load = llvm::dyn_cast<llvm::LoadInst>(&operation); load != nullptr) {
      if (!load->isVolatile()) {
        scalar(ir, load->getPointerOperand(), load->getType(), Access::read);
      }
      return;
    }
    if (auto *store = llvm::dyn_cast<llvm::StoreInst>(&operation); store != nullptr) {
      if (!store->isVolatile()) {
        scalar(ir, store->getPointerOperand(), store->getValueOperand()->getType(), Access::write);
      }
      return;
    }
    if (auto *bulk = llvm::dyn_cast<llvm::MemIntrinsic>(&operation); bulk != nullptr) {
      if (auto *copy = llvm::dyn_cast<llvm::MemTransferInst>(bulk); copy != nullptr) {
        mark(ir,
             {.address = copy->getRawSource(),
              .width = copy->getLength(),
              .operation = Access::read});
      }
      mark(ir,
           {.address = bulk->getRawDest(), .width = bulk->getLength(), .operation = Access::write});
      return;
    }
    if (llvm::isa<llvm::AtomicCmpXchgInst, llvm::AtomicRMWInst>(operation)) {
      throw std::invalid_argument("Unexpected prepared atomic");
    }
    if (operation.isTerminator() || llvm::isa<llvm::CallBase>(operation)) { flush(ir); }
  }

private:
  void scalar(llvm::IRBuilder<> &ir, llvm::Value *address, llvm::Type *type, const Access access) {
    const auto width = module_.getDataLayout().getTypeStoreSize(type);
    check(!width.isScalable(), "Fixed-width original register accesses");
    mark(ir,
         {.address = address, .width = ir.getInt64(width.getFixedValue()), .operation = access});
  }

  void mark(llvm::IRBuilder<> &ir, const RegisterAccess access) {
    const auto *base = llvm::getUnderlyingObject(access.address);
    if (llvm::isa<llvm::GlobalValue, llvm::AllocaInst>(base) || batch(access)) { return; }
    flush(ir);
    ir.CreateCall(&hook_,
                  {access.address,
                   ir.CreateZExtOrTrunc(access.width, ir.getInt64Ty()),
                   ir.getInt1(access.operation == Access::write)});
  }

  bool batch(const RegisterAccess access) {
    const auto *length = llvm::dyn_cast<llvm::ConstantInt>(access.width);
    std::int64_t offset = 0;
    if (owner_ == nullptr || length == nullptr || length->isNegative() ||
        llvm::GetPointerBaseWithConstantOffset(access.address, offset, module_.getDataLayout()) !=
            owner_ ||
        offset < origin_) {
      return false;
    }
    const auto first = static_cast<std::size_t>(offset - origin_);
    if (first < sizeof(Registers)) {
      const auto bytes = std::min<std::size_t>(length->getZExtValue(), sizeof(Registers) - first);
      for (auto at = first; at < first + bytes; ++at) {
        const auto chunk = at / 64;
        const auto bit = std::uint64_t{1} << (at % 64);
        if (access.operation == Access::write) {
          writes_[chunk] |= bit;
        } else {
          reads_[chunk] |= bit & ~writes_[chunk];
        }
      }
    }
    return true;
  }

  void flush(llvm::IRBuilder<> &ir) {
    const auto empty = [](const auto bits) { return bits == 0; };
    if (std::ranges::all_of(reads_, empty) && std::ranges::all_of(writes_, empty)) { return; }
    auto *thread =
        ir.CreateCall(llvm::Intrinsic::getDeclaration(&module_, llvm::Intrinsic::thread_pointer));
    auto *slot = ir.CreateGEP(ir.getInt8Ty(), thread, metadata_offset(ir));
    auto *state = ir.CreateLoad(ir.getPtrTy(), slot);
    for (std::size_t index = 0; index < reads_.size(); ++index) {
      if ((reads_[index] | writes_[index]) == 0) { continue; }
      store_trace(ir, state, index);
    }
    reads_.fill(0);
    writes_.fill(0);
  }

  void store_trace(llvm::IRBuilder<> &ir, llvm::Value *state, const std::size_t index) const {
    const auto displacement = sizeof(Touches::Mask::value_type) * index;
    auto *written_at = ir.CreateGEP(
        ir.getInt8Ty(), state, ir.getInt64(offsetof(Touches, register_writes) + displacement));
    auto *written = ir.CreateLoad(ir.getInt64Ty(), written_at);
    if (reads_[index] != 0) {
      auto *read_at = ir.CreateGEP(
          ir.getInt8Ty(), state, ir.getInt64(offsetof(Touches, register_inputs) + displacement));
      auto *read = ir.CreateLoad(ir.getInt64Ty(), read_at);
      auto *live_in = ir.CreateAnd(ir.getInt64(reads_[index]), ir.CreateNot(written));
      ir.CreateStore(ir.CreateOr(read, live_in), read_at);
    }
    if (writes_[index] != 0) {
      ir.CreateStore(ir.CreateOr(written, ir.getInt64(writes_[index])), written_at);
    }
  }

  llvm::Module &module_;
  llvm::Function &hook_;
  std::int64_t origin_;
  llvm::Value *owner_;
  Touches::Mask reads_{};
  Touches::Mask writes_{};
};

void instrument_registers(llvm::Module &module, const std::int64_t register_start) {
  auto *hook = emit_register_access(module);
  hook->removeFnAttr(llvm::Attribute::AlwaysInline);
  hook->addFnAttr(llvm::Attribute::NoInline);
  for (auto &function : module) {
    if (&function == hook) { continue; }
    RegisterBatch effects(*hook, image_owner(function), register_start);
    for (auto &block : function) {
      std::vector<llvm::Instruction *> operations;
      for (auto &operation : block) { operations.push_back(&operation); }
      for (auto *operation : operations) { effects.observe(*operation); }
    }
  }
}

void tag(llvm::Module &module) {
  std::vector<llvm::LoadInst *> reads;
  std::vector<llvm::StoreInst *> writes;
  for (auto &function : module) {
    for (auto &block : function) {
      for (auto &operation : block) {
        if (auto *load = llvm::dyn_cast<llvm::LoadInst>(&operation);
            load != nullptr && load->isVolatile()) {
          check(load->getType()->isIntegerTy(32), "Direct DSP word read required");
          reads.push_back(load);
        }
        if (auto *store = llvm::dyn_cast<llvm::StoreInst>(&operation);
            store != nullptr && store->isVolatile()) {
          check(store->getValueOperand()->getType()->isIntegerTy(32),
                "Direct DSP word store required");
          writes.push_back(store);
        }
      }
    }
  }
  auto *const recorder = emit_touched_page(module);
  for (auto *load : reads) {
    llvm::IRBuilder<> builder(load->getNextNode());
    builder.SetCurrentDebugLocation(load->getDebugLoc());
    auto *plain = builder.CreateAnd(load, builder.getInt32(word_mask));
    load->replaceUsesWithIf(plain,
                            [plain](const llvm::Use &use) { return use.getUser() != plain; });
    auto *flags = builder.CreateAnd(load, builder.getInt32(word_read | word_written));
    auto *first_touch = builder.CreateICmpEQ(flags, builder.getInt32(0));
    auto *record = llvm::SplitBlockAndInsertIfThen(first_touch, builder.GetInsertPoint(), false);
    builder.SetInsertPoint(record);
    builder.CreateCall(recorder, {load->getPointerOperand()});
    auto *metadata = builder.getInt32(word_read);
    builder.CreateStore(builder.CreateOr(plain, metadata), load->getPointerOperand())
        ->setVolatile(true);
  }
  for (auto *store : writes) {
    llvm::IRBuilder<> builder(store);
    builder.SetCurrentDebugLocation(store->getDebugLoc());
    auto *old = builder.CreateLoad(builder.getInt32Ty(), store->getPointerOperand());
    old->setVolatile(true);
    auto *preserved = builder.CreateAnd(old, builder.getInt32(word_read));
    auto *word = builder.CreateAnd(store->getValueOperand(), builder.getInt32(word_mask));
    auto *metadata = builder.CreateOr(preserved, builder.getInt32(word_written));
    builder.CreateCall(recorder, {store->getPointerOperand()});
    store->setOperand(0, builder.CreateOr(word, metadata));
  }
  check(!writes.empty() && !reads.empty(), "Instrument actual original native RAM effects");
}

namespace native = blofeld::host::aot;
using native::detail::Owner;

Owner<LLVMModuleRef, LLVMDisposeModule> parse_worker(LLVMContextRef context,
                                                     std::span<const std::byte> bytes) {
  const Owner<LLVMMemoryBufferRef, LLVMDisposeMemoryBuffer> buffer(
      LLVMCreateMemoryBufferWithMemoryRangeCopy(
          reinterpret_cast<const char *>(bytes.data()), bytes.size(), "blofeld-worker"));
  LLVMModuleRef source = nullptr;
  check(LLVMParseBitcodeInContext2(context, buffer.get(), &source) == 0 && source != nullptr,
        "Read the worker's LLVM module");
  return Owner<LLVMModuleRef, LLVMDisposeModule>(source);
}

std::vector<std::byte> instrument_worker(const SemanticImage &semantic,
                                         const native::Artifact &artifact,
                                         const native::Preparation preparation) {
  if (auto bytes = artifact.load()) { return std::move(*bytes); }
  if (preparation == native::Preparation::load_only) {
    throw std::runtime_error("Missing transformed worker image. Run blofeld-headless --prepare.");
  }
  const Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  const auto source = parse_worker(context.get(), semantic.bitcode());
  auto &module = *llvm::unwrap(source.get());
  const auto start = register_origin(module);
  const Owner<LLVMPassBuilderOptionsRef, LLVMDisposePassBuilderOptions> options(
      LLVMCreatePassBuilderOptions());
  if (auto *error = LLVMRunPasses(source.get(), "default<O3>", nullptr, options.get())) {
    const Owner<char *, LLVMDisposeErrorMessage> message(LLVMGetErrorMessage(error));
    throw std::runtime_error(message.get());
  }
  instrument_registers(module, start);
  tag(module);
  bind_register_origin(module, semantic.exports(), start);
  llvm::SmallVector<char> data;
  llvm::raw_svector_ostream stream(data);
  llvm::WriteBitcodeToFile(module, stream);
  const auto bytes = std::as_bytes(std::span(data));
  artifact.store(bytes);
  return {bytes.begin(), bytes.end()};
}

NativeImage prepare_worker_image(const CodeImage code,
                                 const CompileOptions options,
                                 const std::filesystem::path &cache,
                                 const native::Preparation preparation) {
  const auto semantic =
      blofeld::app::aot::detail::prepare_semantic<Core>(code, options, cache, preparation);
  const auto key = std::string(semantic.identity()) +
                   ":blofeld-worker-" BLOFELD_FOREGROUND_PIPELINE ":" +
                   std::to_string(local_slot());
  const native::Artifact transformed(cache, std::as_bytes(std::span(key)));
  const auto bitcode = instrument_worker(semantic, transformed, preparation);
  std::vector<native::Export> exports;
  for (const auto symbol : semantic.exports()) {
    exports.push_back({.name = symbol.name, .abi = symbol.abi});
  }
  const std::array modules{native::Module{.bitcode = bitcode, .exports = exports}};
  const auto binary = std::make_shared<native::Code>(modules,
                                                     std::span<const native::Import>{},
                                                     cache,
                                                     native::Code::ExecutableMemory::standard,
                                                     preparation);

  return blofeld::app::aot::detail::bind_dsp(semantic, binary);
}

NativeImage load_worker(const WorkerPartition &body,
                        const std::filesystem::path &cache,
                        const native::Preparation preparation) {
  const std::array variants{CodeVariant{.id = body.program.variant,
                                        .code = body.program.spans,
                                        .entries = body.program.entries,
                                        .failed_fetches = {}},
                            CodeVariant{.id = outside_capture,
                                        .code = body.exit_spans,
                                        .entries = body.exit_entries,
                                        .failed_fetches = body.failed}};
  constexpr std::array modes{Word{0}, Word{1} << 11};
  return prepare_worker_image({.variants = variants},
                              {.memory = {},
                               .interrupts = {},
                               .transitions = body.program.transitions,
                               .callbacks = {},
                               .bindings = {},
                               .memory_mode = CompileOptions::MemoryMode::direct,
                               .region_modes = modes},
                              cache,
                              preparation);
}

// A private lane stops if it fails to reach the original mixer transfer.
constexpr std::uint64_t worker_cycle_limit = 8192;
constexpr std::size_t words = geometry::physical_words;
constexpr std::size_t line_words = 64 / sizeof(Word);
constexpr std::size_t lines_per_page = page_words / line_words;
static_assert(page_words % line_words == 0 && words % page_words == 0);
using LineEpochs = std::array<std::uint64_t, words / line_words>;
using PageEpochs = std::array<std::uint64_t, words / page_words>;

dsp56300::Memory direct_memory(blofeld::vi::dsp::Memory &memory, const Word omr) {
  memory.configure(omr);
  for (const auto space : {Space::x, Space::y}) {
    for (const auto &window : memory.windows(space)) { std::ranges::fill(window.known, 1U); }
  }
  static_cast<void>(memory.promote_initialized());
  return memory.loan();
}

void protect(const std::array<Memory::Views, 3> &views, const Word record) {
  check(sysconf(_SC_PAGESIZE) == 4096, "Original guarded host mapping");
  constexpr Word page = 4096 / sizeof(Word);
  const auto apply = [](const std::span<Word> range, const int flags) {
    check(mprotect(range.data(), range.size_bytes(), flags) == 0, "Protect the shadow DSP views");
  };
  for (const auto &view : views) { apply(view.write.data, PROT_NONE); }
  constexpr int writable = PROT_READ | PROT_WRITE;
  const auto first = record & ~(page - 1);
  const auto last = (record + geometry::capacity * geometry::stride + page - 1) & ~(page - 1);
  apply(views[1].write.data.first(geometry::banks[1]), writable);
  apply(views[2].write.data.first(geometry::banks[2]), writable);
  apply(views[1].write.data.subspan(first, last - first), writable);
  // Worker tracing uses private read aliases to tag the first access to a word.
  apply(views[1].read.data.first(geometry::banks[1]), writable);
  apply(views[1].read.data.subspan(geometry::high_first, geometry::high_x_words), writable);
  apply(views[2].read.data.first(geometry::banks[2]), writable);
  apply(views[2].read.data.subspan(geometry::high_first, geometry::banks[4]), writable);
}

using LineMask = std::uint16_t;
constexpr auto full_line = std::numeric_limits<LineMask>::max();
static_assert(line_words == std::numeric_limits<LineMask>::digits);

void copy_words(Word *destination, const Word *source, LineMask mask) noexcept {
#if defined(__AVX512F__)
  _mm512_mask_storeu_epi32(destination, mask, _mm512_loadu_si512(source));
#else
  while (mask != 0) {
    const auto index = std::countr_zero(mask);
    destination[index] = source[index];
    mask &= static_cast<LineMask>(mask - 1);
  }
#endif
}

bool different_words(const Word *left, const Word *right, LineMask mask) noexcept {
#if defined(__AVX512F__)
  return _mm512_mask_cmpneq_epi32_mask(mask, _mm512_loadu_si512(left), _mm512_loadu_si512(right)) !=
         0;
#else
  if (mask == full_line) { return std::memcmp(left, right, line_words * sizeof(Word)) != 0; }
  while (mask != 0) {
    const auto index = std::countr_zero(mask);
    if (left[index] != right[index]) { return true; }
    mask &= static_cast<LineMask>(mask - 1);
  }
  return false;
#endif
}

struct LineAccess {
  LineMask reads = 0, writes = 0;
};

LineAccess collect_line(Word *data, const bool clear) noexcept {
#if defined(__AVX512F__)
  const auto block = _mm512_loadu_si512(data);
  const auto lower = _mm512_set1_epi32(static_cast<int>(word_mask));
  const auto bits =
      _mm512_cmpneq_epi32_mask(_mm512_andnot_si512(lower, block), _mm512_setzero_si512());
  if (bits == 0) { return {}; }
  const LineAccess access{
      .reads = _mm512_test_epi32_mask(block, _mm512_set1_epi32(static_cast<int>(word_read))),
      .writes = _mm512_test_epi32_mask(block, _mm512_set1_epi32(static_cast<int>(word_written)))};
  if (clear) { _mm512_mask_storeu_epi32(data, bits, _mm512_and_si512(lower, block)); }
  return access;
#else
  LineAccess access;
  for (unsigned index = 0; index < line_words; ++index) {
    const auto word = data[index];
    access.reads |= static_cast<LineMask>(((word & word_read) != 0 ? 1U : 0U) << index);
    access.writes |= static_cast<LineMask>(((word & word_written) != 0 ? 1U : 0U) << index);
    if (clear && word > word_mask) { data[index] = word & word_mask; }
  }
  return access;
#endif
}

void capture_line(Word *destination, const Word *source, LineMask mask) noexcept {
#if defined(__AVX512F__)
  const auto block = _mm512_maskz_loadu_epi32(mask, source);
  const auto low = _mm512_set1_epi32(static_cast<int>(word_mask));
  _mm512_storeu_si512(destination, _mm512_and_si512(block, low));
#else
  while (mask != 0) {
    const auto index = std::countr_zero(mask);
    destination[index] = source[index] & word_mask;
    mask &= static_cast<LineMask>(mask - 1);
  }
#endif
}

struct Effects {
  struct InputLine {
    Word base;
    LineMask reads;
  };

  std::vector<InputLine> reads = std::vector<InputLine>(words / 16);

  struct OutputLine {
    Word base;
    LineMask writes;
  };

  std::vector<OutputLine> writes = std::vector<OutputLine>(words / 16);
  std::size_t drawn = 0, touched = 0;

  [[nodiscard]] std::span<const InputLine> input() const { return std::span(reads).first(drawn); }

  [[nodiscard]] std::span<const OutputLine> output() const {
    return std::span(writes).first(touched);
  }

  [[nodiscard]] bool writes_word(const Word word) const noexcept {
    const auto lines = output();
    const auto base = word & ~Word{15};
    // NOLINTNEXTLINE(misc-include-cleaner) The public ranges algorithm is declared by <algorithm>.
    const auto found = std::ranges::lower_bound(lines, base, {}, &OutputLine::base);
    return found != lines.end() && found->base == base &&
           (found->writes & (1U << (word - base))) != 0;
  }

  void writeback(const std::span<Word> destination,
                 const std::span<const Word> source) const noexcept {
    for (const auto [base, mask] : output()) {
      copy_words(destination.data() + base, source.data() + base, mask);
    }
  }

  void extract(std::span<Word> ram,
               const std::bitset<words / page_words> &pages,
               const bool full,
               const bool clear = true) {
    check(ram.size() == words, "Scan every backing word, including aliases");
    drawn = touched = 0;
    for (std::size_t page = 0; page < pages.size(); ++page) {
      if (!full && !pages[page]) { continue; }
      const auto first = page * page_words;
      for (std::size_t base = first; base < first + page_words; base += line_words) {
        const auto [loads, stores] = collect_line(ram.data() + base, clear);
        if (loads != 0) { reads[drawn++] = {.base = static_cast<Word>(base), .reads = loads}; }
        if (stores != 0) {
          writes[touched++] = {.base = static_cast<Word>(base), .writes = stores};
        }
      }
    }
  }
};

struct Continuation {
  Registers registers{};
  std::uint64_t instructions = 0, cycles = 0;
  Word current_pc = 0;
  bool admissible = false;

  static Continuation read(const Core &core) noexcept {
    ForegroundCommit work{};
    if (!core.capture_foreground(work)) { return {}; }
    work.registers.a <<= accumulator_shift;
    work.registers.b <<= accumulator_shift;
    return {.registers = work.registers,
            .instructions = work.work.instructions,
            .cycles = work.work.estimated_cycles,
            .current_pc = work.last_pc,
            .admissible = true};
  }
};

class Lane {
public:
  Lane(const Ram &initial,
       const Registers &registers,
       blofeld::vi::dsp::BootRom rom,
       const NativeImage &image,
       const std::uint64_t variant)
      : backing(rom),
        storage(backing.storage()),
        memory({}, storage),
        inspection(storage.mappings[geometry::voice_mapping].views[0].read.data, rom),
        core(direct_memory(memory, registers.omr), &inspection) {
    check(blofeld::vi::dsp::Memory::mapping_index(registers.omr) == 4, "Observed direct view");
    assign(initial);
    core.set_registers(registers);
    core.install(image, variant);
  }

  void assign(const Ram &ram) const { std::ranges::copy(ram, storage.physical.begin()); }

  void reset(const Registers &registers) {
    core.reset();
    core.set_registers(registers);
  }

  blofeld::app::DspMemory backing;
  blofeld::vi::dsp::Memory::Storage storage;
  blofeld::vi::dsp::Memory memory;
  ProgramInspection inspection;
  Core core;
};

struct ShortReceipt {
  using Line = std::array<Word, 16>;
  Continuation resume{};
  Effects access;
  std::vector<Line> payload;
  Touches::Mask inputs{}, writes{};

  void apply(std::span<Word> ram) const noexcept {
    std::size_t index = 0;
    for (const auto [base, mask] : access.output()) {
      copy_words(ram.data() + base, payload[index++].data(), mask);
    }
  }
};

class Voice {
public:
  Voice(const Ram &initial,
        const Registers &start,
        blofeld::vi::dsp::BootRom rom,
        const NativeImage &image,
        const std::uint64_t variant)
      : lane(initial, start, rom, image, variant), base_(initial) {
    protect(lane.storage.mappings[geometry::voice_mapping].views, geometry::voice_base);
    for (auto &entry : footprints_) { entry.ordered_lines.reserve(words / line_words); }
  }

  void seed_job(const unsigned slot, Registers confirmed) noexcept {
    confirmed.r[0] = geometry::list_base + slot + 1;
    confirmed.r[5] = base_[geometry::low_x + geometry::list_base + slot];
    const auto pointer = confirmed.r[5];
    footprint_ = &footprints_[slot];
    if (footprint_->voice != pointer) {
      footprint_->voice = pointer;
      footprint_->lines.reset();
      footprint_->ordered_lines.clear();
    }
    confirmed.x = (confirmed.x & ~std::uint64_t{word_mask}) | pointer;
    confirmed.a = (static_cast<std::uint64_t>(
                       static_cast<std::int32_t>(pointer << accumulator_shift) >> accumulator_shift)
                   << dsp_word_bits) &
                  accumulator_mask;
    predicted_ = confirmed;
  }

  void prepare(const LineEpochs &line_epochs,
               const PageEpochs &page_epochs,
               const std::uint64_t short_credit,
               std::atomic<bool> &short_ready,
               const auto &still_pending) {
    restore_memory(line_epochs, page_epochs);
    tainted_ = true;
    lane.reset(predicted_);
    seed_ = predicted_;
    seed_.a <<= accumulator_shift;
    seed_.b <<= accumulator_shift;
    touches_.used = 0;
    touches_.buckets.fill(0);
    touches_.register_inputs.fill(0);
    touches_.register_writes.fill(0);
    lane.inspection.clear_irq_queries();
    active_touches = &touches_;
    RunResult result;
    try {
      if (short_credit != 0) {
        result = lane.core.run_until(short_credit);
        record_prefix(result, short_credit, line_epochs, short_ready);
      }
      if (short_credit == 0 || (!result.yielded && result.fault == Fault{} && still_pending())) {
        result = lane.core.run_until(worker_cycle_limit);
      }
    } catch (...) {
      active_touches = nullptr;
      throw;
    }
    active_touches = nullptr;
    effects.extract(lane.storage.physical, touched_pages(), touches_.used > page_capacity);
    const auto stopped_pc = lane.core.registers().pc;
    const auto mixer_exit = result.yielded && std::ranges::contains(mix, stopped_pc);
    // The isolated image declares the mixer boundary as a transfer. Device
    // callbacks reject writes and never request a yield. Capture revalidates
    // the native continuation before the owner is offered these effects.
    usable_ = lane.inspection.irq_queries() == 0 && result.fault == Fault{} && mixer_exit &&
              touches_.used <= page_capacity;
    // The tagged native code reports each read before its first write to the
    // word. A changed line outside the predicted footprint may alter the branch
    // path, so discard the entire speculative result and re-seed next time.
    bool staged_inputs = true;
    for (const auto &access : effects.input()) {
      const auto line = static_cast<std::size_t>(access.base) / line_words;
      staged_inputs &= staged_[line] == line_epochs[line];
      if (!footprint_->lines[line]) {
        footprint_->lines.set(line);
        footprint_->ordered_lines.push_back(static_cast<std::uint16_t>(line));
      }
    }
    usable_ = usable_ && staged_inputs;
    // All marked pages have been extracted and cleaned. The next attempt
    // restores stores and updates just its learned cache lines.
    tainted_ = false;
  }

  bool usable() const noexcept { return usable_; }

  const ShortReceipt &early() const noexcept { return short_; }

  const Registers &entry() const noexcept { return seed_; }

  const Touches::Mask &reads() const noexcept { return touches_.register_inputs; }

  const Touches::Mask &writes() const noexcept { return touches_.register_writes; }

  [[nodiscard]] Continuation continuation() const noexcept { return Continuation::read(lane.core); }

  [[nodiscard]] const Effects &changes() const noexcept { return effects; }

  void publish(const std::span<Word> destination, const bool early) const noexcept {
    if (early) {
      short_.apply(destination);
    } else {
      effects.writeback(destination, lane.storage.physical);
    }
  }

private:
  Lane lane;
  Effects effects;

  void restore_memory(const LineEpochs &line_epochs, const PageEpochs &page_epochs) {
    const auto target = lane.storage.physical;
    if (tainted_) {
      lane.assign(base_);
      staged_ = line_epochs;
      for (const auto page : program_pages) { program_versions_[page] = page_epochs[page]; }
    } else {
      effects.writeback(target, base_);
      for (const auto page : program_pages) {
        if (program_versions_[page] == page_epochs[page]) { continue; }
        const auto first = page * page_words;
        std::copy_n(base_.data() + first, page_words, target.data() + first);
        for (std::size_t line = page * lines_per_page; line < (page + 1) * lines_per_page; ++line) {
          staged_[line] = line_epochs[line];
        }
        program_versions_[page] = page_epochs[page];
      }
      for (const auto line : footprint_->ordered_lines) {
        const auto version = line_epochs[line];
        if (staged_[line] == version) { continue; }
        const auto first = static_cast<std::size_t>(line) * line_words;
        std::copy_n(base_.data() + first, line_words, target.data() + first);
        staged_[line] = version;
      }
    }
  }

  void record_prefix(const RunResult &result,
                     const std::uint64_t deadline,
                     const LineEpochs &line_epochs,
                     std::atomic<bool> &ready) {
    const auto target = lane.storage.physical;
    if (lane.inspection.irq_queries() == 0 && result.fault == Fault{} && !result.yielded &&
        result.state == ExecutionState::running && result.progress.estimated_cycles >= deadline &&
        result.progress.instructions != 0 && touches_.used <= page_capacity) {
      short_.access.extract(target, touched_pages(), false, false);
      if (have_inputs(short_.access, line_epochs)) {
        short_.resume = Continuation::read(lane.core);
        short_.inputs = touches_.register_inputs;
        short_.writes = touches_.register_writes;
        short_.payload.resize(short_.access.output().size());
        std::size_t index = 0;
        for (const auto [base, mask] : short_.access.output()) {
          capture_line(short_.payload[index++].data(), target.data() + base, mask);
        }
        ready.store(true, std::memory_order_release);
      }
    }
  }

  bool have_inputs(const Effects &probe, const LineEpochs &line_epochs) const {
    return std::ranges::all_of(probe.input(), [&](const auto access) {
      const auto line = static_cast<std::size_t>(access.base) / line_words;
      return staged_[line] == line_epochs[line];
    });
  }

  [[nodiscard]] std::optional<Word> physical_alias(const std::span<const Word> view,
                                                   const std::uintptr_t address,
                                                   const Space space) const noexcept {
    const auto base = reinterpret_cast<std::uintptr_t>(view.data());
    if (address < base || address - base >= view.size_bytes()) { return {}; }
    const auto offset = (address - base) / sizeof(Word);
    return lane.memory.physical_word(space, static_cast<Word>(offset));
  }

  [[nodiscard]] Word touched_word(const std::uintptr_t page) const {
    const auto &views = lane.storage.mappings[geometry::voice_mapping].views;
    for (const auto space : {Space::x, Space::y}) {
      const auto &window = views[static_cast<std::size_t>(space)];
      for (const auto &view : {window.read, window.write}) {
        if (const auto physical = physical_alias(view.data, page, space)) { return *physical; }
      }
    }
    throw std::runtime_error("A worker touched an alias outside physical DSP RAM");
  }

  std::bitset<words / page_words> touched_pages() const {
    std::bitset<words / page_words> pages;
    if (touches_.used > page_capacity) { return pages; }
    for (std::uint64_t i = 0; i < touches_.used; ++i) {
      const auto physical = touched_word(touches_.pages[i]);
      check(physical < words && physical % page_words == 0,
            "Resolve a touched private alias to its real backing page");
      pages.set(physical / page_words);
    }
    return pages;
  }

  Touches touches_;
  ShortReceipt short_;
  const Ram &base_;
  Registers predicted_, seed_{};
  bool usable_ = false, tainted_ = true;
  static constexpr std::array<std::size_t, 12> program_pages{
      0, 1, 2, 3, 40, 41, 42, 43, 44, 45, 46, 47};
  static_assert(words / line_words <= std::numeric_limits<std::uint16_t>::max());

  struct Footprint {
    Word voice = 0;
    std::bitset<words / line_words> lines;
    std::vector<std::uint16_t> ordered_lines;
  };

  std::array<Footprint, geometry::capacity> footprints_{};
  Footprint *footprint_ = nullptr;
  std::array<std::uint64_t, words / line_words> staged_{};
  std::array<std::uint64_t, words / page_words> program_versions_{};
};

bool writable_data(const Word word) noexcept {
  // OMR mapping 4 diverts only the high-X tail into P.
  return (word >= geometry::low_x && word < geometry::program_high) ||
         (word >= geometry::high_y && word < words);
}

}

namespace blofeld::app {
class DspWorkers::Impl {
public:
  Impl(const WorkerPartition &partition,
       const NativeImage &prepared,
       vi::dsp::BootRom boot,
       vi::dsp::Memory::Storage storage);

  void playback_ready() noexcept { playable_.store(true, std::memory_order_release); }

  void exclude_dsp_core(const cpu_set_t &physical) noexcept {
    cpu_set_t available{};
    if (::sched_getaffinity(0, sizeof(available), &available) != 0) { return; }
    for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
      if (CPU_ISSET(cpu, &physical)) { CPU_CLR(cpu, &available); }
    }
    if (CPU_COUNT(&available) == 0) { return; }
    // NOLINTNEXTLINE(misc-include-cleaner) POSIX exposes sched_param through <sched.h>.
    const sched_param realtime{.sched_priority = ::sched_get_priority_min(SCHED_RR)};
    for (auto &thread : workers_) {
      auto id = thread.native_handle();
      if (::pthread_setaffinity_np(id, sizeof(available), &available) != 0) { continue; }
      if (!placed_ && realtime.sched_priority >= 0) {
        static_cast<void>(::pthread_setschedparam(id, SCHED_RR, &realtime));
      }
    }
    placed_ = true;
  }

  const ForegroundCommit *
  offer(const blofeld::vi::dsp::Io &device, const Core &owner, const ForegroundGrant &grant) {
    if (grant.variant != variant_ || grant.pc != next_setup || canonical_x_.empty()) {
      return nullptr;
    }
    const auto registers = owner.registers();
    if (registers.omr != voice_mode) { return nullptr; }

    const auto cursor = registers.r[0];
    if (cursor <= geometry::list_base || cursor > geometry::list_base + geometry::capacity) {
      return nullptr;
    }

    const auto index = cursor - geometry::list_base - 1;

    if (!playable_.load(std::memory_order_acquire)) { return nullptr; }
    const auto progress = owner.progress();
    if (device.epoch() != epoch_) {
      epoch_ = device.epoch();
      last_start_ = std::numeric_limits<std::uint64_t>::max();
      last_packet_ = last_start_;
      most_recent_.reset();
      history_ = {};
      prefix_windows_ = {};
      requested_refresh_.set();
    }

    history_[index] = {.voice = registers.r[5], .y = registers.y, .b = registers.b};
    if (index == 0 && progress.estimated_cycles != last_start_) {
      begin(owner, registers, progress, grant.revision);
    }
    if (index > 0) { most_recent_ = DataRegisters{.x = registers.x, .y = registers.y}; }
    if (index != 0 && last_packet_ == last_start_) {
      passed_slot_.store(index, std::memory_order_relaxed);
    }
    return try_ready(device, owner, registers, progress, grant, index);
  }

  ~Impl() {
    for (auto &worker : workers_) { worker.request_stop(); }
    for (auto &worker : workers_) {
      if (worker.joinable()) { worker.join(); }
    }
  }

private:
  static unsigned voice_count(const std::span<const Word> x) noexcept {
    unsigned seen = 0;
    for (unsigned slot = 0; slot <= geometry::capacity; ++slot) {
      const auto pointer = x[geometry::list_base + slot];
      if (pointer == 0) { return slot; }
      if (slot == geometry::capacity || pointer < geometry::voice_base) { return 0; }
      const auto relative = pointer - geometry::voice_base;
      const auto index = relative / geometry::stride;
      if (relative % geometry::stride != 0 || index >= geometry::capacity ||
          (seen & (1U << index)) != 0) {
        return 0;
      }
      seen |= 1U << index;
      const auto level = x[pointer + geometry::amplifier];
      if (level == 0 || level == geometry::idle_callback || x[pointer + geometry::channel] >= 16) {
        return 0;
      }
    }
    return 0;
  }

  void begin(const Core &owner,
             Registers registers,
             const Progress progress,
             const std::uint64_t revision) {
    last_start_ = progress.estimated_cycles;
    if (state_.load(std::memory_order_acquire) == busy) {
      passed_slot_.store(geometry::capacity, std::memory_order_relaxed);
      return;
    }
    // Refresh generations can be extended by up to 256 future packets.
    constexpr auto last_generation = std::numeric_limits<std::uint64_t>::max() - 256;
    if (arm_count_ >= last_generation || last_packet_ == progress.estimated_cycles) { return; }
    const auto active_voices = voice_count(canonical_x_);
    if (active_voices <= 1) { return; }

    // The seed may contain prior values. Every original input read still
    // has to match canonical RAM before a result can be accepted.
    auto refresh = requested_refresh_;
    if (arm_count_ == 0) { refresh.set(); }
    for (std::size_t page = 0; page < refresh.size(); ++page) {
      if (refresh_through_[page] > arm_count_) { refresh.set(page); }
    }
    refresh.set(geometry::low_x / page_words);
    refresh.set((geometry::low_x + geometry::list_base) / page_words);
    refresh.set(geometry::low_y / page_words);
    prepare_jobs(registers, active_voices, refresh);
    const auto generation = arm_count_ + 1;
    for (std::size_t page = 0; page < refresh.size(); ++page) {
      if (!refresh[page]) { continue; }
      versions_[page] = generation;
      const auto first = page * lines_per_page;
      for (auto line = first; line < first + lines_per_page; ++line) {
        const auto offset = line * line_words;
        auto *const saved = seed_.data() + offset;
        const auto *contents = canonical_ram_.data() + offset;
        if (!different_words(saved, contents, full_line)) { continue; }
        std::memcpy(saved, contents, line_words * sizeof(Word));
        line_epochs_[line] = generation;
        page_epochs_[page] = generation;
      }
    }
    requested_refresh_.reset();
    owner_ = &owner;
    revision_ = revision;
    last_packet_ = progress.estimated_cycles;
    ++arm_count_;
    passed_slot_.store(0, std::memory_order_relaxed);
    remaining_.store(workers_count, std::memory_order_relaxed);
    state_.store(busy, std::memory_order_release);
    notification_.fetch_add(1, std::memory_order_release);
    notification_.notify_all();
  }

  void prepare_jobs(Registers registers,
                    const unsigned active_voices,
                    std::bitset<words / page_words> &refresh) {
    const auto list_x = canonical_x_;
    registers.pc = next_setup;
    if (most_recent_) {
      registers.x = (most_recent_->x & ~std::uint64_t{word_mask}) | (registers.x & word_mask);
      registers.y = most_recent_->y;
    }
    for (unsigned j = 0; j < jobs_.size(); ++j) {
      auto &job = jobs_[j];
      const auto index = j + 1;
      job.slot = index < active_voices ? index : geometry::capacity;
      job.ready.store(false, std::memory_order_relaxed);
      job.short_ready.store(false, std::memory_order_relaxed);
      job.used = false;
      if (job.slot >= geometry::capacity) { continue; }
      job.seed = registers;
      if (index < history_.size() && history_[index].voice != 0 &&
          history_[index].voice == list_x[geometry::list_base + index]) {
        const auto &prior = history_[index];
        job.seed.y = prior.y;
        job.seed.b = prior.b;
      }
      // Use the previous real grant from the same voice entry. Canonical
      // admission still checks the actual next boundary before publication.
      const auto previous_window = prefix_windows_[job.slot];
      job.short_credit = previous_window.valid_until > arm_count_ &&
                                 previous_window.window > instruction_slack_ &&
                                 previous_window.window < worker_cycle_limit &&
                                 previous_window.voice == list_x[geometry::list_base + job.slot]
                             ? previous_window.window - instruction_slack_
                             : 0;
      const auto pointer = list_x[geometry::list_base + job.slot];
      const auto physical = geometry::high_x + pointer - geometry::high_first;
      refresh.set(physical / page_words);
      refresh.set((physical + geometry::stride - 1) / page_words);
    }
  }

  void work(const std::stop_token &stop, const std::size_t index) noexcept {
    std::uint64_t generation = 0;
    while (!stop.stop_requested()) {
      notification_.wait(generation, std::memory_order_acquire);
      if (stop.stop_requested()) { break; }
      generation = notification_.load(std::memory_order_acquire);
      if (state_.load(std::memory_order_acquire) != busy) { continue; }
      for (std::size_t j = index; j < jobs_.size(); j += workers_count) {
        auto &job = jobs_[j];
        if (job.slot >= geometry::capacity) { continue; }
        // A stale cursor can only cause unnecessary speculative computation.
        // Admission is still owned by the DSP and rechecks the whole result.
        if (job.slot <= passed_slot_.load(std::memory_order_relaxed)) { continue; }
        try {
          auto &voice = *voices_[j];
          voice.seed_job(job.slot, job.seed);
          voice.prepare(line_epochs_, page_epochs_, job.short_credit, job.short_ready, [&] {
            return passed_slot_.load(std::memory_order_relaxed) <= job.slot;
          });
          job.ready.store(voice.usable(), std::memory_order_release);
        } catch (...) {
          // The owner resumes its ordinary prepared instructions when the
          // private lane fails to produce a complete receipt.
          job.ready.store(false, std::memory_order_release);
        }
      }
      // Acquire all participants' writes before making the snapshot reusable.
      if (remaining_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        state_.store(done, std::memory_order_release);
      }
    }
  }

  struct Candidate {
    Continuation continuation;
    const Effects &memory;
    const Touches::Mask &reads;
    const Touches::Mask &writes;
    bool prefix = false;
  };

  const ForegroundCommit *try_ready(const blofeld::vi::dsp::Io &device,
                                    const Core &owner,
                                    const Registers &registers,
                                    const Progress progress,
                                    const ForegroundGrant &grant,
                                    const unsigned slot) {
    if (slot == 0 || last_packet_ != last_start_ || state_.load(std::memory_order_acquire) == 0 ||
        &owner != owner_ || grant.revision != revision_ ||
        progress.estimated_cycles < last_packet_ ||
        grant.precise_cycles ==
            std::numeric_limits<std::uint64_t>::max() - progress.estimated_cycles) {
      return nullptr;
    }
    auto &job = jobs_[slot - 1];
    if (job.used || job.slot != slot) { return nullptr; }

    const auto full_ready = job.ready.load(std::memory_order_acquire);
    const auto early_ready = job.short_ready.load(std::memory_order_acquire);
    if (!full_ready && !early_ready) { return nullptr; }
    job.used = true;

    const auto &voice = *voices_[slot - 1];
    if (full_ready) {
      const auto *result = commit(device,
                                  registers,
                                  grant,
                                  slot,
                                  voice.entry(),
                                  {.continuation = voice.continuation(),
                                   .memory = voice.changes(),
                                   .reads = voice.reads(),
                                   .writes = voice.writes()});
      if (result != nullptr) { return result; }
    }
    if (!early_ready) { return nullptr; }
    const auto &early = voice.early();
    return commit(device,
                  registers,
                  grant,
                  slot,
                  voice.entry(),
                  {.continuation = early.resume,
                   .memory = early.access,
                   .reads = early.inputs,
                   .writes = early.writes,
                   .prefix = true});
  }

  const ForegroundCommit *commit(const blofeld::vi::dsp::Io &device,
                                 Registers registers,
                                 const ForegroundGrant &grant,
                                 const unsigned slot,
                                 const Registers &entry,
                                 Candidate candidate) {
    auto &output = candidate.continuation;
    const auto count = output.instructions;
    const auto duration = output.cycles;
    const auto window = grant.precise_cycles;
    if (!output.admissible || output.registers.omr != registers.omr || count == 0 ||
        count > grant.instructions || duration == 0) {
      return nullptr;
    }
    if (duration > grant.foreground_cycles) {
      if (!candidate.prefix && window > instruction_slack_ && window < duration) {
        prefix_windows_[slot] = {
            .window = window, .valid_until = arm_count_ + 5, .voice = registers.r[5]};
      }
      return nullptr;
    }
    const bool crossing = duration > window;
    registers.a <<= accumulator_shift;
    registers.b <<= accumulator_shift;
    auto before = registers;
    before.pc = next_setup;
    // The verified first three original instructions end with the full
    // B move. The worker ran them without IRQ, fault or program transition.
    const bool b_replaced = count >= 3;
    if (candidate.prefix && crossing) { return nullptr; }
    if (different_registers(
            {.predicted = entry, .canonical = before}, candidate.reads, b_replaced)) {
      return nullptr;
    }
    if (!valid_memory(candidate.memory)) { return nullptr; }
    if (crossing &&
        !admit_audio_crossing(device, registers, slot, candidate.memory, window, duration)) {
      return nullptr;
    }
    const auto b = output.registers.b;
    preserve_registers(output.registers, registers, candidate.writes);
    if (b_replaced) { output.registers.b = b; }
    output.registers.a >>= accumulator_shift;
    output.registers.b >>= accumulator_shift;

    publishing_slot_ = slot;
    publishing_early_ = candidate.prefix;
    receipt_ = {.registers = output.registers,
                .work = {.instructions = count, .estimated_cycles = duration},
                .last_pc = output.current_pc,
                .publish = [](void *owner) noexcept { static_cast<Impl *>(owner)->publish(); },
                .context = this};
    return &receipt_;
  }

  [[nodiscard]] bool valid_memory(const Effects &effects) {
    for (const auto [base, reads] : effects.input()) {
      if (different_words(canonical_ram_.data() + base, seed_.data() + base, reads)) {
        const auto page = base / page_words;
        if (versions_[page] < arm_count_) {
          if (last_stale_[page] != 0 && arm_count_ - last_stale_[page] < 64) {
            refresh_through_[page] = arm_count_ + 256;
          }
          last_stale_[page] = arm_count_;
          requested_refresh_.set(page);
        }
        return false;
      }
    }
    return std::ranges::all_of(effects.output(),
                               [](const auto line) { return writable_data(line.base); });
  }

  void publish() noexcept {
    voices_[publishing_slot_ - 1]->publish(canonical_ram_, publishing_early_);
  }

  bool admit_audio_crossing(const blofeld::vi::dsp::Io &device,
                            const Registers &registers,
                            const unsigned slot,
                            const Effects &effects,
                            const std::uint64_t window,
                            const std::uint64_t duration) {
    using blofeld::vi::dsp::ForegroundCrossing;
    const auto result = device.foreground_crossing(
        window,
        duration,
        {.context = &effects, .contains = [](const void *context, const Word word) noexcept {
           return static_cast<const Effects *>(context)->writes_word(word);
         }});
    if (result == ForegroundCrossing::timer_effect) {
      prefix_windows_[slot] = {
          .window = window, .valid_until = arm_count_ + 5, .voice = registers.r[5]};
    }
    return result == ForegroundCrossing::quiet;
  }

  struct PrefixWindow {
    std::uint64_t window = 0, valid_until = 0;
    Word voice = 0;
  };

  std::array<PrefixWindow, geometry::capacity> prefix_windows_{};
  const std::span<Word> canonical_ram_;
  const std::span<const Word> canonical_x_;
  std::atomic<bool> playable_{false};
  std::vector<Word> boot_main_, boot_eeprom_;
  blofeld::vi::dsp::BootRom rom_;
  const std::uint64_t variant_;
  Ram seed_{};
  std::bitset<words / page_words> requested_refresh_;
  LineEpochs line_epochs_{};
  PageEpochs page_epochs_{}, versions_{}, last_stale_{}, refresh_through_{};

  struct Job {
    Registers seed{};
    unsigned slot = 0;
    std::uint64_t short_credit = 0;
    bool used = false;
    std::atomic<bool> ready{false}, short_ready{false};
  };

  static constexpr unsigned workers_count = 2;
  std::array<Job, geometry::capacity - 1> jobs_;
  std::array<std::unique_ptr<Voice>, geometry::capacity - 1> voices_;
  static constexpr unsigned busy = 1, done = 2;
  std::atomic<unsigned> state_{0}, remaining_{0};
  std::atomic<std::uint64_t> notification_{0};
  alignas(64) std::atomic<unsigned> passed_slot_{0};
  std::array<std::jthread, workers_count> workers_;
  bool placed_ = false;
  const Core *owner_ = nullptr;
  std::uint64_t epoch_ = 0;
  unsigned publishing_slot_ = 0;
  bool publishing_early_ = false;
  ForegroundCommit receipt_{};

  struct DataRegisters {
    std::uint64_t x, y;
  };

  std::optional<DataRegisters> most_recent_;

  struct EarlierVoice {
    Word voice = 0;
    std::uint64_t y = 0, b = 0;
  };

  std::array<EarlierVoice, geometry::capacity> history_{};
  std::uint64_t instruction_slack_ = 0;
  std::uint64_t revision_ = 0, last_packet_ = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t last_start_ = std::numeric_limits<std::uint64_t>::max();

  std::uint64_t arm_count_ = 0;
};

DspWorkers::Impl::Impl(const WorkerPartition &partition,
                       const NativeImage &prepared,
                       const vi::dsp::BootRom boot,
                       const vi::dsp::Memory::Storage storage)
    : canonical_ram_(storage.physical),
      canonical_x_(storage.mappings[geometry::voice_mapping].views[1].read.data),
      boot_main_(boot.main.begin(), boot.main.end()),
      boot_eeprom_(boot.eeprom.begin(), boot.eeprom.end()),
      rom_({.main = boot_main_, .eeprom = boot_eeprom_}),
      variant_(partition.program.variant) {
  instruction_slack_ = partition.instruction_slack;
  const auto fill = [&](const CodeSpan &block) {
    check(block.address < geometry::program_end &&
              block.words.size() <= geometry::program_end - block.address,
          "Prepare the native worker's leased original code");
    for (std::size_t i = 0; i < block.words.size(); ++i) {
      const auto pc = block.address + i;
      const auto physical =
          pc < geometry::program_low ? pc : geometry::program_high + (pc - geometry::program_low);
      seed_[physical] = block.words[i];
    }
  };
  for (const auto span : partition.program.spans) { fill(span); }
  for (const auto span : partition.exit_spans) { fill(span); }
  const Registers start{.pc = next_setup, .omr = voice_mode};
  for (auto &voice : voices_) {
    voice = std::make_unique<Voice>(seed_, start, rom_, prepared, variant_);
  }
  for (std::size_t index = 0; index < workers_.size(); ++index) {
    workers_[index] = std::jthread([this, index](const std::stop_token &stop) {
      const std::stop_callback wake_on_stop(stop, [this] {
        notification_.fetch_add(1, std::memory_order_release);
        notification_.notify_all();
      });
      static_cast<void>(::pthread_setname_np(::pthread_self(),
                                             index == 0 ? "blofeld-stage-0" : "blofeld-stage-1"));
      work(stop, index);
    });
  }
}

DspWorkers::DspWorkers(const vi::dsp::Memory::Storage storage) : storage_(storage) {}

DspWorkers::~DspWorkers() = default;

void DspWorkers::prepare(const CodeImage image,
                         const vi::dsp::BootRom boot,
                         const llvm::json::Object &plan,
                         const std::filesystem::path &cache,
                         const native::Preparation preparation) {
  const auto id = plan.getInteger("variant");
  if (!id || *id < 0) {
    throw std::invalid_argument("The foreground worker plan requires its source variant");
  }
  const auto worker_variant = static_cast<std::uint64_t>(*id);
  const auto variant = std::ranges::find(image.variants, worker_variant, &CodeVariant::id);
  check(!impl_ && boot.valid() && variant != image.variants.end(),
        "The foreground worker requires the original variant and boot ROM");
  const auto selection = partition(*variant, plan);
  const auto prepared = load_worker(selection, cache, preparation);
  if (preparation == native::Preparation::allow) { return; }
  const auto index = geometry::voice_mapping;
  check(storage_.physical.size() == words && index < storage_.mappings.size() &&
            storage_.mappings[index].views[1].read.data.size() == address_words,
        "The foreground worker requires direct, borrowed DSP RAM");
  impl_ = std::make_unique<Impl>(selection, prepared, boot, storage_);
}

vi::dsp::Foreground DspWorkers::foreground() noexcept {
  if (!impl_) { return {}; }
  return {.context = impl_.get(),
          .offer = [](void *self,
                      vi::dsp::Io &device,
                      const Core &owner,
                      const ForegroundGrant &grant) noexcept -> const ForegroundCommit * {
            if (active_touches != nullptr) { return nullptr; }
            return static_cast<Impl *>(self)->offer(device, owner, grant);
          }};
}

void DspWorkers::playback_ready(void *context) noexcept {
  if (context != nullptr) { static_cast<Impl *>(context)->playback_ready(); }
}

void DspWorkers::exclude_dsp_core(const cpu_set_t &physical) noexcept {
  if (impl_) { impl_->exclude_dsp_core(physical); }
}
}
