// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08.hpp"

#include "address.hh"
#include "blofeld/bus.hpp"
#include "error.hh"
#include "globalcontext.hh"
#include "hcs08_ccr.hpp"
#include "hcs08_native.hpp"
#include "hcs08_registers.hpp"
#include "memstate.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "types.h"
#include "xml.hh"

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace blofeld {
namespace native = detail::hcs08;

namespace {
constexpr unsigned fixed_ccr = 0x60;

std::uint16_t read_reset_vector(Bus &bus, const ExecutionContext context) {
  bus.begin_instruction(context);
  bus.begin_reset(context);
  constexpr unsigned vector = 0xfffe;
  std::uint16_t value = 0;
  try {
    value = static_cast<std::uint16_t>(bus.read_memory({.address = vector, .size = 2}));
  } catch (...) {
    bus.end_reset();
    throw;
  }
  bus.end_reset();
  return value;
}
}

class Hcs08::Impl {
public:
  Impl(Bus &input, const std::filesystem::path &specification)
      : bus(input), image(input), translator(&image, &context), memory(&translator) {
    std::istringstream configuration("<sleigh>" + native::xml_text(specification.string()) +
                                     "</sleigh>");
    ghidra::DocumentStorage documents;
    auto *root = documents.parseDocument(configuration)->getRoot();
    documents.registerTag(root);
    translator.initialize(documents);
    layout = native::register_layout(translator);
    ram = std::make_unique<native::BusBank>(translator.getDefaultCodeSpace(), bus);
    registers = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getSpaceByName("register"), 8, 256, 1024, nullptr);
    temporaries = std::make_unique<ghidra::MemoryHashOverlay>(
        translator.getUniqueSpace(), 8, 4096, 4096, nullptr);
    memory.setMemoryBank(ram.get());
    memory.setMemoryBank(registers.get());
    memory.setMemoryBank(temporaries.get());
    executor = std::make_unique<native::InstructionExecutor>(bus, translator, memory, layout.pc);
  }

  void refresh(const bool force = false) {
    const auto revision = bus.code_revision();
    if (!force && decoded_revision == revision) { return; }
    executor->clear_translations();
    translator.reset(&image, &context);
    ghidra::DocumentStorage documents;
    translator.initialize(documents);
    layout = native::register_layout(translator);
    decoded_revision = revision;
  }

  void normalize_ccr() { memory.setValue(&layout.ccr, memory.getValue(&layout.ccr) | fixed_ccr); }

  Bus &bus;
  native::CodeImage image;
  ghidra::ContextInternal context;
  ghidra::Sleigh translator;
  ghidra::MemoryState memory;
  native::RegisterLayout layout{};
  std::unique_ptr<native::BusBank> ram;
  std::unique_ptr<ghidra::MemoryHashOverlay> registers;
  std::unique_ptr<ghidra::MemoryHashOverlay> temporaries;
  std::unique_ptr<native::InstructionExecutor> executor;
  std::uint64_t decoded_revision = 0;
  std::uint8_t ccr_known = Hcs08State::all_ccr;
  std::uint8_t a_known = Hcs08State::all_a;
  std::uint16_t hx_known = Hcs08State::all_hx;
  bool pc_known = true;
  bool active = false;
  bool healthy = false;
  bool initialized = false;
};

Hcs08::Hcs08(Bus &bus, const std::filesystem::path &specification) try
    : impl_(std::make_unique<Impl>(bus, specification)) {
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot initialize HCS08 semantics: " + error.explain);
}

Hcs08::~Hcs08() = default;

void Hcs08::set_state(const Hcs08State &state) {
  const native::ExecutionGuard guard(impl_->active);
  impl_->healthy = false;
  impl_->a_known = state.a_known;
  impl_->hx_known = state.hx_known;
  impl_->memory.setValue(&impl_->layout.a, state.a & state.a_known);
  impl_->memory.setValue(&impl_->layout.hx, state.hx & state.hx_known);
  impl_->memory.setValue(&impl_->layout.sp, state.sp);
  impl_->ccr_known = static_cast<std::uint8_t>(state.ccr_known | fixed_ccr);
  impl_->memory.setValue(&impl_->layout.ccr, (state.ccr | fixed_ccr) & impl_->ccr_known);
  impl_->pc_known = state.pc_known;
  const auto pc = state.pc_known ? state.pc : std::uint16_t{0};
  impl_->memory.setValue(&impl_->layout.pc, pc);
  impl_->executor->setExecuteAddress(ghidra::Address(impl_->translator.getDefaultCodeSpace(), pc));
  impl_->executor->clear_background_request();
  impl_->initialized = true;
  impl_->healthy = true;
}

Hcs08State Hcs08::state() const {
  if (!impl_->initialized) { throw Fault("HCS08 entry registers have not been supplied"); }
  return {
      .pc = static_cast<std::uint16_t>(impl_->executor->getExecuteAddress().getOffset()),
      .hx = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.hx) & impl_->hx_known),
      .sp = static_cast<std::uint16_t>(impl_->memory.getValue(&impl_->layout.sp)),
      .a = static_cast<std::uint8_t>(impl_->memory.getValue(&impl_->layout.a) & impl_->a_known),
      .ccr_known = impl_->ccr_known,
      .ccr =
          static_cast<std::uint8_t>(impl_->memory.getValue(&impl_->layout.ccr) & impl_->ccr_known),
      .pc_known = impl_->pc_known,
      .a_known = impl_->a_known,
      .hx_known = impl_->hx_known};
}

std::string Hcs08::disassemble() const try {
  if (!state().pc_known) {
    throw Fault("HCS08 disassemble address=unknown pc=unknown pc-known=0: unknown CPU PC");
  }
  impl_->refresh();
  native::Assembly assembly;
  impl_->translator.printAssembly(assembly, impl_->executor->getExecuteAddress());
  return assembly.text;
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot disassemble HCS08 at " + hex(state().pc) + ": " + error.explain);
}

bool Hcs08::background_requested() const {
  return impl_->executor->background_requested();
}

bool Hcs08::faulted() const {
  return !impl_->healthy;
}

Hcs08Checkpoint Hcs08::checkpoint() const {
  if (impl_->active || !impl_->healthy || !impl_->initialized) {
    throw Fault("HCS08 checkpoint requires a healthy completed native boundary");
  }
  Hcs08Checkpoint result;
  std::map<ghidra::VarnodeData, std::string> registers;
  impl_->translator.getAllRegisters(registers);
  const auto *space = impl_->translator.getSpaceByName("register");
  for (const auto &[reg, name] : registers) {
    if (reg.space == space &&
        (reg.offset >= result.registers_.size() ||
         static_cast<ghidra::uintb>(reg.size) > result.registers_.size() - reg.offset)) {
      throw Fault("HCS08 checkpoint cannot represent native register " + name);
    }
  }
  impl_->registers->getChunk(
      0, static_cast<ghidra::int4>(result.registers_.size()), result.registers_.data());
  result.pc_ = state().pc;
  result.background_ = background_requested();
  result.ccr_known_ = impl_->ccr_known;
  result.pc_known_ = impl_->pc_known;
  result.a_known_ = impl_->a_known;
  result.hx_known_ = impl_->hx_known;
  return result;
}

void Hcs08::restore_checkpoint(const Hcs08Checkpoint &checkpoint) {
  const native::ExecutionGuard guard(impl_->active);
  impl_->healthy = false;
  impl_->refresh(true);
  impl_->registers->setChunk(
      0, static_cast<ghidra::int4>(checkpoint.registers_.size()), checkpoint.registers_.data());
  impl_->executor->setExecuteAddress(
      ghidra::Address(impl_->translator.getDefaultCodeSpace(), checkpoint.pc_));
  impl_->executor->restore_background_request(checkpoint.background_);
  impl_->ccr_known = checkpoint.ccr_known_;
  impl_->pc_known = checkpoint.pc_known_;
  impl_->a_known = checkpoint.a_known_;
  impl_->hx_known = checkpoint.hx_known_;
  impl_->initialized = true;
  impl_->healthy = true;
}

void Hcs08::reset(const Hcs08ResetEntry mode, const Hcs08ResetCause cause) {
  if (mode != Hcs08ResetEntry::normal && mode != Hcs08ResetEntry::active_background) {
    throw Fault("HCS08 reset requires a qualified architectural entry mode");
  }
  if (cause != Hcs08ResetCause::warm && cause != Hcs08ResetCause::power_on) {
    throw Fault("HCS08 reset requires a qualified architectural reset cause");
  }
  if (impl_->active || (impl_->initialized && !impl_->healthy) ||
      (!impl_->initialized && cause != Hcs08ResetCause::power_on)) {
    throw Fault("HCS08 reset requires healthy supplied pre-reset A/X and coordinated board state");
  }
  const auto before = impl_->initialized ? state() : Hcs08State{.pc_known = false};
  auto entry = before;
  if (cause == Hcs08ResetCause::power_on) {
    entry.a = 0;
    entry.a_known = 0;
    entry.hx = 0;
    entry.hx_known = 0;
  }
  {
    const native::ExecutionGuard guard(impl_->active);
    impl_->healthy = false;
    entry.pc_known = mode == Hcs08ResetEntry::normal;
    entry.pc = 0; // Canonical representation of an unknown PC, never a fetch address.
    if (entry.pc_known) {
      constexpr unsigned vector = 0xfffe;
      try {
        entry.pc = read_reset_vector(impl_->bus, {.pc = before.pc, .ccr = before.ccr});
      } catch (const Fault &error) {
        throw Fault("HCS08 reset vector read address=" + hex(vector) +
                    " pc=" + (before.pc_known ? hex(before.pc) : "unknown pc-known=0") + ": " +
                    error.what());
      }
    }
  }
  constexpr unsigned low_mask = 0xff;
  constexpr unsigned high_mask = 0xff00;
  constexpr unsigned reset_known = 0x68;
  entry.hx &= low_mask;
  entry.hx_known = static_cast<std::uint16_t>((entry.hx_known & low_mask) | high_mask);
  entry.sp = low_mask;
  entry.ccr = reset_known;
  entry.ccr_known = reset_known;
  set_state(entry);
}

void Hcs08::step() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (background_requested()) { throw Fault("HCS08 BGND request requires an owning BDC decision"); }
  impl_->healthy = false;
  const auto before = state();
  if (!before.pc_known) {
    throw Fault("HCS08 execute address=unknown pc=unknown pc-known=0 ccr=" + hex(before.ccr) +
                ": unknown CPU PC");
  }
  try {
    impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
    if (impl_->bus.interrupt_request(before.ccr)) {
      throw Fault("HCS08 interrupt entry is not qualified");
    }
    const auto registers = detail::hcs08::registers_after(impl_->bus, before);
    const auto known = detail::hcs08::ccr_after(impl_->bus, before);
    impl_->refresh();
    impl_->executor->execute();
    impl_->ccr_known = known;
    impl_->normalize_ccr();
    impl_->bus.retire_instruction();
    impl_->a_known = registers.a;
    impl_->hx_known = registers.hx;
    impl_->healthy = true;
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HCS08 execution pc=" + hex(before.pc) + ": " + error.explain);
  } catch (const Fault &error) {
    throw Fault("HCS08 execution pc=" + hex(before.pc) + ": " + error.what());
  }
}
} // namespace blofeld
