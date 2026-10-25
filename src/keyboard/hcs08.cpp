// SPDX-License-Identifier: GPL-3.0-only
#include "keyboard/hcs08.hpp"

#include "address.hh"
#include "common/bus.hpp"
#include "error.hh"
#include "globalcontext.hh"
#include "keyboard/hcs08_ccr.hpp"
#include "keyboard/hcs08_interrupts.hpp"
#include "keyboard/hcs08_known_memory.hpp"
#include "keyboard/hcs08_native.hpp"
#include "keyboard/hcs08_registers.hpp"
#include "memstate.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "types.h"
#include "xml.hh"

#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace blofeld {
namespace native = detail::hcs08;

namespace {
constexpr unsigned fixed_ccr = 0x60, address_space = 65536;

struct InstructionIdentity {};

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
    executor = std::make_unique<native::InstructionExecutor>(
        bus,
        translator,
        memory,
        layout,
        native::ReturnKnowledge{.ccr = ccr_known, .a = a_known, .hx = hx_known});
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

  void push_irq_byte(const Hcs08KnownByte byte) {
    const auto sp = static_cast<std::uint16_t>(memory.getValue(&layout.sp));
    if (byte.known == Hcs08KnownByte::all) {
      bus.write(sp, byte.value);
    } else {
      auto *known = dynamic_cast<Hcs08KnownMemory *>(&bus);
      if (known == nullptr) { throw Fault("HCS08 IRQ frame requires masked RAM storage"); }
      known->write_known_byte({.address = sp, .value = byte.value}, byte.known);
    }
    memory.setValue(&layout.sp, static_cast<std::uint16_t>(sp + address_space - 1U));
  }

  void require_irq_storage(const Hcs08State &before) const {
    constexpr unsigned byte_mask = 0xff;
    if ((before.ccr_known != Hcs08State::all_ccr || before.a_known != Hcs08State::all_a ||
         (before.hx_known & byte_mask) != byte_mask) &&
        dynamic_cast<Hcs08KnownMemory *>(&bus) == nullptr) {
      throw Fault("HCS08 IRQ frame write address=" +
                  hex(static_cast<std::uint16_t>(before.sp + address_space - 4U)) +
                  " pc=" + hex(before.pc) + " ccr=" + hex(before.ccr) +
                  ": unknown register bits require explicit masked RAM storage");
    }
  }

  void enter_interrupt(const Hcs08State &before, const std::uint16_t vector) {
    constexpr unsigned byte_mask = 0xff;
    constexpr unsigned shift = 8;
    constexpr unsigned interrupt = 8;
    require_irq_storage(before);
    bus.begin_interrupt({.vector = vector, .kind = InterruptKind::maskable}, false);
    push_irq_byte({.value = static_cast<std::uint8_t>(before.pc & byte_mask)});
    push_irq_byte({.value = static_cast<std::uint8_t>(before.pc >> shift)});
    push_irq_byte({.value = static_cast<std::uint8_t>(before.hx & byte_mask),
                   .known = static_cast<std::uint8_t>(before.hx_known & byte_mask)});
    push_irq_byte({.value = before.a, .known = before.a_known});
    push_irq_byte({.value = before.ccr, .known = before.ccr_known});
    ccr_known = static_cast<std::uint8_t>(before.ccr_known | interrupt);
    memory.setValue(&layout.ccr, before.ccr | interrupt);
    const auto target = static_cast<std::uint16_t>(bus.read_memory({.address = vector, .size = 2}));
    memory.setValue(&layout.pc, target);
    executor->setExecuteAddress(ghidra::Address(translator.getDefaultCodeSpace(), target));
    bus.acknowledge_interrupt(vector);
  }

  void irq_stack_cycle(Hcs08IrqBusState &sequence, const unsigned phase) {
    const auto &s = sequence.entry;
    constexpr unsigned byte_mask = 0xff;
    constexpr unsigned shift = 8;
    constexpr unsigned interrupt = 8;
    const std::array<Hcs08KnownByte, 5> bytes{
        {{.value = static_cast<std::uint8_t>(s.pc & byte_mask)},
         {.value = static_cast<std::uint8_t>(s.pc >> shift)},
         {.value = static_cast<std::uint8_t>(s.hx & byte_mask),
          .known = static_cast<std::uint8_t>(s.hx_known & byte_mask)},
         {.value = s.a, .known = s.a_known},
         {.value = s.ccr, .known = s.ccr_known}}};
    if (phase == 1) {
      bus.begin_interrupt({.vector = sequence.admitted_vector, .kind = InterruptKind::maskable},
                          false);
    }
    push_irq_byte(bytes.at(phase - 1U));
    if (phase == bytes.size()) {
      ccr_known = static_cast<std::uint8_t>(s.ccr_known | interrupt);
      memory.setValue(&layout.ccr, s.ccr | interrupt);
    }
  }

  void irq_vector_cycle(Hcs08IrqBusState &sequence, const unsigned phase) {
    constexpr unsigned high_cycle = 6;
    if (phase == high_cycle) {
      const auto selected = native::pending_interrupt(bus, sequence.entry);
      if (!selected) {
        throw Fault("HCS08 IRQ vector select address=" + hex(sequence.admitted_vector) +
                    ": source withdrawal before vector fetch is unqualified");
      }
      sequence.vector = *selected;
      sequence.vector_selected = true;
    }
    if (!sequence.vector_selected) {
      throw Fault("HCS08 IRQ vector read requires a selected source");
    }
    const auto index = phase - high_cycle;
    const auto address = static_cast<std::uint16_t>(sequence.vector + index);
    sequence.vector_bytes.at(index) =
        static_cast<std::uint8_t>(bus.read_memory({.address = address}));
    ++sequence.vector_reads;
    if (index == 1) {
      bus.acknowledge_interrupt(sequence.vector);
      sequence.acknowledged = true;
    }
  }

  void irq_free_cycle(Hcs08IrqBusState &sequence) {
    constexpr unsigned shift = 8;
    if (sequence.vector_reads != sequence.vector_bytes.size()) {
      throw Fault("HCS08 IRQ free cycle requires both actual vector reads");
    }
    // No CPU memory access is invented for f. The vector propagation interval
    // completes before the first consecutive program-byte request.
    sequence.target = static_cast<std::uint16_t>(
        (static_cast<unsigned>(sequence.vector_bytes[0]) << shift) | sequence.vector_bytes[1]);
    sequence.target_known = true;
    memory.setValue(&layout.pc, sequence.target);
    executor->setExecuteAddress(ghidra::Address(translator.getDefaultCodeSpace(), sequence.target));
  }

  void irq_program_cycle(Hcs08IrqBusState &sequence, const unsigned phase) {
    constexpr unsigned first_fetch = 9;
    if (!sequence.target_known) {
      throw Fault("HCS08 IRQ program fetch requires its actual target");
    }
    const auto index = phase - first_fetch;
    const auto address = static_cast<std::uint16_t>(sequence.target + index);
    bus.check_execute({.address = address, .length = 1});
    sequence.prefetched.at(index) =
        static_cast<std::uint8_t>(bus.read_memory({.address = address}));
    ++sequence.prefetched_bytes;
    sequence.program_byte_in_flight = true;
  }

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
  std::optional<Hcs08IrqBusState> irq_bus;
  std::optional<Hcs08InstructionReservation> instruction;
  const std::shared_ptr<const void> instruction_identity =
      std::make_shared<const InstructionIdentity>();
  // Ownership metadata is not emulated state: never rewind future admissions.
  std::uint64_t instruction_epoch = 0;
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
  impl_->irq_bus.reset();
  impl_->instruction.reset();
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
  result.irq_bus_ = impl_->irq_bus;
  result.instruction_ = impl_->instruction;
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
  impl_->irq_bus = checkpoint.irq_bus_;
  impl_->instruction = checkpoint.instruction_;
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

std::optional<std::uint16_t> Hcs08::pending_interrupt() const {
  if (impl_->active || !impl_->healthy || !impl_->initialized || impl_->irq_bus) {
    throw Fault("HCS08 IRQ query requires a healthy completed native boundary");
  }
  if (impl_->instruction) { return {}; } // Finish the accepted instruction before IRQ admission.
  return native::pending_interrupt(impl_->bus, state());
}

bool Hcs08::service_interrupt() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (background_requested()) { throw Fault("HCS08 BGND request requires an owning BDC decision"); }
  if (impl_->instruction) { return false; }
  impl_->healthy = false;
  const auto before = state();
  try {
    if (impl_->irq_bus) { throw Fault("HCS08 IRQ bus sequence/prefetch still has an owner"); }
    const auto vector = native::pending_interrupt(impl_->bus, before);
    if (!vector) {
      impl_->healthy = true;
      return false;
    }
    impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
    impl_->enter_interrupt(before, *vector);
    impl_->healthy = true;
    return true;
  } catch (const Fault &error) {
    throw Fault("HCS08 interrupt entry pc=" + hex(before.pc) + ": " + error.what());
  }
}

std::optional<Hcs08IrqBusState> Hcs08::irq_bus_state() const {
  return impl_->irq_bus;
}

bool Hcs08::start_irq_bus(const Hcs08IrqBusContract contract) {
  if (contract != Hcs08IrqBusContract::reference_unstretched_sequence) {
    throw Fault("HCS08 IRQ bus requires the explicit unstretched sequence contract");
  }
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (impl_->instruction) { return false; }
  impl_->healthy = false;
  const auto before = state();
  if (background_requested() || impl_->irq_bus) {
    throw Fault("HCS08 IRQ bus admission requires an unowned normal-running CPU boundary");
  }
  const auto vector = native::pending_interrupt(impl_->bus, before);
  if (!vector) {
    impl_->healthy = true;
    return false;
  }
  impl_->require_irq_storage(before);
  impl_->irq_bus = Hcs08IrqBusState{.entry = before, .admitted_vector = *vector};
  impl_->healthy = true;
  return true;
}

void Hcs08::advance_irq_bus() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  impl_->healthy = false;
  constexpr unsigned vector_high = 6;
  constexpr unsigned free = 8;
  constexpr unsigned last = 11;
  auto &owner = impl_->irq_bus;
  if (!owner || owner->completed_cycles >= last) {
    throw Fault("HCS08 IRQ bus advance requires an unfinished owned sequence; prefetch consumption "
                "remains unqualified");
  }
  auto &sequence = *owner;
  const auto phase = sequence.completed_cycles + 1;
  ++sequence.attempted_cycles;
  // Previous program data reaches the CPU one user-view bus period later.
  // Keep that known arrival even if this period's new access then faults.
  sequence.ready_program_bytes = sequence.prefetched_bytes;
  sequence.program_byte_in_flight = false;
  const auto before = state();
  try {
    impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
    if (phase < vector_high) {
      impl_->irq_stack_cycle(sequence, phase);
    } else if (phase < free) {
      impl_->irq_vector_cycle(sequence, phase);
    } else if (phase == free) {
      impl_->irq_free_cycle(sequence);
    } else {
      impl_->irq_program_cycle(sequence, phase);
    }
    sequence.completed_cycles = phase;
    impl_->healthy = true;
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HCS08 IRQ bus cycle=" + std::to_string(phase) + " pc=" + hex(before.pc) + ": " +
                error.explain);
  } catch (const Fault &error) {
    throw Fault("HCS08 IRQ bus cycle=" + std::to_string(phase) + " pc=" + hex(before.pc) +
                " ccr=" + hex(before.ccr) + ": " + error.what());
  }
}

bool Hcs08::instruction_reserved() const {
  return impl_->instruction.has_value();
}

bool Hcs08InstructionReservation::matches_context(const Hcs08State &before,
                                                  const std::uint64_t code_revision) const {
  return before_ == before && code_revision_ == code_revision;
}

bool Hcs08::matches_reservation(const Hcs08InstructionReservation &reservation) const {
  const auto &owned = impl_->instruction;
  if (!owned) { return false; }
  return *owned == reservation && state() == reservation.before_ &&
         impl_->bus.code_revision() == reservation.code_revision_;
}

bool Hcs08::reservation_matches(const Hcs08InstructionReservation &reservation) const {
  return !impl_->active && impl_->healthy && impl_->initialized && matches_reservation(reservation);
}

std::optional<Hcs08InstructionReservation>
Hcs08::reserve_instruction(const Hcs08InstructionContract contract) {
  if (contract != Hcs08InstructionContract::reference_instruction_retirement) {
    throw Fault("HCS08 instruction admission requires the explicit retirement reference contract");
  }
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (impl_->instruction || impl_->irq_bus || background_requested()) { return {}; }
  impl_->healthy = false;
  const auto before = state();
  if (!before.pc_known) {
    throw Fault(
        "HCS08 instruction admission address=unknown pc=unknown pc-known=0: unknown CPU PC");
  }
  impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
  if (native::pending_interrupt(impl_->bus, before)) {
    impl_->healthy = true;
    return {};
  }
  if (impl_->instruction_epoch == std::numeric_limits<std::uint64_t>::max()) {
    throw Fault("HCS08 instruction admission address=" + hex(before.pc) + " pc=" + hex(before.pc) +
                ": reservation identity counter exhausted");
  }
  Hcs08InstructionReservation reservation;
  reservation.identity_ = impl_->instruction_identity;
  reservation.epoch_ = ++impl_->instruction_epoch;
  reservation.code_revision_ = impl_->bus.code_revision();
  reservation.before_ = before;
  impl_->instruction = reservation;
  impl_->healthy = true;
  return reservation;
}

void Hcs08::retire_reserved_instruction(const Hcs08InstructionReservation &reservation) {
  execute_instruction(&reservation);
}

void Hcs08::step() {
  execute_instruction(nullptr);
}

void Hcs08::execute_instruction(const Hcs08InstructionReservation *reservation) {
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
    if (impl_->irq_bus) {
      throw Fault("HCS08 IRQ bus sequence/prefetch still has an owner; queue consumption is "
                  "unqualified");
    }
    if (reservation != nullptr) {
      if (!matches_reservation(*reservation)) {
        throw Fault("HCS08 reserved instruction owner/state/code changed address=" +
                    hex(before.pc));
      }
    } else if (impl_->instruction) {
      throw Fault("HCS08 accepted instruction reservation still has an owner address=" +
                  hex(before.pc));
    }
    impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
    if (reservation == nullptr && native::pending_interrupt(impl_->bus, before)) {
      throw Fault(
          "HCS08 interrupt entry is not qualified by instruction stepping; service required");
    }
    impl_->bus.check_execute({.address = before.pc, .length = 1});
    constexpr unsigned rti = 0x80;
    const bool returning = impl_->bus.peek(before.pc) == rti;
    auto registers = returning
                         ? native::RegisterKnowledge{.a = before.a_known, .hx = before.hx_known}
                         : native::registers_after(impl_->bus, before);
    auto known = returning ? before.ccr_known : native::ccr_after(impl_->bus, before);
    impl_->executor->prepare_return_frame(returning ? std::optional(before.sp) : std::nullopt);
    impl_->refresh();
    impl_->executor->execute();
    if (returning) {
      const auto masks = impl_->executor->return_masks();
      known = static_cast<std::uint8_t>(masks[0] | fixed_ccr);
      registers.a = masks[1];
      constexpr unsigned high_mask = 0xff00;
      registers.hx = static_cast<std::uint16_t>((before.hx_known & high_mask) | masks[2]);
    }
    impl_->ccr_known = known;
    impl_->a_known = registers.a;
    impl_->hx_known = registers.hx;
    impl_->normalize_ccr();
    // Native effects are complete even if the subsequent observer refuses retirement.
    if (reservation != nullptr) { impl_->instruction.reset(); }
    impl_->bus.retire_instruction();
    impl_->healthy = true;
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HCS08 execution pc=" + hex(before.pc) + ": " + error.explain);
  } catch (const Fault &error) {
    throw Fault("HCS08 execution pc=" + hex(before.pc) + ": " + error.what());
  }
}
} // namespace blofeld
