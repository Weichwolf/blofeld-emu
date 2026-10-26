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
#include "keyboard/hcs08_prefetch.hpp"
#include "keyboard/hcs08_registers.hpp"
#include "keyboard/hcs08_vector_memory.hpp"
#include "keyboard/hcs08_write_state.hpp"
#include "memstate.hh"
#include "pcoderaw.hh"
#include "sleigh.hh"
#include "types.h"
#include "xml.hh"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
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
  Impl(Bus &input, const std::filesystem::path &specification, const Hcs08RtiIrqContract contract)
      : bus(input),
        image(input),
        translator(&image, &context),
        memory(&translator),
        rti_contract(contract) {
    if (contract != Hcs08RtiIrqContract::unqualified &&
        contract != Hcs08RtiIrqContract::reference_immediate &&
        contract != Hcs08RtiIrqContract::reference_next_instruction) {
      throw Fault("HCS08 invalid RTI IRQ reference contract");
    }
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
    if (executor->suspended()) { return; }
    const auto revision = bus.code_revision();
    if (!force && decoded_valid && decoded_revision == revision) { return; }
    executor->clear_translations();
    translator.reset(&image, &context);
    ghidra::DocumentStorage documents;
    translator.initialize(documents);
    layout = native::register_layout(translator);
    decoded_revision = revision;
    decoded_valid = true;
  }

  bool run_native(const Bus *program,
                  Hcs08DataRead *operand,
                  const bool word_write,
                  const bool byte_move) {
    image.program(program);
    executor->captured_read(operand);
    constexpr unsigned rti = 0x80;
    constexpr unsigned rts = 0x81;
    const unsigned opcode = prefetch ? prefetch->instruction[0] : 0U;
    auto *returns = program != nullptr && prefetch && (opcode == rti || opcode == rts)
                        ? &prefetch->return_reads
                        : nullptr;
    executor->captured_return(returns);
    bool complete = false;
    try {
      refresh(program != nullptr);
      complete = executor->execute(program, word_write, byte_move);
      if (operand != nullptr && !operand->consumed) {
        throw Fault("HCS08 native instruction did not consume its actual captured data read");
      }
      if (returns != nullptr &&
          !std::ranges::all_of(std::span(*returns).first(opcode == rti ? 5U : 2U),
                               [](const auto &read) { return read.consumed; })) {
        throw Fault("HCS08 native return did not consume all actual captured stack reads");
      }
    } catch (...) {
      executor->captured_read(nullptr);
      executor->captured_return(nullptr);
      image.program(nullptr);
      if (program != nullptr) { decoded_valid = false; }
      throw;
    }
    executor->captured_read(nullptr);
    executor->captured_return(nullptr);
    image.program(nullptr);
    if (program != nullptr) { decoded_valid = false; }
    return complete;
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

  std::uint8_t read_interrupt_vector_byte(const std::uint16_t address) {
    if (auto *const owner = dynamic_cast<Hcs08VectorMemory *>(&bus); owner != nullptr) {
      return owner->read_interrupt_vector_byte(address);
    }
    // Flat CPU test buses retain their declared vector cells. Hardware owners
    // implement the capability; ordinary data/program reads never use it.
    return static_cast<std::uint8_t>(bus.read_memory({.address = address}));
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
    const auto high = read_interrupt_vector_byte(vector);
    const auto low = read_interrupt_vector_byte(static_cast<std::uint16_t>(vector + 1U));
    const auto target = static_cast<std::uint16_t>((static_cast<unsigned>(high) << shift) | low);
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
    sequence.vector_bytes.at(index) = read_interrupt_vector_byte(address);
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

  void begin_program_instruction(Hcs08PrefetchState &program, const Hcs08State &before) const {
    program.before = before;
    if (const auto vector = native::pending_interrupt(bus, before, irq_delay)) {
      throw Fault("HCS08 prefetch IRQ vector=" + hex(*vector) +
                  ": eligible source requires a qualified queue handoff");
    }
    program.native_executed = false;
    if (!before.pc_known || before.pc != program.head || program.ready_bytes == 0) {
      throw Fault("HCS08 prefetch admission address=" + hex(program.head) +
                  " pc=" + hex(before.pc) + ": known matching PC and a ready opcode are required");
    }
    const auto shape = native::prefetch_shape(program.queued[0],
                                              {.pc = before.pc, .ccr = before.ccr},
                                              program.contract,
                                              program.ready_bytes > 1 ? program.queued[1] : 0U);
    if (shape.bytes > program.ready_bytes || shape.bytes > address_space - before.pc) {
      throw Fault("HCS08 prefetch operands address=" + hex(before.pc) + " pc=" + hex(before.pc) +
                  ": actual ready instruction bytes are insufficient or wrap");
    }
    program.operand = {};
    program.clear_read = {};
    program.return_reads = {};
    program.return_discarded.reset();
    program.jump_discarded.reset();
    program.instruction = {};
    std::copy_n(program.queued.begin(), shape.bytes, program.instruction.begin());
    program.instruction_bytes = shape.bytes;
    program.instruction_periods = shape.periods;
    program.instruction_cycle = 0;
    program.native_executed = false;
    for (unsigned n = shape.bytes; n < program.queued_bytes; ++n) {
      program.queued[n - shape.bytes] = program.queued[n];
    }
    program.queued_bytes -= shape.bytes;
    program.ready_bytes -= shape.bytes;
    for (auto n = program.queued_bytes; n < program.queued.size(); ++n) { program.queued[n] = 0; }
    program.head = static_cast<std::uint16_t>(program.head + shape.bytes);
  }

  void program_read(Hcs08PrefetchState &program) {
    if (program.program_byte_in_flight || program.queued_bytes >= program.queued.size()) {
      throw Fault("HCS08 prefetch read address=" + hex(program.next_read) +
                  " pc=" + hex(program.before.pc) + ": program queue has no qualified read slot");
    }
    bus.check_execute({.address = program.next_read, .length = 1});
    const auto value = static_cast<std::uint8_t>(bus.read_memory({.address = program.next_read}));
    program.queued[program.queued_bytes++] = value;
    program.next_read = static_cast<std::uint16_t>(program.next_read + 1U);
    program.program_byte_in_flight = true;
  }

  static void arrive_program_reads(Hcs08PrefetchState &program) {
    program.operand.in_flight = false;
    program.clear_read.in_flight = false;
    for (auto &read : program.return_reads) { read.in_flight = false; }
    if (program.discarded) { program.discarded->in_flight = false; }
    if (program.return_discarded) { program.return_discarded->in_flight = false; }
    if (program.jump_discarded) { program.jump_discarded->in_flight = false; }
    if (program.program_byte_in_flight) {
      program.ready_bytes = program.queued_bytes;
      program.program_byte_in_flight = false;
    }
  }

  void require_program_handoff(const Hcs08State &before) const {
    if (!prefetch || !before.pc_known || before.pc != prefetch->head ||
        prefetch->instruction_bytes != 0 || prefetch->instruction_periods != 0 ||
        prefetch->instruction_cycle != 0 || prefetch->queued_bytes != 3 ||
        prefetch->ready_bytes != 2 || !prefetch->program_byte_in_flight ||
        prefetch->next_read != static_cast<std::uint16_t>(prefetch->head + 3U)) {
      throw Fault("HCS08 IRQ queue handoff pc=" + hex(before.pc) +
                  ": completed instruction and actual matching three-byte queue are required");
    }
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
  bool decoded_valid = true;
  std::uint8_t ccr_known = Hcs08State::all_ccr;
  std::uint8_t a_known = Hcs08State::all_a;
  std::uint16_t hx_known = Hcs08State::all_hx;
  bool pc_known = true;
  bool active = false;
  bool healthy = false;
  bool initialized = false;
  std::optional<Hcs08IrqBusState> irq_bus;
  std::optional<Hcs08InstructionReservation> instruction;
  std::optional<Hcs08PrefetchState> prefetch;
  Hcs08IrqDelay irq_delay = Hcs08IrqDelay::none;
  const Hcs08RtiIrqContract rti_contract;
  const std::shared_ptr<const void> instruction_identity =
      std::make_shared<const InstructionIdentity>();
  // Ownership metadata is not emulated state: never rewind future admissions.
  std::uint64_t instruction_epoch = 0;
};

Hcs08::Hcs08(Bus &bus,
             const std::filesystem::path &specification,
             const Hcs08RtiIrqContract contract) try
    : impl_(std::make_unique<Impl>(bus, specification, contract)) {
} catch (const ghidra::LowlevelError &error) {

  throw Fault("Cannot initialize HCS08 semantics: " + error.explain);
}

Hcs08::~Hcs08() = default;

struct Hcs08Checkpoint::NativeStorage {
  native::NativeInstructionState instruction;
};

void Hcs08::set_state(const Hcs08State &state) {
  const native::ExecutionGuard guard(impl_->active);
  impl_->healthy = false;
  impl_->irq_bus.reset();
  impl_->instruction.reset();
  impl_->prefetch.reset();
  impl_->irq_delay = Hcs08IrqDelay::none;
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
  result.irq_delay_ = impl_->irq_delay;
  result.rti_contract_ = impl_->rti_contract;
  result.prefetch_ = impl_->prefetch;
  if (const auto saved = impl_->executor->checkpoint_write()) {
    result.native_ = std::make_shared<const Hcs08Checkpoint::NativeStorage>(
        Hcs08Checkpoint::NativeStorage{.instruction = *saved});
  }
  return result;
}

void Hcs08::restore_checkpoint(const Hcs08Checkpoint &checkpoint) {
  const native::ExecutionGuard guard(impl_->active);
  if (checkpoint.rti_contract_ != impl_->rti_contract) {
    throw Fault("HCS08 checkpoint RTI IRQ reference contract mismatch");
  }
  impl_->healthy = false;
  impl_->executor->setExecuteAddress(
      ghidra::Address(impl_->translator.getDefaultCodeSpace(), checkpoint.pc_));
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
  impl_->irq_delay = checkpoint.irq_delay_;
  impl_->prefetch = checkpoint.prefetch_;
  if (checkpoint.native_) {
    const auto &owner = impl_->prefetch;
    if (!owner) { throw Fault("HCS08 suspended native write has no captured program"); }
    const auto &program = *owner;
    if ((program.contract != Hcs08PrefetchContract::reference_scanner_word_cycles &&
         program.contract != Hcs08PrefetchContract::reference_scanner_move_cycles &&
         program.contract != Hcs08PrefetchContract::reference_indexed_store_cycles &&
         program.contract != Hcs08PrefetchContract::reference_clear_program_cycles &&
         program.contract != Hcs08PrefetchContract::reference_modify_program_cycles &&
         program.contract != Hcs08PrefetchContract::reference_call_program_cycles &&
         program.contract != Hcs08PrefetchContract::reference_subroutine_program_cycles) ||
        program.native_executed || program.instruction_bytes == 0 ||
        program.before.pc != checkpoint.native_->instruction.address) {
      throw Fault("HCS08 suspended native write lost its captured program owner");
    }
    const native::PrefetchView view(
        impl_->bus,
        program.before.pc,
        std::span(program.instruction).first(program.instruction_bytes));
    impl_->image.program(&view);
    try {
      impl_->executor->restore_write(checkpoint.native_->instruction, program.instruction[0]);
    } catch (...) {
      impl_->image.program(nullptr);
      impl_->decoded_valid = false;
      throw;
    }
    impl_->image.program(nullptr);
    impl_->decoded_valid = false;
  }
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

Hcs08IrqDelay Hcs08::irq_delay() const {
  return impl_->irq_delay;
}

void Hcs08::resume_background() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  const auto before = state();
  if (!before.pc_known || impl_->instruction || impl_->irq_bus || impl_->prefetch) {
    throw Fault(
        "HCS08 BDC resume operation address=" + (before.pc_known ? hex(before.pc) : "unknown") +
        " pc=" + (before.pc_known ? hex(before.pc) : "unknown pc-known=0") + " ccr=" +
        hex(before.ccr) + ": known PC and an unowned instruction/IRQ queue boundary are required");
  }
  impl_->executor->clear_background_request();
}

std::optional<std::uint16_t> Hcs08::pending_interrupt() const {
  if (impl_->active || !impl_->healthy || !impl_->initialized || impl_->irq_bus ||
      impl_->prefetch) {
    throw Fault("HCS08 IRQ query requires a healthy completed native boundary");
  }
  if (impl_->instruction) { return {}; } // Finish the accepted instruction before IRQ admission.
  return native::pending_interrupt(impl_->bus, state(), impl_->irq_delay);
}

bool Hcs08::service_interrupt() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (background_requested()) { throw Fault("HCS08 BGND request requires an owning BDC decision"); }
  if (impl_->instruction) { return false; }
  impl_->healthy = false;
  const auto before = state();
  try {
    if (impl_->irq_bus || impl_->prefetch) {
      throw Fault("HCS08 IRQ bus sequence/prefetch still has an owner");
    }
    const auto vector = native::pending_interrupt(impl_->bus, before, impl_->irq_delay);
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
  if (contract != Hcs08IrqBusContract::reference_unstretched_sequence &&
      contract != Hcs08IrqBusContract::reference_owned_program_handoff) {
    throw Fault("HCS08 IRQ bus requires an explicit unstretched sequence or owned-program handoff "
                "contract");
  }
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  if (impl_->instruction) { return false; }
  const bool handoff = contract == Hcs08IrqBusContract::reference_owned_program_handoff;
  const auto &queue = impl_->prefetch;
  if (handoff && queue && queue->instruction_bytes != 0) { return false; }
  impl_->healthy = false;
  const auto before = state();
  if (background_requested() || impl_->irq_bus || (!handoff && impl_->prefetch)) {
    throw Fault("HCS08 IRQ bus admission requires an unowned normal-running CPU boundary");
  }
  if (handoff) { impl_->require_program_handoff(before); }
  const auto vector = native::pending_interrupt(impl_->bus, before, impl_->irq_delay);
  if (!vector) {
    impl_->healthy = true;
    return false;
  }
  impl_->require_irq_storage(before);
  impl_->irq_bus = Hcs08IrqBusState{.entry = before,
                                    .admitted_vector = *vector,
                                    .discarded_program = handoff ? impl_->prefetch : std::nullopt};
  if (handoff) { impl_->prefetch.reset(); }
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
  if (sequence.discarded_program) { Impl::arrive_program_reads(*sequence.discarded_program); }
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

std::optional<Hcs08PrefetchState> Hcs08::prefetch_state() const {
  return impl_->prefetch;
}

void Hcs08::start_irq_program(const Hcs08PrefetchContract contract) {
  if (contract != Hcs08PrefetchContract::reference_linear_program_cycles &&
      contract != Hcs08PrefetchContract::reference_data_branch_cycles &&
      contract != Hcs08PrefetchContract::reference_short_branch_refill &&
      contract != Hcs08PrefetchContract::reference_peripheral_program_cycles &&
      contract != Hcs08PrefetchContract::reference_jump_program_cycles &&
      contract != Hcs08PrefetchContract::reference_scanner_byte_cycles &&
      contract != Hcs08PrefetchContract::reference_scanner_word_cycles &&
      contract != Hcs08PrefetchContract::reference_scanner_move_cycles &&
      contract != Hcs08PrefetchContract::reference_indexed_store_cycles &&
      contract != Hcs08PrefetchContract::reference_clear_program_cycles &&
      contract != Hcs08PrefetchContract::reference_modify_program_cycles &&
      contract != Hcs08PrefetchContract::reference_call_program_cycles &&
      contract != Hcs08PrefetchContract::reference_subroutine_program_cycles) {
    throw Fault("HCS08 prefetch requires an explicit supported program-cycle reference contract");
  }
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  const auto &irq = impl_->irq_bus;
  constexpr unsigned finished = 11;
  if (!irq || irq->completed_cycles != finished || irq->prefetched_bytes != 3 ||
      irq->ready_program_bytes != 2 || !irq->program_byte_in_flight || impl_->prefetch ||
      impl_->instruction || background_requested()) {
    throw Fault("HCS08 prefetch handoff requires the complete owned IRQ queue");
  }
  const auto before = state();
  if (!before.pc_known || before.pc != irq->target) {
    throw Fault("HCS08 prefetch handoff PC does not match the actual IRQ target");
  }
  impl_->prefetch = Hcs08PrefetchState{.contract = contract,
                                       .head = irq->target,
                                       .next_read = static_cast<std::uint16_t>(irq->target + 3U),
                                       .queued = irq->prefetched,
                                       .queued_bytes = irq->prefetched_bytes,
                                       .ready_bytes = irq->ready_program_bytes,
                                       .program_byte_in_flight = true,
                                       .discarded = std::nullopt,
                                       .return_discarded = std::nullopt,
                                       .jump_discarded = std::nullopt,
                                       .before = before};
  impl_->irq_bus.reset();
}

void Hcs08::execute_program(Hcs08PrefetchState &program, Hcs08DataRead *operand) {
  const native::PrefetchView view(impl_->bus,
                                  program.before.pc,
                                  std::span(program.instruction).first(program.instruction_bytes));
  const auto shape =
      native::prefetch_shape(program.instruction[0],
                             {.pc = program.before.pc, .ccr = program.before.ccr},
                             program.contract,
                             program.instruction_bytes > 1 ? program.instruction[1] : 0U);
  program.native_executed = execute_native(program.before,
                                           &view,
                                           operand,
                                           shape.word_write || shape.call,
                                           shape.byte_move || shape.modify);
}

void Hcs08::advance_data_program(Hcs08PrefetchState &program,
                                 const unsigned phase,
                                 const bool branch) {
  const bool extended = !branch && program.instruction_bytes == 3;
  constexpr unsigned pull_h = 0x8a;
  constexpr unsigned pull_x = 0x88;
  constexpr unsigned lda_indexed8 = 0xe6;
  constexpr unsigned lda_indexed16 = 0xd6;
  const bool pull = program.instruction[0] == pull_h || program.instruction[0] == pull_x;
  const bool indexed =
      program.instruction[0] == lda_indexed8 || program.instruction[0] == lda_indexed16;
  const unsigned read_phase = extended ? 2 : 1;
  if (phase == read_phase) {
    program.operand.address = program.instruction[1];
    if (extended) {
      program.operand.address = static_cast<std::uint16_t>(
          (static_cast<unsigned>(program.instruction[1]) << 8U) | program.instruction[2]);
    }
    if (pull) { program.operand.address = static_cast<std::uint16_t>(program.before.sp + 1U); }
    if (indexed) {
      constexpr unsigned all_index_bits = 0xffff;
      if (program.before.hx_known != all_index_bits) {
        throw Fault("HCS08 indexed read address=unknown: incomplete native H:X knowledge");
      }
      program.operand.address = static_cast<std::uint16_t>(
          static_cast<unsigned>(program.operand.address) + program.before.hx);
    }
    constexpr unsigned lda_direct = 0xb6;
    constexpr unsigned lda_extended = 0xc6;
    const auto opcode = program.instruction[0];
    auto *known = dynamic_cast<Hcs08KnownMemory *>(&impl_->bus);
    const bool raw = known != nullptr && (opcode == lda_direct || opcode == lda_extended);
    const auto byte =
        raw ? known->read_known_byte(program.operand.address)
            : Hcs08KnownByte{.value = static_cast<std::uint8_t>(
                                 impl_->bus.read_memory({.address = program.operand.address}))};
    program.operand.value = static_cast<std::uint8_t>(byte.value & byte.known);
    program.operand.known = byte.known;
    program.operand.in_flight = true;
    return;
  }
  const auto program_address = program.next_read;
  if (!pull || phase != read_phase + 1) { impl_->program_read(program); }
  if (phase != read_phase + 1) { return; }
  execute_program(program, &program.operand);
  if (!branch) { return; }
  // Explicit branch reference: p2 is sequential speculation; the selected
  // native PC supplies three refill reads on p3..p5, including not-taken cases.
  if (program.queued_bytes != 1 || program.ready_bytes != 0 || !program.program_byte_in_flight ||
      !state().pc_known) {
    throw Fault(
        "HCS08 bit-branch refill requires its actual speculative program read and native PC");
  }
  program.discarded = Hcs08DiscardedProgramRead{
      .address = program_address, .value = program.queued[0], .in_flight = true};
  program.queued = {};
  program.queued_bytes = 0;
  program.ready_bytes = 0;
  program.program_byte_in_flight = false;
  program.head = state().pc;
  program.next_read = program.head;
}

void Hcs08::advance_short_branch(Hcs08PrefetchState &program, const unsigned phase) {
  if (phase == 1) {
    if (program.queued_bytes != 1 || program.ready_bytes != 1 || program.program_byte_in_flight) {
      throw Fault("HCS08 short-branch refill requires its actual ready sequential successor");
    }
    execute_program(program);
    if (!state().pc_known) { throw Fault("HCS08 short-branch native PC remains unknown"); }
    // Explicit reference: discard the already-fetched successor, then p1..p3
    // refill from the native selected PC for both branch outcomes.
    program.discarded = Hcs08DiscardedProgramRead{
        .address = program.head, .value = program.queued[0], .in_flight = false};
    program.queued = {};
    program.queued_bytes = 0;
    program.ready_bytes = 0;
    program.head = state().pc;
    program.next_read = program.head;
  }
  impl_->program_read(program);
}

void Hcs08::advance_jump_program(Hcs08PrefetchState &program, const unsigned phase) {
  const unsigned execute_phase = program.instruction_bytes == 3 ? 2U : 1U;
  if (phase == execute_phase) {
    const unsigned successors = program.instruction_bytes == 1 ? 2U : 1U;
    if (program.queued_bytes != successors || program.ready_bytes != successors ||
        program.program_byte_in_flight) {
      throw Fault("HCS08 JMP requires its actual ready sequential successor bytes");
    }
    execute_program(program);
    if (!state().pc_known) { throw Fault("HCS08 JMP native target remains unknown"); }
    // Explicit reference: three-byte forms speculate sequentially on p1;
    // native JMP then discards the real successor(s) and refills three p reads.
    program.discarded = Hcs08DiscardedProgramRead{
        .address = program.head, .value = program.queued[0], .in_flight = false};
    if (successors == 2) {
      program.jump_discarded =
          Hcs08DiscardedProgramRead{.address = static_cast<std::uint16_t>(program.head + 1U),
                                    .value = program.queued[1],
                                    .in_flight = false};
    }
    program.queued = {};
    program.queued_bytes = 0;
    program.ready_bytes = 0;
    program.head = state().pc;
    program.next_read = program.head;
  }
  impl_->program_read(program);
}

void Hcs08::advance_call_program(Hcs08PrefetchState &program, const unsigned phase) {
  const unsigned first = program.instruction_bytes == 3 ? 2U : 1U;
  if (phase == first || phase == first + 1) {
    execute_program(program);
    if (phase == first) { return; }
    if (!program.native_executed || !state().pc_known) {
      throw Fault("HCS08 call requires its completed native stack transfer and target");
    }
    const unsigned successors = program.instruction_bytes == 1 ? 2U : 1U;
    if (program.queued_bytes != successors || program.ready_bytes != successors ||
        program.program_byte_in_flight) {
      throw Fault("HCS08 call requires its actual ready sequential successor bytes");
    }
    program.discarded = Hcs08DiscardedProgramRead{
        .address = program.head, .value = program.queued[0], .in_flight = false};
    if (successors == 2) {
      program.jump_discarded =
          Hcs08DiscardedProgramRead{.address = static_cast<std::uint16_t>(program.head + 1U),
                                    .value = program.queued[1],
                                    .in_flight = false};
    }
    program.queued = {};
    program.queued_bytes = 0;
    program.ready_bytes = 0;
    program.head = state().pc;
    program.next_read = program.head;
    return;
  }
  impl_->program_read(program);
}

void Hcs08::advance_modify_program(Hcs08PrefetchState &program, const unsigned phase) {
  const unsigned read_phase = program.instruction_bytes == 3 ? 2U : 1U;
  const unsigned write_phase = read_phase + 2;
  // Native r suspends at its real STORE; f consumes no bus access, then w resumes.
  if (phase == read_phase || phase == write_phase) {
    execute_program(program);
  } else if (phase != read_phase + 1) {
    impl_->program_read(program);
  }
}

void Hcs08::advance_clear_program(Hcs08PrefetchState &program, const unsigned phase) {
  constexpr unsigned stack_prefix = 0x9e;
  constexpr unsigned indexed = 0x7f;
  constexpr unsigned indexed8 = 0x6f;
  const auto opcode = program.instruction[0];
  const bool stack = opcode == stack_prefix;
  const unsigned read_phase = stack ? 2U : 1U;
  const unsigned write_phase = read_phase + 2;
  if (phase == read_phase) {
    unsigned address = stack ? program.instruction[2] : program.instruction[1];
    if (opcode == indexed || opcode == indexed8) {
      if (program.before.hx_known != native::address_mask) {
        throw Fault("HCS08 CLR read address=" + hex(program.before.hx) +
                    " pc=" + hex(program.before.pc) + " hx-known=" + hex(program.before.hx_known) +
                    ": indexed address is unknown");
      }
      address = program.before.hx + (opcode == indexed ? 0U : address);
    } else if (stack) {
      address += program.before.sp;
    }
    program.clear_read.address = static_cast<std::uint16_t>(address);
    // The ISA r period occurs even though CLR does not use the old byte.
    // Native SLEIGH supplies only its semantic zero write and flags at w.
    auto *known = dynamic_cast<Hcs08KnownMemory *>(&impl_->bus);
    const auto byte =
        known != nullptr ? known->read_known_byte(program.clear_read.address)
                         : Hcs08KnownByte{.value = static_cast<std::uint8_t>(impl_->bus.read_memory(
                                              {.address = program.clear_read.address}))};
    program.clear_read.value = byte.value;
    program.clear_read.known = byte.known;
    program.clear_read.in_flight = true;
  } else if (phase == write_phase) {
    if (!program.clear_read.value || program.clear_read.in_flight) {
      throw Fault("HCS08 CLR write pc=" + hex(program.before.pc) +
                  ": actual r period has not completed");
    }
    execute_program(program);
  } else if (phase != read_phase + 1) {
    impl_->program_read(program);
  }
}

void Hcs08::advance_write_program(Hcs08PrefetchState &program, const unsigned phase) {
  const unsigned write_phase = program.instruction_bytes == 3 ? 2 : 1;
  if (phase == write_phase) {
    execute_program(program);
  } else {
    impl_->program_read(program);
  }
}

void Hcs08::advance_return_program(Hcs08PrefetchState &program, const unsigned phase) {
  constexpr unsigned rts = 0x81;
  const bool subroutine = program.instruction[0] == rts;
  const unsigned stack_reads = subroutine ? 2U : 5U;
  const unsigned raw_fields = subroutine ? 0U : 3U;
  const unsigned execute_phase = stack_reads + 1;
  if (phase <= stack_reads) {
    auto &read = program.return_reads.at(phase - 1);
    read.address = static_cast<std::uint16_t>(program.before.sp + phase);
    auto *known = dynamic_cast<Hcs08KnownMemory *>(&impl_->bus);
    const auto byte = phase <= raw_fields && known != nullptr
                          ? known->read_known_byte(read.address)
                          : Hcs08KnownByte{.value = static_cast<std::uint8_t>(
                                               impl_->bus.read_memory({.address = read.address}))};
    read.value = byte.value;
    read.known = byte.known;
    read.in_flight = true;
    return;
  }
  if (phase == execute_phase) {
    if (program.queued_bytes != 2 || program.ready_bytes != 2 || program.program_byte_in_flight) {
      throw Fault("HCS08 return requires both actual ready sequential successor bytes");
    }
    execute_program(program);
    if (!state().pc_known) { throw Fault("HCS08 native return PC remains unknown"); }
    // Explicit references: RTS uufppp, RTI uuuuufppp. Native state completes at f;
    // the last three p periods refill from the genuine returned PC.
    program.discarded = Hcs08DiscardedProgramRead{
        .address = program.head, .value = program.queued[0], .in_flight = false};
    program.return_discarded =
        Hcs08DiscardedProgramRead{.address = static_cast<std::uint16_t>(program.head + 1U),
                                  .value = program.queued[1],
                                  .in_flight = false};
    program.queued = {};
    program.queued_bytes = 0;
    program.ready_bytes = 0;
    program.head = state().pc;
    program.next_read = program.head;
    return;
  }
  impl_->program_read(program);
}

void Hcs08::advance_split_write_program(Hcs08PrefetchState &program,
                                        const unsigned phase,
                                        const bool move) {
  const unsigned first = move || program.instruction_bytes == 2 ? 1U : 2U;
  const unsigned second = move ? 3U : first + 1;
  // MOV: native r1 suspends before w3; STHX: its two native w periods.
  if (phase == first || phase == second) {
    execute_program(program);
  } else {
    impl_->program_read(program);
  }
}

void Hcs08::advance_program_period(Hcs08PrefetchState &program, const unsigned phase) {
  const auto before = program.before;
  impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
  const auto shape =
      native::prefetch_shape(program.instruction[0],
                             {.pc = before.pc, .ccr = before.ccr},
                             program.contract,
                             program.instruction_bytes > 1 ? program.instruction[1] : 0U);
  const bool final = phase == program.instruction_periods;
  if (shape.call) {
    advance_call_program(program, phase);
  } else if (shape.modify) {
    advance_modify_program(program, phase);
  } else if (shape.clear) {
    advance_clear_program(program, phase);
  } else if (shape.word_write || shape.byte_move) {
    advance_split_write_program(program, phase, shape.byte_move);
  } else if (shape.jump) {
    advance_jump_program(program, phase);
  } else if (shape.returning) {
    advance_return_program(program, phase);
  } else if (shape.read) {
    advance_data_program(program, phase, shape.branch);
  } else if (shape.write) {
    advance_write_program(program, phase);
  } else if (shape.branch) {
    advance_short_branch(program, phase);
  } else {
    if (shape.push && phase == 1) { execute_program(program); }
    // pp/ppp immediate, sp pushes, p singles, ffffp MUL and fffffp DIV.
    const bool program_access = final || (!shape.push && shape.bytes > 1);
    if (program_access) { impl_->program_read(program); }
    if (!shape.push && final) { execute_program(program); }
  }
}

void Hcs08::advance_irq_program() {
  const native::ExecutionGuard guard(impl_->active);
  if (!impl_->healthy) { throw Fault("HCS08 incomplete execution requires coordinated recovery"); }
  impl_->healthy = false;
  auto &owner = impl_->prefetch;
  if (!owner) { throw Fault("HCS08 prefetch advance requires its owned program queue"); }
  auto &program = *owner;
  constexpr auto limit = std::numeric_limits<std::uint64_t>::max();
  if (program.attempted_periods == limit || program.completed_periods == limit ||
      program.retired_instructions == limit) {
    throw Fault("HCS08 prefetch period or instruction counter exhausted");
  }
  ++program.attempted_periods;
  Impl::arrive_program_reads(program);
  try {
    if (program.instruction_bytes == 0) { impl_->begin_program_instruction(program, state()); }
    const auto phase = program.instruction_cycle + 1;
    const bool final = phase == program.instruction_periods;
    advance_program_period(program, phase);
    program.instruction_cycle = phase;
    ++program.completed_periods;
    if (final) {
      if (!program.native_executed) {
        throw Fault("HCS08 native instruction remains unfinished at retirement");
      }
      program.instruction_bytes = 0;
      program.instruction_periods = 0;
      program.instruction_cycle = 0;
      impl_->bus.retire_instruction();
      ++program.retired_instructions;
    }
    impl_->healthy = true;
  } catch (const ghidra::LowlevelError &error) {
    throw Fault("HCS08 prefetch cycle pc=" + hex(program.before.pc) + ": " + error.explain);
  } catch (const Fault &error) {
    throw Fault("HCS08 prefetch cycle pc=" + hex(program.before.pc) +
                " ccr=" + hex(program.before.ccr) + ": " + error.what());
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
  if (impl_->instruction || impl_->irq_bus || impl_->prefetch || background_requested()) {
    return {};
  }
  impl_->healthy = false;
  const auto before = state();
  if (!before.pc_known) {
    throw Fault(
        "HCS08 instruction admission address=unknown pc=unknown pc-known=0: unknown CPU PC");
  }
  impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
  if (native::pending_interrupt(impl_->bus, before, impl_->irq_delay)) {
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

bool Hcs08::execute_native(const Hcs08State &before,
                           const Bus *program,
                           Hcs08DataRead *operand,
                           const bool word_write,
                           const bool byte_move) {
  const auto &code = program != nullptr ? *program : impl_->bus;
  constexpr unsigned rti = 0x80;
  const auto opcode = code.peek(before.pc);
  constexpr unsigned swi = 0x83;
  if (opcode == swi) {
    throw Fault("HCS08 SWI vector read address=0xfffc pc=" + hex(before.pc) +
                " ccr=" + hex(before.ccr) + " ccr-known=" + hex(before.ccr_known) +
                " agent=CPU: native vector dereference/owned routing is unqualified");
  }
  const bool returning = opcode == rti;
  auto registers = returning ? native::RegisterKnowledge{.a = before.a_known, .hx = before.hx_known}
                             : native::registers_after(code, before, impl_->bus);
  auto known = returning ? before.ccr_known : native::ccr_after(code, before);
  impl_->executor->prepare_return_frame(returning ? std::optional(before.sp) : std::nullopt);
  if (!impl_->run_native(program, operand, word_write, byte_move)) { return false; }
  if (const auto loaded = impl_->executor->loaded_operand()) {
    registers.a = loaded->known;
    known = native::byte_vnz_after(before, *loaded);
  }
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
  impl_->irq_delay = native::irq_delay_after(before, state(), opcode, impl_->rti_contract);
  return true;
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
    if (impl_->irq_bus || impl_->prefetch) {
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
    if (reservation == nullptr && native::pending_interrupt(impl_->bus, before, impl_->irq_delay)) {
      throw Fault(
          "HCS08 interrupt entry is not qualified by instruction stepping; service required");
    }
    impl_->bus.check_execute({.address = before.pc, .length = 1});
    execute_native(before);
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
