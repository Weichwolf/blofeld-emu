// SPDX-License-Identifier: GPL-3.0-only
#include "isa.hpp"
#include "mcu/memory.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {
constexpr cpu12::Address overlay_first = 0x1600;
constexpr std::size_t overlay_bytes = 0x210;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void mapping() {
  using blofeld::vi::mcu::Space;
  blofeld::vi::FlashImage flash;
  flash.known.fill(1);
  blofeld::vi::mcu::Memory memory(flash);
  constexpr std::uint16_t system = 0x1200;
  constexpr std::uint16_t old_queue = 0x0400;
  constexpr std::uint16_t queue_reserved = 0x0600;
  constexpr std::uint16_t new_queue = 0x2400;
  constexpr std::uint16_t queue_register = 0x12;
  constexpr std::uint8_t queue_at_2000 = 0x21;
  constexpr std::uint8_t data = 0x55;
  std::uint8_t value = 0;
  require(memory.locate(0).space == Space::registers, "Registers override QRAM");
  require(memory.locate(system).space == Space::system_ram &&
              memory.locate(old_queue).space == Space::queue_ram &&
              memory.locate(queue_reserved).space == Space::reserved,
          "Reset map has exact SRAM, QRAM and reserved ranges");
  require(!memory.read(system, value), "Power-on RAM is unknown");
  require(memory.write({.address = system, .value = data}) &&
              memory.write({.address = old_queue, .value = data}),
          "Actual RAM stores");
  require(memory.write_mapping({.address = queue_register, .value = queue_at_2000}),
          "Stage QRAM remap");
  require(memory.read_mapping(queue_register) == 1 && memory.read(old_queue, value),
          "Mapping remains unchanged inside the instruction");
  require(memory.commit_mapping() && memory.read(new_queue, value) && value == data,
          "Remap exposes the same backing bytes");
  require(!memory.read(old_queue, value), "Old QRAM address is no longer granted");
  memory.reset();
  require(memory.read(system, value) && value == data && memory.read(old_queue, value),
          "Warm reset retains RAM while restoring the reset map");
  memory.power_on();
  require(!memory.read(system, value) && !memory.read(old_queue, value),
          "Power cycle retires volatile knowledge");
  constexpr std::uint16_t initrm = 0x10;
  require(memory.write_mapping({.address = initrm, .value = 0}) && memory.commit_mapping(),
          "First INITRM write");
  require(!memory.write_mapping({.address = initrm, .value = 9}) &&
              memory.read_mapping(initrm) == 0,
          "INITRM is write-once in normal mode");
  constexpr std::uint16_t primary = 0x8000;
  constexpr std::uint16_t alias = 0x4000;
  flash.bytes[0] = data;
  require(memory.read(primary, value) && value == data && memory.read(alias, value) &&
              value == data,
          "Primary and lower flash mapping alias physical storage");
  require(!memory.write({.address = primary, .value = 0}),
          "Flash writes require the flash controller");
}

void native_memory() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t program_address = 0x8200;
  constexpr std::size_t program_offset = 0x200;
  constexpr std::uint16_t destination = 0x1200;
  constexpr std::uint16_t stack = 0x1fff;
  constexpr std::uint16_t reset_vector = 0xfffe;
  constexpr std::uint8_t data = 0x55;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                                        std::array{data},
                                        isa::opcode<isa::Opcode::staa_extended>(),
                                        std::array{static_cast<std::uint8_t>(destination >> 8U),
                                                   static_cast<std::uint8_t>(destination)},
                                        isa::stop);
  blofeld::vi::FlashImage image;
  image.known.fill(1);
  std::ranges::copy(program, image.bytes.begin() + program_offset);
  image.bytes[blofeld::vi::FlashImage::byte_count - 2] =
      static_cast<std::uint8_t>(program_address >> 8U);
  image.bytes[blofeld::vi::FlashImage::byte_count - 1] = static_cast<std::uint8_t>(program_address);
  blofeld::vi::mcu::Memory memory(image);
  const auto grants = memory.regions();
  cpu12::Core core(cpu12::Memory::mapped(grants));
  require(core.reset(reset_vector).accepted && core.registers().pc == program_address,
          "Native reset vector comes from the borrowed flash image");
  auto registers = core.registers();
  registers.sp = stack;
  registers.ccr = 0;
  core.set_registers(registers);
  const auto result = core.run();
  require(result.fault == cpu12::Fault::none && result.state == cpu12::ExecutionState::stopped,
          "Authored native program stops without an execution budget");
  std::uint8_t value = 0;
  require(memory.read(destination, value) && value == data,
          "Native RAM writes and knownness reach the consumer directly");
}

void unknown_memory() {
  namespace isa = cpu12::isa;
  constexpr std::uint16_t program_address = 0x8200;
  constexpr std::size_t program_offset = 0x200;
  constexpr std::uint16_t unreadable = 0x1210;
  constexpr std::uint16_t flash_hole = 0x8300;
  constexpr std::size_t physical_hole = 0x300;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::ldaa_extended>(),
                                        std::array{static_cast<std::uint8_t>(unreadable >> 8U),
                                                   static_cast<std::uint8_t>(unreadable)});
  blofeld::vi::FlashImage image;
  image.known.fill(1);
  image.known[physical_hole] = 0;
  std::ranges::copy(program, image.bytes.begin() + program_offset);
  blofeld::vi::mcu::Memory memory(image);
  const auto grants = memory.regions();
  for (const auto grant : grants) {
    require(flash_hole < grant.first || flash_hole - grant.first >= grant.bytes.size(),
            "Unknown flash never becomes an immutable native grant");
  }
  cpu12::Core core(cpu12::Memory::mapped(grants));
  core.set_registers({.pc = program_address});
  const auto result = core.run();
  require(result.fault == cpu12::Fault::memory_read && result.address == unreadable &&
              result.pc == program_address && result.retired == 0,
          "Uninitialized native load fails with its actual PC and address");
}

cpu12::Ram grant(blofeld::vi::mcu::Memory &memory, const cpu12::Address address) {
  for (const auto view : memory.regions()) {
    if (address >= view.first && std::size_t{address} - view.first < view.bytes.size()) {
      return view;
    }
  }
  throw std::runtime_error("Expected an actual borrowed memory grant");
}

void program_bindings() {
  blofeld::vi::FlashImage image;
  blofeld::vi::mcu::Memory memory(image);
  constexpr auto original = cpu12::isa::nop[0];
  constexpr cpu12::Byte replacement = 0x5a;
  require(!memory.bind_program(overlay_first), "Unknown RAM cannot become immutable program");
  for (std::size_t index = 0; index < overlay_bytes - 1; ++index) {
    require(memory.write(
                {.address = static_cast<cpu12::Address>(overlay_first + index), .value = original}),
            "Initialize independently authored program bytes");
  }
  require(!memory.bind_program(overlay_first), "One unknown tail byte prevents executable binding");
  require(memory.write({.address = overlay_first + overlay_bytes - 1, .value = original}) &&
              memory.bind_program(overlay_first),
          "Only a complete actual payload receives an immutable binding");
  const auto first = grant(memory, overlay_first);
  require(!first.writable && first.known.empty() && first.identity != 0 &&
              first.bytes.size() == overlay_bytes && first.first == overlay_first,
          "The exact code extent has initialized read-only native grants");
  require(grant(memory, overlay_first - 1).writable &&
              grant(memory, overlay_first + overlay_bytes).writable &&
              !memory.bind_program(overlay_first),
          "Adjacent data remains writable and the active binding is reused");
  require(memory.write({.address = overlay_first, .value = original}) &&
              !memory.program_changed() && !memory.commit_mapping(),
          "An unchanged real write preserves the same executable identity");
  require(memory.write({.address = overlay_first, .value = replacement}) &&
              memory.program_changed() && !memory.bind_program(overlay_first) &&
              memory.commit_mapping() && !memory.program_changed(),
          "An actual code change releases ownership until its instruction boundary commits");
  require(memory.bind_program(overlay_first) &&
              grant(memory, overlay_first).identity != first.identity,
          "Changed bytes receive a different immutable code identity");
  const auto changed_identity = grant(memory, overlay_first).identity;
  require(memory.write({.address = overlay_first, .value = original}) && memory.commit_mapping() &&
              memory.bind_program(overlay_first) &&
              grant(memory, overlay_first).identity == first.identity,
          "Restoring identical contents reuses the original translation identity");
  memory.power_on();
  require(!memory.bind_program(overlay_first) && grant(memory, overlay_first).writable,
          "Power-on removes executable ownership and does not manufacture initialized RAM");
  require(memory.commit_mapping(), "Power-on retires the preceding executable grant");
  for (std::size_t index = 0; index < overlay_bytes; ++index) {
    require(memory.write({.address = static_cast<cpu12::Address>(overlay_first + index),
                          .value = replacement}),
            "Initialize different code after power-on");
  }
  require(memory.bind_program(overlay_first) &&
              grant(memory, overlay_first).identity != first.identity &&
              grant(memory, overlay_first).identity != changed_identity,
          "Power-on cannot reuse an old identity for different executable bytes");
}

struct ProgramWriter {
  blofeld::vi::mcu::Memory &memory;
  cpu12::Core *core = nullptr;

  static bool write(void *owner, const cpu12::ByteWrite access) noexcept {
    auto &self = *static_cast<ProgramWriter *>(owner);
    const bool accepted = self.memory.write(access);
    if (self.memory.program_changed()) { self.core->request_yield(); }
    return accepted;
  }

  static bool write_word(void *owner, const cpu12::WordWrite access) noexcept {
    constexpr unsigned byte_mask = 0xff;
    return write(owner,
                 {.address = access.address,
                  .value = static_cast<cpu12::Byte>(access.value >> 8U)}) &&
           write(owner,
                 {.address = static_cast<cpu12::Address>(access.address + 1U),
                  .value = static_cast<cpu12::Byte>(access.value & byte_mask)});
  }
};

void native_program_transition(const bool word) {
  namespace isa = cpu12::isa;
  constexpr cpu12::Byte replacement = 0x5a;
  constexpr cpu12::Byte previous = 0x12;
  constexpr cpu12::Address operand = overlay_first + 6;
  constexpr cpu12::Address stack = 0x1fff;
  constexpr auto byte_program =
      isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                   std::array{replacement},
                   isa::opcode<isa::Opcode::staa_extended>(),
                   std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(operand >> 8U),
                                              static_cast<cpu12::Byte>(operand)},
                   isa::opcode<isa::Opcode::ldab_immediate>(),
                   std::array{previous},
                   isa::wai);
  constexpr cpu12::Address word_operand = overlay_first + 7;
  constexpr auto word_program =
      isa::program(isa::opcode<isa::Opcode::ldd_immediate>(),
                   std::array<cpu12::Byte, 2>{0x5a, 0x96},
                   isa::opcode<isa::Opcode::std_extended>(),
                   std::array<cpu12::Byte, 2>{static_cast<cpu12::Byte>(word_operand >> 8U),
                                              static_cast<cpu12::Byte>(word_operand)},
                   isa::opcode<isa::Opcode::ldd_immediate>(),
                   std::array<cpu12::Byte, 2>{0x12, 0x34},
                   isa::wai);
  const auto program = word ? std::span<const cpu12::Byte>(word_program)
                            : std::span<const cpu12::Byte>(byte_program);
  blofeld::vi::FlashImage image;
  blofeld::vi::mcu::Memory memory(image);
  for (std::size_t index = 0; index < overlay_bytes; ++index) {
    const auto byte = index < program.size() ? program[index] : isa::nop[0];
    require(memory.write(
                {.address = static_cast<cpu12::Address>(overlay_first + index), .value = byte}),
            "Load an authored program with a real write to its following immediate operand");
  }
  require(memory.bind_program(overlay_first), "Bind initialized RAM code before native execution");
  auto views = memory.regions();
  const auto identity = grant(memory, overlay_first).identity;
  ProgramWriter writer{.memory = memory};
  cpu12::Io io{
      .owner = &writer, .write = ProgramWriter::write, .write_word = ProgramWriter::write_word};
  cpu12::Core core(cpu12::Memory::mapped(views), &io);
  writer.core = &core;
  core.set_registers({.pc = overlay_first, .sp = stack});
  const auto prefix = core.run();
  require(prefix.fault == cpu12::Fault::none && prefix.retired == 2 &&
              core.registers().pc == overlay_first + (word ? 6 : 5) && memory.program_changed(),
          "A genuine code write completes exactly once before leaving the old native binding");
  require(memory.commit_mapping() && memory.bind_program(core.registers().pc) &&
              grant(memory, overlay_first).identity != identity,
          "The consumer publishes changed code at the actual completed-instruction boundary");
  auto next = memory.regions();
  core.set_memory(cpu12::Memory::mapped(next));
  views.swap(next);
  const auto suffix = core.run();
  constexpr cpu12::Address byte_expected = 0x5a5a;
  constexpr cpu12::Address word_expected = 0x5a96;
  require(suffix.fault == cpu12::Fault::none && suffix.retired == 2 &&
              suffix.state == cpu12::ExecutionState::waiting &&
              core.registers().d == (word ? word_expected : byte_expected),
          "The new binding executes the changed operand without replaying the prior store");
}

void interrupt_program_transition() {
  constexpr cpu12::Address stack = overlay_first + 9;
  constexpr cpu12::Address vector = 0xfff0;
  constexpr std::size_t vector_offset = 0x7ff0;
  blofeld::vi::FlashImage image;
  image.known.fill(1);
  image.bytes[vector_offset] = static_cast<cpu12::Byte>(overlay_first >> 8U);
  image.bytes[vector_offset + 1] = static_cast<cpu12::Byte>(overlay_first);
  blofeld::vi::mcu::Memory memory(image);
  for (std::size_t index = 0; index < overlay_bytes; ++index) {
    require(memory.write({.address = static_cast<cpu12::Address>(overlay_first + index),
                          .value = cpu12::isa::nop[0]}),
            "Initialize an authored overlay before an overlapping interrupt stack write");
  }
  require(memory.bind_program(overlay_first), "Bind the original program before warming it");
  auto views = memory.regions();
  ProgramWriter writer{.memory = memory};
  cpu12::Io io{
      .owner = &writer, .write = ProgramWriter::write, .write_word = ProgramWriter::write_word};
  cpu12::Core core(cpu12::Memory::mapped(views), &io);
  writer.core = &core;
  core.set_registers({.pc = overlay_first, .sp = stack});
  require(core.run(1).retired == 1, "Warm an actual native entry for the original code image");
  const auto entered = core.interrupt({.vector = vector});
  require(entered.accepted && memory.program_changed() && core.registers().pc == overlay_first,
          "A real interrupt frame can overwrite code before its target instruction executes");
  require(memory.commit_mapping() && memory.bind_program(core.registers().pc),
          "Commit interrupt-entry writes before preparing the next executable binding");
  auto next = memory.regions();
  core.set_memory(cpu12::Memory::mapped(next));
  views.swap(next);
  const auto result = core.run();
  require(result.fault == cpu12::Fault::unsupported_instruction && result.retired == 0 &&
              result.pc == overlay_first && result.opcode == 0,
          "The newly written zero CCR byte is decoded instead of executing stale cached NOPs");
}
}

int main() try {
  mapping();
  native_memory();
  unknown_memory();
  program_bindings();
  native_program_transition(false);
  native_program_transition(true);
  interrupt_program_transition();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
