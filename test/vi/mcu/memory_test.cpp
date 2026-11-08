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
#include <stdexcept>

namespace {
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
}

int main() try {
  mapping();
  native_memory();
  unknown_memory();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
