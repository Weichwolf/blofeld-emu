// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_core_fixture.hpp"

#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace blofeld::test::core {
namespace {
constexpr unsigned low = 0xffff80, high = 0xffffc0, word_mask = 0xffffff;
constexpr unsigned field_shift = 8, y_bit = 0x40, bit_count = 24;

void require(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}
}

void Program::emit(const std::uint32_t opcode, const std::optional<std::uint32_t> extension) {
  program_.word(opcode);
  if (extension) { program_.word(*extension); }
  ++instructions_;
}

void Program::immediate(const DspRomRegister reg, const std::uint32_t value) {
  program_.immediate(reg, value);
  ++instructions_;
}

void Program::jump(const std::uint32_t target) {
  constexpr unsigned absolute_jump = 0x0af080;
  emit(absolute_jump, target);
}

void Program::write(const DspStore store) {
  // FM13-135:qq low X/Y and pp high X/Y immediate MOVEP fields.
  constexpr unsigned low_x = 0x07f400;
  constexpr unsigned low_y = 0x07b480;
  constexpr unsigned high_x = 0x08f480;
  constexpr unsigned high_y = 0x09f480;
  require(store.address >= low && store.address <= word_mask &&
              (store.space == DspSpace::x || store.space == DspSpace::y),
          "Literal core diagnostic requires a documented MOVEP peripheral group");
  const bool y = store.space == DspSpace::y;
  const auto low_prefix = y ? low_y : low_x;
  const auto high_prefix = y ? high_y : high_x;
  const auto prefix = store.address < high ? low_prefix : high_prefix;
  emit(prefix | (store.address - (store.address < high ? low : high)), store.value);
}

void Program::receive(const DspAddress source, const DspRomRegister destination) {
  // FM13-136 low-I/O register MOVEP: q5 occupies opcode bit6.
  constexpr unsigned receive_base = 0x044000;
  constexpr unsigned low_mask = 31;
  constexpr unsigned upper_bit = 32;
  constexpr unsigned x_group = 0x80;
  constexpr unsigned y_group = 0x20;
  const auto reg = static_cast<unsigned>(destination);
  require(source.address >= low && source.address < high &&
              (source.space == DspSpace::x || source.space == DspSpace::y) &&
              (destination == DspRomRegister::x0 || destination == DspRomRegister::a0),
          "Literal core receive supports only its documented X0/A0 low-I/O forms");
  const auto offset = source.address - low;
  emit(receive_base | (reg << field_shift) | (source.space == DspSpace::x ? x_group : y_group) |
       ((offset & upper_bit) << 1) | (offset & low_mask));
}

void Program::bit(const Bit input) {
  // FM BCLR/BSET/BCHG/BTST aa6,pp,qq instruction format tables.
  constexpr std::array short_forms{0x0a0000U, 0x0a0020U, 0x0b0000U, 0x0b0020U};
  constexpr std::array low_forms{0x010000U, 0x010020U, 0x014000U, 0x014020U};
  constexpr std::array high_forms{0x0a8000U, 0x0a8020U, 0x0b8000U, 0x0b8020U};
  constexpr unsigned short_words = 64;
  const auto action = static_cast<unsigned>(input.action);
  const auto address = input.operand.address;
  require(action < short_forms.size() && input.index < bit_count &&
              (input.operand.space == DspSpace::x || input.operand.space == DspSpace::y) &&
              (address < short_words || (address >= low && address <= word_mask)),
          "Literal bit diagnostic refuses unsupported address or bit fields");
  auto prefix = short_forms.at(action);
  if (address >= low) { prefix = address < high ? low_forms.at(action) : high_forms.at(action); }
  const auto peripheral_base = address < high ? low : high;
  const auto offset = address < short_words ? address : address - peripheral_base;
  emit(prefix | (offset << field_shift) | (input.operand.space == DspSpace::y ? y_bit : 0U) |
       input.index);
}

void Program::branch(const BitBranch input) {
  // FM BRCLR qq uses a signed displacement; JCLR/JSET qq use an absolute word.
  constexpr std::array prefixes{0x048000U, 0x018080U, 0x0180a0U};
  const auto kind = static_cast<unsigned>(input.kind);
  require(input.operand.address >= low && input.operand.address < high &&
              (input.operand.space == DspSpace::x || input.operand.space == DspSpace::y) &&
              input.index < bit_count && kind < prefixes.size(),
          "Literal core branch supports only its documented low-peripheral forms");
  emit(prefixes.at(kind) | ((input.operand.address - low) << field_shift) |
           (input.operand.space == DspSpace::y ? y_bit : 0U) | input.index,
       input.target);
}

void Program::append(const Program &program) {
  program_.append(program.words());
  instructions_ += program.instructions();
}

void Program::install(DspBus &bus) const {
  bus.install_rom({.start = DspRomProgram::boot, .words = words()});
}

Driver::Driver(DspMachine &machine) : machine_(machine) {
  hardware_reset();
}

void Driver::assert_reset(const bool pinit) {
  constexpr std::uint64_t hold = 90;
  machine_.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = pinit, .recovery_cycles = 2});
  require(machine_.advance_hardware_reset_ticks(hold) == hold, "Core diagnostic reset hold");
}

void Driver::hardware_reset() {
  constexpr std::uint64_t recovery = 90;
  assert_reset(false);
  machine_.release_hardware_reset();
  require(machine_.advance_hardware_reset_ticks(recovery) == recovery,
          "Core diagnostic reset recovery");
}

void Driver::step() {
  require(machine_.cpu().fault().kind == dsp56300::FaultKind::none,
          "Faulted native core diagnostic requires reset");
  try {
    dma_test::step_native(machine_);
  } catch (const Fault &error) {
    const auto fault = machine_.cpu().fault();
    throw Fault("Native core kind=" + std::to_string(static_cast<unsigned>(fault.kind)) +
                " space=" + std::to_string(static_cast<unsigned>(fault.space)) + " address=" +
                hex(fault.address, 6) + " value=" + hex(fault.value, 6) + ": " + error.what());
  }
}
}
