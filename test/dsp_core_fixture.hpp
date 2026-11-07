// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_rom_program.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace blofeld::test::core {
enum class BitAction : std::uint8_t { clear, set, change, test };

struct Bit {
  DspAddress operand;
  unsigned index = 0;
  BitAction action = BitAction::test;
};
enum class BranchKind : std::uint8_t { relative_clear, jump_clear, jump_set };

struct BitBranch {
  DspAddress operand;
  unsigned index = 0, target = 0;
  BranchKind kind = BranchKind::relative_clear;
};

// Literal instruction fields from DSP56300FM Rev.5, not an assembler/backend.
inline constexpr unsigned clear_a = 0x200013, add_x0_a = 0x200040, add_x1_a = 0x200060;
inline constexpr unsigned negate_a = 0x200036, store_a_absolute = 0x567000;
inline constexpr unsigned store_x0_program_r0 = 0x076084, store_x0_post = 0x445800;
inline constexpr unsigned store_a0_post = 0x505800, change_r0_post = 0x0b5800;
inline constexpr unsigned set_r0_post = 0x0a5820, reset_opcode = 0x000084;
inline constexpr unsigned short_x0_five = 0x240500, short_r0_seven = 0x300700;
inline constexpr unsigned short_a1_nine = 0x2c0900, repeat_x0 = 0x06c420;
inline constexpr unsigned repeat_three = 0x0603a0, do_two = 0x060280;
inline constexpr unsigned short_x0_store = 0x441000, short_x1_store = 0x451100;

void pinit_checks();

class Program {
public:
  void emit(std::uint32_t opcode, std::optional<std::uint32_t> extension = std::nullopt);
  void immediate(DspRomRegister reg, std::uint32_t value);
  void jump(std::uint32_t target);
  void write(DspStore store);
  void receive(DspAddress source, DspRomRegister destination);
  void bit(Bit input);
  void branch(BitBranch input);
  void append(const Program &program);
  void install(DspBus &bus) const;

  [[nodiscard]] unsigned instructions() const { return instructions_; }

  [[nodiscard]] unsigned pc() const { return program_.pc(); }

  [[nodiscard]] std::span<const std::uint32_t> words() const { return program_.words(); }

private:
  DspRomProgram program_;
  unsigned instructions_ = 0;
};

// Actual reset and one untimed native instruction per step; no guest state edits.
class Driver {
public:
  explicit Driver(DspMachine &machine);
  void assert_reset(bool pinit);
  void hardware_reset();
  void step();

  [[nodiscard]] dsp56300::Registers state() const { return machine_.cpu().registers(); }

private:
  DspMachine &machine_;
};
}
