// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace blofeld::test::irq {
// FM instruction fields, authored independently of the removed SDK assembler.
inline constexpr unsigned initial_sr = 0xc00000, shi_receive_enabled = 0x1009;
inline constexpr unsigned move_sr = 0x05f439, move_vba = 0x05f430;
inline constexpr unsigned move_x0 = 0x44f400, move_x1 = 0x45f400;
inline constexpr unsigned move_r0 = 0x60f400, move_r7 = 0x67f400, initial_x1 = 0x654321;
inline constexpr unsigned short_r1_five = 0x310500, short_r2_seven = 0x320700;
inline constexpr unsigned short_r7_zero = 0x370000;
inline constexpr unsigned short_r2_nine = 0x320900, movem_x0_r0 = 0x076084;
inline constexpr unsigned receive_x0 = 0x044494, clear_a = 0x200013, reset = 0x000084;
inline constexpr unsigned release_mask = 0x00fcb8, rep_two = 0x0602a0, rep_three = 0x0603a0;
inline constexpr unsigned x0_x1 = 0x208500, y0_y1 = 0x20c700, do_two = 0x060280;

void load_program(DspBus &bus, unsigned start, std::span<const std::uint32_t> words);

struct TimerValues {
  unsigned load = 5;
  unsigned compare = 5;
};

struct Setup {
  unsigned priorities = 0;
  unsigned status = initial_sr;
};

class Program {
public:
  void emit(std::uint32_t opcode, std::optional<std::uint32_t> extension = std::nullopt);
  void write(DspStore store);
  void timer(unsigned csr, TimerValues values = {});
  void setup(Setup inputs);
  void install(DspBus &bus, unsigned start = dma_test::boot) const;

  [[nodiscard]] unsigned instructions() const { return instructions_; }

  [[nodiscard]] unsigned pc() const;

  [[nodiscard]] std::span<const std::uint32_t> words() const { return words_; }

private:
  std::vector<std::uint32_t> words_;
  unsigned instructions_ = 0;
};

// Untimed component driver: genuine native instructions and admitted vectors.
// It supplies no clocks, register effects, vector words or success responses.
class Driver {
public:
  explicit Driver(DspMachine &machine);

  [[nodiscard]] dsp56300::Registers state() const { return machine_.cpu().registers(); }

  void step();
  void hardware_reset();

private:
  DspMachine &machine_;
};

[[nodiscard]] unsigned stack_pc(const dsp56300::Registers &state);
[[nodiscard]] unsigned stack_sr(const dsp56300::Registers &state);
void checkpoint_checks();
}
