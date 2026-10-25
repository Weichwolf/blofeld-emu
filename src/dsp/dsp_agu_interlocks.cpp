// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_agu_interlocks.hpp"

#include "common/bus.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"
#include "dsp56kEmu/registers.h"

#include <algorithm>
#include <array>
#include <cstdint>

namespace blofeld {
namespace {
using dsp56k::getFieldValue;
using dsp56k::getRegister;
using dsp56k::hasField;
using dsp56k::Instruction;
using dsp56k::Register;
constexpr unsigned register_count = 8;
constexpr unsigned settling_clocks = DspAguInterlocks::maximum_stall_units;
constexpr unsigned maximum_native_step_cycles = 1048576;

unsigned pair(const Register reg) {
  if (reg < Register::R0 || reg > Register::M7) { return 0; }
  return 1U << ((static_cast<unsigned>(reg) - static_cast<unsigned>(Register::R0)) %
                register_count);
}

bool move(const Instruction inst) {
  return inst >= dsp56k::Move_Nop && inst <= dsp56k::Movep_SYqq;
}

unsigned sources(const Instruction inst, const unsigned word) {
  unsigned result = 0;
  if (hasField(inst, dsp56k::Field_MMM) || hasField(inst, dsp56k::Field_MM)) {
    const auto mode = getFieldValue(
        inst, hasField(inst, dsp56k::Field_MMM) ? dsp56k::Field_MMM : dsp56k::Field_MM, word);
    if (mode != dsp56k::MMM_AbsAddr) {
      result |= 1U << getFieldValue(inst, dsp56k::Field_RRR, word);
    }
  }
  if (hasField(inst, dsp56k::Field_mm)) {
    const auto x = getFieldValue(inst, dsp56k::Field_RRR, word);
    const auto y = getFieldValue(inst, dsp56k::Field_rr, word) + (x < 4 ? 4U : 0U);
    result |= 1U << y;
  }
  constexpr std::array displacement_forms{dsp56k::Movex_Rnxxx,
                                          dsp56k::Movey_Rnxxx,
                                          dsp56k::Movex_Rnxxxx,
                                          dsp56k::Movey_Rnxxxx,
                                          dsp56k::Lua_Rn};
  if (std::ranges::find(displacement_forms, inst) != displacement_forms.end()) {
    result |= 1U << getFieldValue(inst, dsp56k::Field_RRR, word);
  }
  return result;
}

enum class Direction : std::uint8_t { always, to_register, from_peripheral };

struct Destination {
  Instruction inst;
  dsp56k::Field field;
  Direction direction = Direction::to_register;
};

unsigned destinations(const Instruction inst, const unsigned word) {
  const auto direction =
      hasField(inst, dsp56k::Field_W) ? getFieldValue(inst, dsp56k::Field_W, word) : 0;
  if (inst == dsp56k::Movec_S1D2) {
    return pair(
        getRegister(inst, direction != 0 ? dsp56k::Field_DDDDD : dsp56k::Field_eeeeee, word));
  }
  constexpr std::array roles{
      Destination{
          .inst = dsp56k::Move_xx, .field = dsp56k::Field_ddddd, .direction = Direction::always},
      Destination{
          .inst = dsp56k::Mover, .field = dsp56k::Field_ddddd, .direction = Direction::always},
      Destination{.inst = dsp56k::Movex_ea, .field = dsp56k::Field_ddd},
      Destination{.inst = dsp56k::Movex_aa, .field = dsp56k::Field_ddd},
      Destination{.inst = dsp56k::Movey_ea, .field = dsp56k::Field_ddd},
      Destination{.inst = dsp56k::Movey_aa, .field = dsp56k::Field_ddd},
      Destination{.inst = dsp56k::Movex_Rnxxx, .field = dsp56k::Field_DDDD},
      Destination{.inst = dsp56k::Movey_Rnxxx, .field = dsp56k::Field_DDDD},
      Destination{.inst = dsp56k::Movex_Rnxxxx, .field = dsp56k::Field_DDDDDD},
      Destination{.inst = dsp56k::Movey_Rnxxxx, .field = dsp56k::Field_DDDDDD},
      Destination{.inst = dsp56k::Movec_ea, .field = dsp56k::Field_DDDDD},
      Destination{.inst = dsp56k::Movec_aa, .field = dsp56k::Field_DDDDD},
      Destination{
          .inst = dsp56k::Movec_xx, .field = dsp56k::Field_DDDDD, .direction = Direction::always},
      Destination{.inst = dsp56k::Movem_ea, .field = dsp56k::Field_dddddd},
      Destination{.inst = dsp56k::Movem_aa, .field = dsp56k::Field_dddddd},
      Destination{.inst = dsp56k::Movep_Spp,
                  .field = dsp56k::Field_dddddd,
                  .direction = Direction::from_peripheral},
      Destination{.inst = dsp56k::Movep_SXqq,
                  .field = dsp56k::Field_dddddd,
                  .direction = Direction::from_peripheral},
      Destination{.inst = dsp56k::Movep_SYqq,
                  .field = dsp56k::Field_dddddd,
                  .direction = Direction::from_peripheral}};
  for (const auto role : roles) {
    if (role.inst != inst) { continue; }
    if ((role.direction == Direction::to_register && direction == 0) ||
        (role.direction == Direction::from_peripheral && direction != 0)) {
      return 0;
    }
    return pair(getRegister(inst, role.field, word));
  }
  return 0;
}

void check_nonmove(const Instruction inst, const unsigned word) {
  if (inst == dsp56k::Tcc_S1D1 || inst == dsp56k::Tcc_S1D1S2D2 || inst == dsp56k::Tcc_S2D2 ||
      inst == dsp56k::Lua_ea || inst == dsp56k::Lua_Rn || inst == dsp56k::Lra_Rn ||
      inst == dsp56k::Lra_xxxx) {
    throw Fault("Unqualified Tcc/LUA/LRA sequence in nominal DSP AGU timing");
  }
  dsp56k::RegisterMask written{};
  dsp56k::RegisterMask read{};
  dsp56k::getRegisters(written, read, inst, word);
  // Address-mode pointer updates are not explicit MOVE destinations. Other
  // register writers need their own forwarding rule before entering this mode.
  if (dsp56k::any(written, dsp56k::RegisterMask::N) ||
      dsp56k::any(written, dsp56k::RegisterMask::M) ||
      (sources(inst, word) == 0 && dsp56k::any(written, dsp56k::RegisterMask::R))) {
    throw Fault("Unqualified non-MOVE AGU register destination in nominal DSP timing");
  }
}
}

DspAguEffect DspAguInterlocks::preview(const std::uint32_t opcode) const {
  static const dsp56k::Opcodes opcodes;
  Instruction first = dsp56k::Invalid;
  Instruction second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) {
    throw Fault("Undefined opcode in nominal DSP AGU timing");
  }
  DspAguEffect effect;
  for (const auto inst : std::array{first, second}) {
    if (inst == dsp56k::Invalid) { continue; }
    const auto used = sources(inst, opcode);
    if (move(inst)) {
      effect.writes |= destinations(inst, opcode);
    } else {
      check_nonmove(inst, opcode);
    }
    for (unsigned index = 0; index < register_count; ++index) {
      if ((used & (1U << index)) == 0 || remaining_.at(index) == 0) { continue; }
      if (!move(inst)) { throw Fault("Unqualified non-MOVE AGU consumer in nominal DSP timing"); }
      effect.stalls = std::max(effect.stalls, remaining_.at(index));
    }
  }
  return effect;
}

void DspAguInterlocks::complete(const DspAguEffect effect, const unsigned native_units) {
  if (native_units == 0 || native_units > maximum_native_step_cycles ||
      effect.stalls > settling_clocks) {
    throw Fault("Invalid nominal DSP AGU history advancement");
  }
  for (unsigned index = 0; index < register_count; ++index) {
    auto &remaining = remaining_.at(index);
    remaining -= std::min(remaining, native_units + effect.stalls);
    if ((effect.writes & (1U << index)) != 0) { remaining = settling_clocks; }
  }
}
}
