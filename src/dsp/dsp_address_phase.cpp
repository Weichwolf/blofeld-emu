// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_phase.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp56kEmu/opcodeanalysis.h"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <array>
#include <cstdint>
#include <exception>
#include <optional>
#include <string>

namespace blofeld {
namespace {
bool effective(const dsp56k::Instruction type) {
  return type == dsp56k::Movex_ea || type == dsp56k::Movey_ea || type == dsp56k::Movec_ea ||
         type == dsp56k::Movel_ea;
}

bool absolute(const dsp56k::Instruction type) {
  return type == dsp56k::Movex_aa || type == dsp56k::Movey_aa || type == dsp56k::Movec_aa ||
         type == dsp56k::Movel_aa;
}

std::optional<unsigned> address_register(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0) {
    throw Fault("Undefined native address-phase opcode=" + hex(opcode, 6));
  }
  const auto type = effective(first) || absolute(first) ? first : second;
  if (absolute(type)) { return std::nullopt; }
  if (!effective(type)) {
    throw Fault("Unqualified native memory-address phase opcode=" + hex(opcode, 6));
  }
  const auto mode = dsp56k::getFieldValue(type, dsp56k::Field_MMM, opcode);
  if (mode == 4 || mode == 5 || mode == 6) { return std::nullopt; }
  return dsp56k::getFieldValue(type, dsp56k::Field_RRR, opcode);
}

void validate_dual(const std::uint32_t opcode,
                   const DspAddressPhase &phase,
                   const std::array<std::uint32_t, 8> &before,
                   const std::array<std::uint32_t, 8> &native) {
  constexpr auto type = dsp56k::Movexy;
  const auto x = dsp56k::getRegister(type, dsp56k::Field_RRR, opcode);
  const auto y = dsp56k::getRegister(type, dsp56k::Field_rr, opcode);
  for (unsigned index = 0; index < before.size(); ++index) {
    const auto reg =
        static_cast<dsp56k::Register>(static_cast<unsigned>(dsp56k::Register::R0) + index);
    const bool updated = (reg == x && dsp56k::getFieldValue(type, dsp56k::Field_MM, opcode) != 0) ||
                         (reg == y && dsp56k::getFieldValue(type, dsp56k::Field_mm, opcode) != 0);
    if (updated) {
      const auto value = phase.published.at(index);
      if (!value || *value != native.at(index)) {
        throw Fault("Dual native addresses require actual prepublished AG2 results");
      }
    } else if (before.at(index) != native.at(index)) {
      throw Fault("Unqualified additional dual native address-register change");
    }
  }
}

bool dual(const std::uint32_t opcode) {
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  return opcodes.getInstructionTypes(opcode, first, second) != 0 &&
         (first == dsp56k::Movexy || second == dsp56k::Movexy);
}

}

void DspAddressPhase::capture(const std::uint32_t opcode,
                              const DspNativeAccess &access,
                              const std::array<std::uint32_t, 8> &before,
                              const std::array<std::uint32_t, 8> &native) {
  try {
    if (dual(opcode)) {
      validate_dual(opcode, *this, before, native);
      return;
    }
    const auto selected = address_register(opcode);
    for (unsigned index = 0; index < native.size(); ++index) {
      if (selected != index && native.at(index) != before.at(index)) {
        throw Fault("Unqualified additional native address-register change");
      }
    }
    if (!selected) { return; }
    if (const auto value = published.at(*selected)) {
      if (*value != native.at(*selected)) {
        throw Fault("Unqualified repeated native address-register update");
      }
      return;
    }
    pending = DspAddressUpdate{.index = *selected,
                               .before = before.at(*selected),
                               .value = native.at(*selected),
                               .space = access.target.space,
                               .address = access.target.address,
                               .operation = access.kind == DspNativeAccessKind::write
                                                ? DspMemoryOperation::write
                                                : DspMemoryOperation::read,
                               .pc = access.pc,
                               .omr = access.omr};
  } catch (const std::exception &error) {
    constexpr std::array spaces{"P:", "X:", "Y:"};
    throw Fault("Native address phase before " +
                std::string(access.kind == DspNativeAccessKind::write ? "write " : "read ") +
                spaces.at(static_cast<unsigned>(access.target.space)) +
                hex(access.target.address, 6) + " pc=" + hex(access.pc, 6) +
                " omr=" + hex(access.omr, 6) + ": " + error.what());
  }
}
}
