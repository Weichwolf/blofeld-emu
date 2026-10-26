// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_peripheral_read_form.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp56kEmu/opcodes.h"
#include "dsp56kEmu/opcodetypes.h"

#include <cstdint>
#include <optional>

namespace blofeld {
namespace {
using Type = dsp56k::Instruction;
constexpr unsigned high_begin = 0xffffc0;
constexpr unsigned low_begin = 0xffff80;

std::optional<DspPeripheralReadForm> branch(const Type type, const unsigned opcode) {
  const bool high = type == dsp56k::Jset_pp || type == dsp56k::Jclr_pp;
  const bool low = type == dsp56k::Jset_qq || type == dsp56k::Jclr_qq;
  if (!high && !low) { return std::nullopt; }
  const auto bit = dsp56k::getFieldValue(type, dsp56k::Field_bbbbb, opcode);
  constexpr unsigned bits = 24;
  if (bit >= bits) { return std::nullopt; }
  const auto field = high ? dsp56k::Field_pppppp : dsp56k::Field_qqqqqq;
  const bool y = dsp56k::getFieldValue(type, dsp56k::Field_S, opcode) != 0;
  return DspPeripheralReadForm{.target = {.space = y ? DspSpace::y : DspSpace::x,
                                          .address = (high ? high_begin : low_begin) +
                                                     dsp56k::getFieldValue(type, field, opcode)},
                               .bit = bit,
                               .set = type == dsp56k::Jset_pp || type == dsp56k::Jset_qq,
                               .words = 2};
}

std::optional<DspPeripheralReadForm> word(const Type type, const unsigned opcode) {
  const bool high = type == dsp56k::Movep_Spp;
  if ((!high && type != dsp56k::Movep_SXqq && type != dsp56k::Movep_SYqq) ||
      dsp56k::getFieldValue(type, dsp56k::Field_W, opcode) != 0) {
    return std::nullopt;
  }
  const bool y = type == dsp56k::Movep_SYqq ||
                 (high && dsp56k::getFieldValue(type, dsp56k::Field_s, opcode) != 0);
  const auto offset =
      high ? dsp56k::getFieldValue(type, dsp56k::Field_pppppp, opcode)
           : dsp56k::getFieldValue(type, dsp56k::Field_q, dsp56k::Field_qqqqq, opcode);
  return DspPeripheralReadForm{.target = {.space = y ? DspSpace::y : DspSpace::x,
                                          .address = (high ? high_begin : low_begin) + offset}};
}
}

std::optional<DspPeripheralReadForm> decode_peripheral_read(const std::uint32_t opcode) {
  constexpr unsigned word_mask = 0xffffff;
  if (opcode > word_mask) { return std::nullopt; }
  static const dsp56k::Opcodes opcodes;
  auto first = dsp56k::Invalid;
  auto second = dsp56k::Invalid;
  if (opcodes.getInstructionTypes(opcode, first, second) == 0 || second != dsp56k::Invalid) {
    return std::nullopt;
  }
  if (const auto form = branch(first, opcode)) { return form; }
  return word(first, opcode);
}
}
