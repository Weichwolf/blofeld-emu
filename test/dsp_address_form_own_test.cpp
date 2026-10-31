// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_address_only.hpp"

#include <exception>
#include <initializer_list>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

void address_fields() {
  // FM13-117 U parallel move: 00100000010MMRRR followed by the ALU byte.
  constexpr unsigned base = 0x204000;
  constexpr unsigned modes = 4;
  constexpr unsigned registers = 8;
  constexpr unsigned alu_values = 256;
  constexpr unsigned mode_shift = 11;
  constexpr unsigned register_shift = 8;
  for (unsigned mode = 0; mode < modes; ++mode) {
    for (unsigned reg = 0; reg < registers; ++reg) {
      for (unsigned alu = 0; alu < alu_values; ++alu) {
        const auto opcode = base | (mode << mode_shift) | (reg << register_shift) | alu;
        const auto form = blofeld::decode_address_only(opcode);
        require(form && form->mode == mode && form->reg == reg,
                "Address update mode or register differs from encoded fields");
      }
    }
  }
  for (const auto opcode : {0U, 0x203fffU, 0x206000U, 0x1000000U}) {
    require(!blofeld::decode_address_only(opcode), "Non-address-update opcode was accepted");
  }
}
}

int main() {
  try {
    address_fields();
    std::cout << "Own DSP address forms match independent field expectations.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
