// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_branch_form.hpp"

#include <exception>
#include <initializer_list>
#include <iostream>
#include <stdexcept>

namespace {
constexpr unsigned short_relative_call = 0x050800;
constexpr unsigned effective_call = 0x0bc080;
constexpr unsigned word_bits = 24;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

void short_fields() {
  constexpr unsigned address_count = 0x1000;
  constexpr unsigned condition_count = 16;
  for (unsigned condition = 0; condition < condition_count; ++condition) {
    for (unsigned address = 0; address < address_count; ++address) {
      const auto opcode = 0x0e0000U | (condition << 12U) | address;
      const auto form = blofeld::decode_branch_form(opcode);
      require(form && form->absolute_short == address && !form->subroutine_call &&
                  blofeld::branch_condition_code(opcode) == condition,
              "Independent short Jcc target/condition fields differ");
    }
  }
  constexpr unsigned offset_count = 512;
  constexpr unsigned sign = 256;
  for (unsigned raw = 0; raw < offset_count; ++raw) {
    const auto bits = ((raw & 0x1e0U) << 1U) | (raw & 0x1fU);
    const auto expected = raw < sign ? raw : 0x1000000U - (offset_count - raw);
    for (const auto prefix : {short_relative_call, 0x050c00U}) {
      const auto form = blofeld::decode_branch_form(prefix | bits);
      require(form && form->displacement == expected &&
                  form->subroutine_call == (prefix == short_relative_call),
              "Independent split relative displacement/call field differs");
    }
    for (unsigned condition = 0; condition < condition_count; ++condition) {
      const auto opcode = 0x050400U | (condition << 12U) | bits;
      const auto form = blofeld::decode_branch_form(opcode);
      require(form && form->displacement == expected && !form->subroutine_call &&
                  blofeld::branch_condition_code(opcode) == condition,
              "Independent conditional relative field differs");
    }
  }
}

void effective_forms() {
  for (unsigned mode = 0; mode < 8; ++mode) {
    for (unsigned reg = 0; reg < 8; ++reg) {
      for (const auto prefix : {0x0ac080U, effective_call}) {
        const auto form = blofeld::decode_branch_form(prefix | (mode << 11U) | (reg << 8U));
        if (mode == 4) {
          require(form && form->address_register == reg &&
                      form->subroutine_call == (prefix == effective_call),
                  "Direct jump register field differs");
        } else if (mode == 6 && reg == 0) {
          require(form && form->absolute_extension &&
                      form->subroutine_call == (prefix == effective_call),
                  "Extended jump ownership differs");
        } else {
          require(!form, "Unsupported effective-address form was accepted");
        }
      }
    }
  }
}

void explicit_forms() {
  for (unsigned address = 0; address < 0x1000; ++address) {
    const auto form = blofeld::decode_branch_form(0x0d0000U | address);
    require(form && form->absolute_short == address && form->subroutine_call,
            "Independent short JSR target differs");
  }
  for (unsigned bit = 0; bit < 32; ++bit) {
    for (const bool set : {false, true}) {
      const auto form = blofeld::decode_branch_form(0x0afa00U | bit | (set ? 0x20U : 0U));
      require(bit < word_bits ? form && form->bit == bit && form->set == set : !form,
              "OMR bit branch field or range differs");
    }
  }
  const auto call = blofeld::decode_branch_form(0x0d1080U);
  const auto ret = blofeld::decode_branch_form(0x00000cU);
  require(call && call->relative_extension && call->subroutine_call && ret &&
              ret->subroutine_return,
          "Extended call and RTS must retain their distinct forms");
  for (const auto opcode : {0U, 0x0c0000U, 0x050000U, 0x0d10c0U, 0x0f0000U, 0x1000000U}) {
    require(!blofeld::decode_branch_form(opcode) && !blofeld::branch_condition_code(opcode),
            "Unsupported or out-of-range opcode was accepted");
  }
}
}

int main() {
  try {
    short_fields();
    effective_forms();
    explicit_forms();
    std::cout << "Own DSP branch forms match independent field expectations.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
