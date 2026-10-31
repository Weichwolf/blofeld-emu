// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp_peripheral_read_encodings.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

void assembled_forms() {
  for (const auto &expected : blofeld::test::peripheral_read::encodings) {
    const auto form = blofeld::decode_peripheral_read(expected.opcode);
    require(form && form->target.space == expected.space &&
                form->target.address == expected.address && form->bit == expected.bit &&
                form->set == expected.set && !form->relative && form->words == expected.words,
            "Peripheral form differs from independently assembled instruction");
  }
}

struct RelativeInput {
  unsigned fields, address, bit;
};

void relative_form(const RelativeInput input) {
  constexpr unsigned base_low = 0x048000;
  constexpr unsigned base_high = 0x0cc000;
  constexpr unsigned address_low = 0xffff80;
  constexpr unsigned address_high = 0xffffc0;
  constexpr unsigned word_bits = 24;
  constexpr unsigned operand_shift = 8;
  constexpr unsigned space_bit = 0x40;
  constexpr unsigned set_bit = 0x20;
  const bool high = (input.fields & 1U) != 0;
  const bool y = (input.fields & 2U) != 0;
  const bool set = (input.fields & 4U) != 0;
  const auto opcode = (high ? base_high : base_low) | (input.address << operand_shift) |
                      (y ? space_bit : 0U) | (set ? set_bit : 0U) | input.bit;
  const auto form = blofeld::decode_peripheral_read(opcode);
  if (input.bit >= word_bits) {
    require(!form, "Reserved peripheral bit was accepted");
    return;
  }
  require(form && form->target.address == (high ? address_high : address_low) + input.address &&
              form->target.space == (y ? blofeld::DspSpace::y : blofeld::DspSpace::x) &&
              form->bit == input.bit && form->set == set && form->relative && form->words == 2,
          "Relative peripheral form differs from instruction fields");
}

void relative_forms() {
  constexpr unsigned address_count = 64;
  constexpr unsigned bit_count = 32;
  for (unsigned fields = 0; fields < 8; ++fields) {
    for (unsigned address = 0; address < address_count; ++address) {
      for (unsigned bit = 0; bit < bit_count; ++bit) {
        relative_form({.fields = fields, .address = address, .bit = bit});
      }
    }
  }
}

void unsupported_forms() {
  constexpr std::array refused{0U,
                               0x018098U,
                               0x01809fU,
                               0x04d190U,
                               0x08d03cU,
                               0x01c080U,
                               0x048080U,
                               0x0b8080U,
                               0x0dc000U,
                               0x1000000U};
  for (const auto opcode : refused) {
    require(!blofeld::decode_peripheral_read(opcode), "Unqualified peripheral form was accepted");
  }
}
}

int main() {
  try {
    assembled_forms();
    relative_forms();
    unsupported_forms();
    std::cout << "Own DSP peripheral read forms match independent expectations.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
