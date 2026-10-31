// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_omr_move.hpp"
#include "dsp/dsp_sp_move.hpp"
#include "dsp/dsp_sr_move.hpp"

#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
constexpr unsigned omr_selector = 0x1a;
constexpr unsigned sp_selector = 0x1b;
constexpr unsigned sr_selector = 0x19;

void require(const bool value, const char *message) {
  if (!value) { throw std::runtime_error(message); }
}

template <typename Decoder> void fields(const unsigned destination, const Decoder &decode) {
  constexpr unsigned register_base = 0x04c0a0;
  constexpr unsigned immediate_base = 0x0500a0;
  constexpr unsigned extension_base = 0x05f420;
  constexpr unsigned unqualified_bit = 0x40;
  for (unsigned source = 0; source < 64; ++source) {
    const bool expected = (source >= 4 && source <= 7) || (source >= 16 && source <= 23);
    const auto opcode = register_base | (source << 8U) | destination;
    const auto result = decode(opcode);
    require(result.has_value() == expected && (!result || result->words == 1),
            "MOVEC register source/destination expectation differs");
    require(!decode(opcode | unqualified_bit),
            "The own core must not predict an unsupported register encoding");
  }
  for (unsigned value = 0; value < 256; ++value) {
    const auto result = decode(immediate_base | (value << 8U) | destination);
    require(result && result->words == 1, "Immediate control write must consume one word");
  }
  const auto extension = decode(extension_base | destination);
  require(extension && extension->words == 2, "Full immediate must consume two words");
  require(!decode(0x1000000U), "A value outside the 24-bit opcode space was accepted");
}
}

int main() {
  try {
    fields(omr_selector, blofeld::decode_omr_move);
    fields(sp_selector, blofeld::decode_sp_move);
    fields(sr_selector, blofeld::decode_sr_move);
    std::cout << "Own DSP control move fields match independent expectations.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
