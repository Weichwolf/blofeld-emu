// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_peripheral_relative_fixture.hpp"

#include <iostream>

namespace blofeld::test::peripheral_relative {
namespace {
void one(const Input input) {
  DspMachine machine;
  const auto before = prepare(machine, input);
  const auto expected = branch_result(before, input);
  const auto saved = machine.checkpoint();
  for (unsigned repeat = 0; repeat < 3; ++repeat) {
    if (repeat != 0) { machine.restore(saved); }
    unsigned bits = 0;
    unsigned words = 0;
    machine.cpu().step({.observe = [&](const DspNativeAccess &access) {
      require(access.pc == input.start && access.omr == ordinary_omr,
              "Native relative branch callbacks retain their real opcode execution context");
      if (access.kind == DspNativeAccessKind::bit) {
        ++bits;
        const auto address = operand(input);
        require(access.target.space == address.space && access.target.address == address.address &&
                    access.bit && access.bit->index == input.bit &&
                    access.bit->operation == DspBitOperation::test && access.ordinal == 2,
                "Native BRCLR/BRSET performs its real selected-bit read after the extension");
        if (access.stage == DspNativeAccessStage::after) {
          require(access.value == static_cast<unsigned>(input.level),
                  "The native branch receives the old selected bit, never a fabricated success");
        }
      } else {
        ++words;
        require(access.kind == DspNativeAccessKind::opcode ||
                    access.kind == DspNativeAccessKind::extension,
                "The native branch has no word data read, write or stack effect");
      }
    }});
    require(bits == 2 && words == 4 && machine.cpu().debug_state() == expected,
            "Independent signed24 target/full CPU/CCR/stack/counter expectation differs");
  }
}

}

void native_checks() {
  for (unsigned port = 0; port < ports.size(); ++port) {
    for (unsigned bit = 0; bit < ports.at(port).bits; ++bit) {
      for (const bool set : {false, true}) {
        for (const bool level : {false, true}) {
          for (const bool flags : {false, true}) {
            for (const auto displacement : {32U, word_mask - 31}) {
              one({.port = port,
                   .bit = bit,
                   .set = set,
                   .level = level,
                   .flags = flags,
                   .displacement = displacement});
            }
          }
        }
      }
    }
  }
  constexpr unsigned wrapping_displacement = 18;
  constexpr unsigned negative_displacement = word_mask - 31;
  for (const bool set : {false, true}) {
    for (const bool level : {false, true}) {
      one({.bit = timer_bit, .set = set, .level = level, .flags = true, .timer = true});
      one({.set = set,
           .level = level,
           .start = word_mask - 1,
           .displacement = wrapping_displacement});
      one({.set = set,
           .level = level,
           .start = word_mask,
           .displacement = wrapping_displacement - 1});
      one({.set = set, .level = level, .start = 1, .displacement = negative_displacement});
    }
  }
  std::cout << "608 relative branches; two restored suffixes each\n";
}
}
