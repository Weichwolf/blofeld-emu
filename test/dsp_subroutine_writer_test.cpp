// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned base = 0x100, target = 0x180, omr = 0x300;
constexpr unsigned sp = 0x0500bb, sr = 0x0500b9, sc = 0x0500b8, jsr = 0x0bf080;
constexpr unsigned retained_la = 0x321, masked_status = 0x300;

void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

void one_case(const unsigned variant, const bool status) {
  DspBus bus;
  const auto opcode = status ? sr : sp;
  const std::array words{variant == 10 ? sc : opcode, jsr, target};
  for (unsigned index = 0; index < words.size(); ++index) {
    bus.write({.address = base + index, .value = words.at(index)});
  }
  bus.write({.address = target});
  DspProgramFetch fetch;
  fetch.start(base, omr);
  for (unsigned index = 0; index < words.size(); ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto writer_word = fetch.select(base);
  const auto call = fetch.select(base + 1);
  const auto extension = fetch.select(base + 2);
  DspLoopArchitecture before{.pc = base,
                             .omr = omr,
                             .la = retained_la,
                             .lc = 7,
                             .sp = 2,
                             .sc = 2,
                             .status = masked_status};
  if (variant == 13) {
    if (status) {
      before.status = 0;
    } else {
      before.sp = 0;
    }
  }
  auto call_inputs = before;
  call_inputs.pc = base + 1;
  auto predicted = call_inputs;
  predicted.pc = target;
  predicted.sp = before.sp + 1;
  predicted.sc = 3;
  fetch.predict_branch(
      {.word = call.word,
       .extension = extension.word,
       .omr = omr,
       .next_pc = target,
       .taken = true,
       .subroutine = DspSubroutineControl{.before = call_inputs, .after = predicted}},
      call.occurrence);
  static_cast<void>(fetch.consume(base));
  DspFetchedInstruction writer{.opcode = writer_word, .extension = std::nullopt};
  auto after = before;
  after.pc = base + 1;
  if (status) {
    after.status = 0;
  } else {
    after.sp = 0;
  }
  switch (variant) {
    case 0: break;
    case 1: ++writer.opcode.occurrence; break;
    case 2: writer.opcode.word.value ^= 1U; break;
    case 3: writer.extension = call; break;
    case 4: ++before.sp; break;
    case 5: ++before.sc; break;
    case 6: ++after.sc; break;
    case 7: ++after.lc; break;
    case 8: after.sp = 15; break;
    case 9: ++after.omr; break;
    case 10: break;
    case 11: ++before.pc; break;
    case 12: after.pc = target; break;
    case 13: break;
    default: throw Fault("Unknown subroutine writer case");
  }
  const auto retained = fetch.view();
  bool refused = false;
  try {
    fetch.follow_subroutine_writer(writer, before, after);
  } catch (const Fault &) { refused = true; }
  require(refused == (variant != 0 && variant != 13) && fetch.view() == retained,
          "Invalid predecessor refuses without changing captured fetch/target state");
  fetch.issue();
  fetch.capture(bus);
  static_cast<void>(fetch.consume(base + 1));
  static_cast<void>(fetch.consume(base + 2));
  if (variant == 0) {
    if (status) {
      predicted.status = 0;
    } else {
      predicted.sp = 1;
    }
  }
  require(fetch.confirm_subroutine(
              {.pc = base + 1, .next_pc = target, .omr = omr, .redirected = true}, predicted),
          "Only accepted older control changes affect the independent call forecast");
}
}

int main() {
  try {
    for (const bool status : {false, true}) {
      for (unsigned variant = 0; variant <= 13; ++variant) { one_case(variant, status); }
    }
    std::cout << "Subroutine writers=4 accepted/24 refused;owned F2 and forecast retained\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
