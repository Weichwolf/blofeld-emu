// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned base = 0x100;
constexpr unsigned target = base + 3;
constexpr unsigned short_branch = 0x050c03;
constexpr unsigned omr_branch = 0x0afa03;
constexpr unsigned omr = 0x305;
constexpr unsigned field_sign = 0x100;
constexpr unsigned field_limit = 0x200;
constexpr unsigned low_mask = 31;
constexpr unsigned address_mask = 0xffffff;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void fields() {
  for (unsigned raw = 0; raw < field_limit; ++raw) {
    const unsigned high = raw / (low_mask + 1);
    const unsigned low = raw % (low_mask + 1);
    constexpr unsigned high_shift = 64;
    constexpr unsigned prefix = 0x050c00;
    const unsigned opcode = prefix + (high * high_shift) + low;
    const unsigned expected = raw < field_sign ? raw : address_mask - (field_limit - raw) + 1;
    const auto form = decode_branch_form(opcode);
    require(
        form && form->displacement == expected,
        "All512 independent manual split-field/sign expectations match the admitted native form");
  }
  require(!decode_branch_form(0),
          "The relative control decoder cannot turn an ordinary NOP into a branch");
}

DspProgramWord queued_word(const DspProgramFetch &fetch, const unsigned index) {
  const auto actual = fetch.queued(index);
  if (!actual) { throw Fault("Own width fixture lost a genuine F2 word"); }
  return actual->word;
}

void width_refuses(const unsigned variant) {
  const bool two = variant >= 3;
  DspBus bus;
  bus.write({.address = base, .value = two ? omr_branch : short_branch});
  bus.write({.address = base + 1, .value = target});
  bus.write({.address = base + 2, .value = 0});
  DspProgramFetch fetch;
  fetch.start(base, omr);
  for (unsigned index = 0; index < 3; ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto actual = fetch.queued(0);
  if (!actual) { throw Fault("Own width fixture lost its actual branch occurrence"); }
  const DspBranchDecision valid{.word = actual->word,
                                .extension =
                                    two ? std::optional(queued_word(fetch, 1)) : std::nullopt,
                                .omr = omr,
                                .next_pc = target,
                                .taken = true};
  auto invalid = valid;
  auto identity = actual->occurrence;
  switch (variant) {
    case 0: invalid.extension = queued_word(fetch, 1); break;
    case 1: invalid.word.value ^= 1U; break;
    case 2: ++identity; break;
    case 3: invalid.extension.reset(); break;
    case 4: invalid.extension = queued_word(fetch, 2); break;
    default: throw Fault("Unknown directed width refusal");
  }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch(invalid, identity);
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("P:") && message.contains("pc=") && message.contains("omr=") &&
                fetch.view() == before,
            "Invalid width/value/identity fails before changing the genuine fetch queue");
    refused = true;
  }
  require(refused, "Invalid single/two-word branch ownership unexpectedly succeeded");
  fetch.predict_branch(valid, actual->occurrence);
  require(fetch.view().used == (two ? 2U : 1U) && fetch.view().next_address == target &&
              !fetch.view().pending_address,
          "A refused forecast must leave no hidden owner that could block the valid actual "
          "instruction");
}

void one_word_available() {
  DspBus bus;
  bus.write({.address = base, .value = short_branch});
  DspProgramFetch fetch;
  fetch.start(base, omr);
  fetch.issue();
  fetch.capture(bus);
  const auto actual = fetch.queued(0);
  if (!actual) { throw Fault("Actual one-word fixture lost F2"); }
  fetch.predict_branch({.word = actual->word, .omr = omr, .next_pc = target, .taken = true},
                       actual->occurrence);
  require(fetch.view().used == 1 && fetch.view().next_address == target,
          "A real short branch needs no invented or readable following extension word");
}
}

int main() {
  try {
    fields();
    for (unsigned variant = 0; variant < 5; ++variant) { width_refuses(variant); }
    one_word_available();
    std::cout << "512directed displacement fields,five ownership refusals and single-F2 forecast "
                 "passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
