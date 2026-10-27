// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_control_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned base = 0x100;
constexpr unsigned target = 0x120;
constexpr unsigned jump_ea = 0x0ac080;
constexpr unsigned and_immediate = 0x0000b8;

void fields() {
  for (unsigned reg = 0; reg < 8; ++reg) {
    for (unsigned mode = 0; mode < 8; ++mode) {
      const unsigned opcode = jump_ea | (mode << 11U) | (reg << 8U);
      const auto form = decode_branch_form(opcode);
      if (mode == 4) {
        require(form && form->address_register == reg && !form->displacement &&
                    pipeline_word_count({.address = base, .value = opcode}) == 1,
                "Direct mode admits every literal R field as one actual word");
      } else if (mode == 6 && reg == 0) {
        require(form && form->absolute_extension && !form->address_register &&
                    !form->displacement &&
                    pipeline_word_count({.address = base, .value = opcode}) == 2,
                "Absolute EA110000 requires its genuine second instruction word");
      } else {
        require(!form, "Other JMP addressing modes retain visible unsupported status");
      }
    }
  }
}

void refuse_decode(const unsigned opcode) {
  bool refused = false;
  try {
    static_cast<void>(decode_pipeline_word(
        {.address = base, .value = opcode}, reset_omr, DspFetchPipeline::automatic_memory));
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("P:" + hex(base, 6)) && message.contains("opcode=") &&
                message.contains("pc=" + hex(base, 6)) && message.contains("omr="),
            "Unsupported control form retains address, operation, PC and mode context");
    refused = true;
  }
  require(refused, "Unsupported control register/mode must not become an accepted no-op");
}

void ownership(const unsigned variant) {
  DspBus bus;
  bus.write({.address = base, .value = control::indirect_jump});
  bus.write({.address = base + 1, .value = 0});
  DspProgramFetch fetch;
  fetch.start(base, reset_omr);
  for (unsigned index = 0; index < 2; ++index) {
    fetch.issue();
    fetch.capture(bus);
  }
  const auto actual = fetch.queued(0);
  const auto next = fetch.queued(1);
  if (!actual || !next) { throw Fault("Both fixture words must come from actual F2 captures"); }
  const DspBranchDecision valid{
      .word = actual->word, .omr = reset_omr, .next_pc = target, .taken = true};
  auto invalid = valid;
  auto occurrence = actual->occurrence;
  if (variant == 0) {
    invalid.extension = next->word;
  } else if (variant == 1) {
    invalid.word.value ^= 0x100U;
  } else {
    ++occurrence;
  }
  const auto before = fetch.view();
  bool refused = false;
  try {
    fetch.predict_branch(invalid, occurrence);
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(fetch.view() == before && message.contains("P:") && message.contains("pc=") &&
                message.contains("omr="),
            "Invalid indirect width/value/receipt leaves the actual queue unchanged");
    refused = true;
  }
  require(refused, "Invalid indirect forecast must be refused");
  fetch.predict_branch(valid, actual->occurrence);
  require(fetch.view().used == 1 && fetch.view().next_address == target,
          "The valid R0 forecast needs no invented extension after a refused request");
}

void one_word() {
  DspBus bus;
  bus.write({.address = base, .value = control::indirect_jump});
  DspProgramFetch fetch;
  fetch.start(base, reset_omr);
  fetch.issue();
  fetch.capture(bus);
  const auto actual = fetch.queued(0);
  if (!actual) { throw Fault("Direct R0 forecast owns one actual F2 word"); }
  fetch.predict_branch({.word = actual->word, .omr = reset_omr, .next_pc = target, .taken = true},
                       actual->occurrence);
  require(fetch.view().used == 1 && fetch.view().next_address == target,
          "Direct R0 forecast works without any readable following instruction word");
}
}

int main() {
  try {
    fields();
    for (const unsigned selector : {0U, 2U, 3U}) { refuse_decode(and_immediate | selector); }
    for (const unsigned mode : {0U, 1U, 2U, 3U, 5U, 7U}) { refuse_decode(jump_ea | (mode << 11U)); }
    constexpr unsigned immediate = 0x0af480;
    refuse_decode(immediate);
    for (unsigned variant = 0; variant < 3; ++variant) { ownership(variant); }
    one_word();
    std::cout << "64 control fields,ten Decode refusals,three ownership refusals and one-F2 R0 "
                 "forecast passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
