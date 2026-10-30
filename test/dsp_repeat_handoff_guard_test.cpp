// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned entry = 0x100, ordinary_omr = 0x300;
constexpr unsigned rep_three = 0x0603a0, rep_two = 0x0602a0, rep_one = 0x0601a0;
unsigned guards = 0;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

std::array<DspFetchedWord, 4> receipts() {
  DspBus bus;
  const std::array code{rep_three, 0U, rep_two, rep_one};
  for (unsigned index = 0; index < code.size(); ++index) {
    bus.write({.address = entry + index, .value = code.at(index)});
  }
  DspProgramFetch fetch;
  fetch.start(entry, ordinary_omr);
  std::array<DspFetchedWord, 4> result{};
  for (std::size_t index = 0; index < code.size(); ++index) {
    fetch.issue();
    fetch.capture(bus);
    const auto word = fetch.queued(index);
    if (!word) { throw Fault("Own guard setup requires actual F1/F2 receipts"); }
    result.at(index) = *word;
  }
  return result;
}

bool same(const std::optional<DspPipelineRepeat> &left,
          const std::optional<DspPipelineRepeat> &right) {
  if (!left || !right) { return left.has_value() == right.has_value(); }
  return left->prefix == right->prefix && left->body == right->body &&
         left->remaining == right->remaining && left->setup_hold == right->setup_hold &&
         left->held_decode == right->held_decode;
}

DspPipelineRepeat &repeat(DspInstructionPipeline &candidate) {
  if (!candidate.repeat_setup) { throw Fault("Own guard setup lost its declared repeat owner"); }
  return *candidate.repeat_setup;
}

void refuses(DspInstructionPipeline candidate) {
  const auto before = candidate.repeat_setup;
  const auto samples = candidate.samples;
  try {
    candidate.accept_repeat_decode();
  } catch (const Fault &error) {
    require(
        std::string(error.what()).contains("Unqualified overlapping automatic REP setup read P:"),
        "Unsupported overlap must retain an explicit operation/address diagnostic");
    require(same(candidate.repeat_setup, before) && candidate.samples == samples,
            "Rejected overlap must preserve old ownership and every actual sampled receipt");
    ++guards;
    return;
  }
  throw Fault("Invalid REP overlap was silently accepted");
}

void negative(const std::array<DspFetchedWord, 4> &words) {
  DspInstructionPipeline ready;
  ready.samples.front() = words[2];
  ready.repeat_setup = {.prefix = words[0], .body = words[1], .setup_hold = 0};
  auto candidate = ready;
  repeat(candidate).remaining = 1;
  refuses(candidate);
  candidate = ready;
  repeat(candidate).body.reset();
  refuses(candidate);
  candidate = ready;
  repeat(candidate).setup_hold = 1;
  refuses(candidate);
  for (const auto prefix : {words[1], words[2], words[3]}) {
    candidate = ready;
    repeat(candidate).prefix = prefix;
    refuses(candidate);
  }
  for (const auto body : {words[0], words[2], words[3]}) {
    candidate = ready;
    repeat(candidate).body = body;
    refuses(candidate);
  }
  candidate = ready;
  candidate.branch = DspBranchDecision{.word = words[0].word};
  refuses(candidate);
  candidate = ready;
  candidate.loop_setup = {.captured = {.opcode = words[0], .extension = words[1]},
                          .target = std::nullopt};
  refuses(candidate);
}

void positive(const std::array<DspFetchedWord, 4> &words) {
  DspInstructionPipeline initial;
  initial.samples.front() = words[0];
  initial.accept_repeat_decode();
  require(initial.repeat_setup && initial.repeat_setup->prefix == words[0] &&
              !initial.repeat_setup->body && initial.repeat_setup->setup_hold == 4,
          "An initial actual REP Decode starts its ordinary setup without a prior body");
  DspInstructionPipeline drained;
  drained.samples.front() = words[2];
  drained.repeat_setup = {.prefix = words[0], .body = words[1], .setup_hold = 0};
  drained.accept_repeat_decode();
  require(drained.repeat_setup && drained.repeat_setup->prefix == words[2] &&
              drained.repeat_setup->body == words[1] && drained.repeat_setup->remaining == 0 &&
              drained.repeat_setup->setup_hold == 4,
          "A younger setup retains the actual older F2 body while its native E2 drains");
  drained.samples.front() = words[3];
  refuses(drained);
}
}

int main() {
  try {
    const auto words = receipts();
    negative(words);
    positive(words);
    std::cout << "PASS REP_HANDOFF_GUARDS refusals=" << guards << " positive=2\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
