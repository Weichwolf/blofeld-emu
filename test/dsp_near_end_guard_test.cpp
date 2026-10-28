// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp_norm_fixture.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
constexpr unsigned test_omr = 0x300;
constexpr unsigned setup = 0x200, body = 0x202, last = 0x204, callee = 0x240;
constexpr unsigned finite_do = 0x060390, forever_do = 0x000203;
constexpr unsigned short_bsr = 0x050800, long_bsr = 0x0d1080, rts = 0x00000c;
constexpr unsigned split_shift = 5, high_shift = 6, low_mask = 31;
constexpr unsigned loop_flag = 0x8000, forever_flag = 0x10000;

DspLoopFetch opened(const bool forever) {
  const DspLoopArchitecture before{
      .pc = setup, .omr = test_omr, .la = 0x55aa33, .lc = 0x33, .status = 0xc00300};
  auto after = before;
  after.pc = body;
  after.la = last;
  after.lc = forever ? before.lc : 3U;
  after.sp = after.sc = 2;
  after.status |= loop_flag | (forever ? forever_flag : 0U);
  const DspProgramWord opcode{.address = setup, .value = forever ? forever_do : finite_do};
  const DspProgramWord extension{.address = setup + 1, .value = forever ? last : last - setup};
  DspLoopFetch fetch(before);
  require(!fetch.fetched(opcode) && !fetch.fetched(extension), "Own setup has no boundary");
  fetch.opened({.word = opcode, .extension = extension, .before = before, .after = after});
  return fetch;
}

struct GuardCase {
  unsigned distance;
  unsigned form;
  bool forever;
  unsigned variant;
};

bool check(const GuardCase sample) {
  const auto [distance, form, forever, variant] = sample;
  auto fetch = opened(forever);
  const auto pc = last - distance;
  const auto delta = callee - pc;
  const auto opcode = form == short_bsr
                          ? short_bsr | ((delta >> split_shift) << high_shift) | (delta & low_mask)
                          : form;
  for (unsigned address = body; address <= pc; ++address) {
    static_cast<void>(fetch.fetched({.address = address, .value = address == pc ? opcode : 0U}));
  }
  const auto prediction = fetch.prediction();
  const auto target = variant == 1 ? body : callee;
  fetch.discard_after(prediction, target);
  auto before = prediction.control();
  before.pc = pc;
  auto after = before;
  after.pc = target;
  const bool call = form != rts;
  after.sp = call ? before.sp + 1 : before.sp - 1;
  after.sc = call ? before.sc + 1 : before.sc - 1;
  switch (variant) {
    case 0:
    case 1: break;
    case 2: before.lc ^= 1U; break;
    case 3: before.sp ^= 1U; break;
    case 4: before.sc ^= 1U; break;
    case 5: before.omr ^= 1U; break;
    case 6: before.status ^= loop_flag; break;
    case 7: after.lc ^= 1U; break;
    case 8: after.sp ^= 1U; break;
    case 9: after.pc ^= 1U; break;
    default: throw Fault("Unknown directed guard variant");
  }
  try {
    fetch.predict_subroutine(
        {.address = pc, .value = opcode}, {.before = before, .after = after}, call);
    return true;
  } catch (const Fault &error) {
    require(std::string(error.what()).contains("Unqualified subroutine/active DO read P:"),
            "Visible exact guard failure");
    return false;
  }
}

struct Projection {
  DspProgramFetch fetch;
  DspLoopArchitecture observed;
  DspFetchedWord first;
  DspFetchedWord second;
  DspFetchedWord plain;
  DspFetchedWord boundary;
};

Projection projection(const bool call_at_end) {
  Projection result;
  auto &fetch = result.fetch;
  DspBus bus;
  constexpr unsigned own_call = 0x05085e; // Split displacement: own P202 -> P240.
  bus.write({.address = setup, .value = finite_do});
  bus.write({.address = setup + 1, .value = last - setup});
  bus.write({.address = body, .value = own_call});
  bus.write({.address = body + 1, .value = 0});
  bus.write({.address = last, .value = call_at_end ? own_call : 0U});
  fetch.start(setup, test_omr);
  const auto capture = [&] {
    fetch.issue();
    fetch.capture(bus);
    const auto sample = fetch.queued(fetch.view().used - 1);
    require(sample.has_value(), "Actual owned F2 receipt");
    return sample.value();
  };
  const auto opcode = capture();
  const auto extension = capture();
  static_cast<void>(fetch.consume(setup));
  static_cast<void>(fetch.consume(setup + 1));
  const DspLoopArchitecture before{
      .pc = setup, .omr = test_omr, .la = 0x55aa33, .lc = 0x33, .status = 0xc00300};
  auto after = before;
  after.pc = body;
  after.la = last;
  after.lc = 3;
  after.sp = after.sc = 2;
  after.status |= loop_flag;
  fetch.open_loop(
      {.word = opcode.word, .extension = extension.word, .before = before, .after = after},
      opcode,
      extension);
  require(fetch.confirm_opening(after, false), "Observed actual loop entry");
  result.observed = after;
  result.first = capture();
  result.plain = capture();
  result.boundary = capture();
  result.second = capture();
  return result;
}

void refuse_projection(const auto &operation) {
  try {
    operation();
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("read P:") && message.contains("pc=") && message.contains("omr="),
            "Projection refusal preserves address,operation and execution context");
    return;
  }
  throw Fault("Expected projection ownership/context refusal");
}

void projection_guards() {
  auto sample = projection(false);
  auto expected = sample.observed;
  expected.lc = 2;
  require(sample.fetch.subroutine_source(sample.first, sample.observed) == sample.observed &&
              sample.fetch.subroutine_source(sample.second, sample.observed) == expected,
          "Equal program addresses retain distinct occurrence-bound loop forecasts");
  auto other_pc = sample.observed;
  ++other_pc.pc;
  require(sample.fetch.subroutine_source(sample.second, other_pc) == expected,
          "Visible PC can precede this owned AG1 control operation");
  for (unsigned field = 0; field < 6; ++field) {
    auto bad = sample.observed;
    switch (field) {
      case 0: ++bad.omr; break;
      case 1: ++bad.la; break;
      case 2: ++bad.lc; break;
      case 3: ++bad.sp; break;
      case 4: ++bad.sc; break;
      case 5: bad.status ^= loop_flag; break;
      default: throw Fault("Unknown control projection field");
    }
    refuse_projection(
        [&] { static_cast<void>(sample.fetch.subroutine_source(sample.second, bad)); });
  }
  for (unsigned variant = 0; variant < 3; ++variant) {
    auto bad = sample.second;
    if (variant == 0) { bad.occurrence = 0; }
    if (variant == 1) { ++bad.occurrence; }
    if (variant == 2) { bad.word.value ^= 1U; }
    refuse_projection(
        [&] { static_cast<void>(sample.fetch.subroutine_source(bad, sample.observed)); });
  }
  refuse_projection(
      [&] { static_cast<void>(sample.fetch.subroutine_source(sample.plain, sample.observed)); });
  static_cast<void>(sample.fetch.consume(body));
  refuse_projection(
      [&] { static_cast<void>(sample.fetch.subroutine_source(sample.first, sample.observed)); });
  const auto endpoint = projection(true);
  refuse_projection([&] {
    static_cast<void>(endpoint.fetch.subroutine_source(endpoint.boundary, endpoint.observed));
  });
}

}

int main() {
  try {
    constexpr unsigned expected_accepted = 2;
    constexpr unsigned expected_refused = 178;
    projection_guards();
    unsigned accepted = 0;
    unsigned refused = 0;
    for (const auto distance : {0U, 1U, 2U}) {
      for (const auto form : {short_bsr, long_bsr, rts}) {
        for (const auto forever : {false, true}) {
          for (unsigned variant = 0; variant < 10; ++variant) {
            const bool expected = distance != 0 && form == short_bsr && !forever && variant == 0;
            require(
                check(
                    {.distance = distance, .form = form, .forever = forever, .variant = variant}) ==
                    expected,
                "Only finite short calls at LA-1/-2 retain exact control/extent checks");
            accepted += expected ? 1U : 0U;
            refused += expected ? 0U : 1U;
          }
        }
      }
    }
    require(accepted == expected_accepted && refused == expected_refused,
            "Complete directed guard catalogue");
    std::cout << "2 accepted controls;178 loop/call and12 projection refusals passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
