// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <optional>

namespace {
using namespace blofeld;
using test::esai::require;
constexpr unsigned load_x0 = 0x44f000; // FM MOVE X:ea,X0, full address extension.
constexpr unsigned bound = 32;
constexpr std::array controls{0xffff8fU, 0xffff8bU, 0xffff87U};
constexpr unsigned seed = 0x123456, compare = 0xffffff;

struct Input {
  unsigned timer;
  unsigned offset;
  bool reject;
};

void sample(const Input input) {
  const auto [timer, offset, reject] = input;
  OwnDspMachine machine(test::dma_test::native_clocks);
  const auto control = controls.at(timer);
  const auto address = control - offset;
  test::dma_test::NativeProgram program;
  program.word(load_x0);
  program.word(address);
  program.word(0);
  test::dma_test::reset_native(machine, program);
  machine.bus().timers().write({.address = control - 1, .value = seed});
  machine.bus().timers().write({.address = control - 2, .value = compare});
  machine.bus().timers().write({.address = control, .value = 1});
  OwnDspTimeline timeline(
      machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
  std::optional<unsigned> sampled;
  const auto initial_edges = machine.clock().timer_clk2_edges();
  for (unsigned cycle = 0; cycle < bound; ++cycle) {
    // CLK/2 starts on the first completed reference pair; first edge loads TLR.
    const auto edges = machine.clock().timer_clk2_edges() - initial_edges;
    const auto expected = edges == 0 ? 0U : seed + static_cast<unsigned>(edges - 1);
    const auto before = machine.cpu().pending_instruction();
    const auto deadline = timeline.next_deadline();
    const auto result = timeline.advance_one(deadline,
                                             OwnDspBoundary::through,
                                             {.cycle = cycle,
                                              .known_requests = 15,
                                              .timing = DspDmaTiming::reference_sampled_cycles,
                                              .grant = DspDmaGrant::available});
    if (!result) { throw Fault("Timer sample missed its actual reference deadline"); }
    if (result->native && result->native->fault.kind != dsp56300::FaultKind::none) {
      const auto &failure = machine.io().failure();
      require(reject && timeline.faulted() && failure && failure->address == address &&
                  machine.cpu().progress().instructions == 0,
              "Write-only/unknown timer samples retain a real unretired contextual fault");
      return;
    }
    const bool owned_read = before && before->effect.kind == dsp56300::EffectKind::read &&
                            before->effect.space == dsp56300::Space::x &&
                            before->effect.address == address;
    // For this captured MOVE, admission may perform its first data-read effect
    // in the same cycle. Later effects retain their actual pending descriptor.
    if ((result->instruction_started || owned_read) && result->native) {
      require(!reject && !sampled, "Exactly one actual successful timer sample occurs");
      sampled = offset == 3 ? expected : compare;
    }
    if (result->native && result->native->retired) {
      require(!reject && sampled && machine.cpu().progress().instructions == 1 &&
                  (machine.cpu().registers().x & dsp56300::word_mask) == *sampled,
              "One real native timer read retires with its pre-edge sample");
      return;
    }
  }
  throw Fault("Timer sample did not complete its bounded real instruction");
}
}

int main() {
  try {
    for (unsigned timer = 0; timer < controls.size(); ++timer) {
      sample({.timer = timer, .offset = 3, .reject = false});
      sample({.timer = timer, .offset = 2, .reject = false});
      sample({.timer = timer, .offset = 1, .reject = true});
      if (timer < 2) { sample({.timer = timer, .offset = 0, .reject = true}); }
    }
    std::cout << "Native timer reads: all3 nonzero running counters/compares and write-only faults "
                 "passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
