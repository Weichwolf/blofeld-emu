// SPDX-License-Identifier: GPL-3.0-only
#include "audio/serial_pcm.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_pcm_native.hpp"
#include "dsp_rom_program.hpp"
#include "serial_pcm_fixture.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld::test::pcm {
namespace {
constexpr std::uint64_t start = 2000;
constexpr std::uint64_t half_period = 23;
constexpr std::uint64_t observations = 516;
constexpr std::uint64_t end = start + ((observations - 1) * half_period);
constexpr std::uint64_t baseline_tick = 1200;
constexpr std::uint64_t capture = 6000;

void native_case(const bool second) {
  constexpr std::uint64_t common = 540000000;
  constexpr unsigned budget = 1024;
  DspPcmRun prefix({.timebase = common,
                    .baseline = baseline_tick,
                    .start = start,
                    .half_period = half_period,
                    .observations = observations,
                    .secondary = second});
  prefix.advance_to(capture, budget);
  require(prefix.frames().empty() && prefix.receiver().locked() && prefix.observed() > 1,
          "The joint snapshot is inside an actually transmitted native word");
  const auto saved = prefix.checkpoint();
  std::optional<dsp56300::Registers> completed;
  std::vector<SerialPcmFrame> expected;
  for (unsigned replay = 0; replay < 3; ++replay) {
    DspPcmRun run(saved);
    require(prefix.machine().cpu().registers() == run.machine().cpu().registers() &&
                prefix.machine().cpu().progress() == run.machine().cpu().progress() &&
                prefix.receiver() == run.receiver() && prefix.observed() == run.observed(),
            "A fresh owner restores the actual native CPU and partial word at the saved boundary");
    run.advance_to(end, budget);
    // Four reset periods, then303 real45-tick periods through13815. The source
    // continues through13845; the next native retirement is still due at13860.
    constexpr unsigned completed_periods = 303;
    constexpr std::uint64_t completed_ticks = 13815;
    constexpr std::uint64_t next_completion = 13860;
    require(run.machine().clock().native_cycle_units() == completed_periods &&
                run.machine().clock().elapsed_ticks() == completed_ticks &&
                run.next_deadline() == next_completion,
            "The receiver cannot charge a future DSP effect or consume a pending retirement");
    // Six MOVEPs need6+5*3 periods;64 NOPs need128; the refill needs3. The
    // remaining151 periods retire25 NOP/JMP pairs, then publish the next NOP PC.
    constexpr unsigned retired = 6 + 64 + 1 + (25 * 2);
    constexpr unsigned loop = DspRomProgram::boot + 78;
    const auto pending = run.machine().cpu().pending_instruction();
    require(run.machine().cpu().progress().instructions == retired &&
                run.machine().cpu().registers().pc == loop + 1 && pending &&
                pending->effect.kind == dsp56300::EffectKind::retire && pending->effect.pc == loop,
            "The literal frontend ledger retains the actual unretired NOP at the cutoff");
    constexpr std::int32_t signed_first = -5913697;
    constexpr std::int32_t signed_next = 1193046;
    require(run.observed() == observations + 1 && run.frames().size() == 1 &&
                run.receiver().completed_frames() == 1,
            "Exactly one frame comes from the actual finite native ESAI clock source");
    const auto &frame = run.frames().front();
    constexpr std::uint64_t first_sample = 2230;
    constexpr std::uint64_t last_sample = 13822;
    require(frame.channels == 2 && frame.samples[0] == signed_first &&
                frame.samples[1] == signed_next && frame.first_tick == first_sample &&
                frame.last_tick == last_sample,
            "The actual native shifter retains its first word and refills the next slot");
    if (!completed) {
      completed = run.machine().cpu().registers();
      expected = run.frames();
    }
    require(run.machine().cpu().registers() == *completed && run.frames() == expected,
            "Joint fresh-owner CPU/divider/source/receiver copies reproduce exact PCM and time");
  }
}
}

void native_serial_pcm() {
  native_case(false);
  native_case(true);
}
}
