// SPDX-License-Identifier: GPL-3.0-only
#include "scheduled_frontier_fixture.hpp"

#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "coupled_boot_fixture.hpp"
#include "original_owned_clock_fixture.hpp"
#include "scheduled_coupling_fixture.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>

namespace blofeld::test::scheduled {
namespace {
constexpr unsigned near_frontier = coupled::long_ram_end - 32;
constexpr unsigned delay_loop = 0xb9ad;

struct Replay {
  McuCheckpoint mcu;
  DspCheckpoint dsp;
  LinkCheckpoint link;
  Progress progress;
  std::size_t offset = 0;
};

void record(const std::string &name,
            const McuMachine &mcu,
            const DspMachine &dsp,
            const ClockedLink &link,
            const Progress &progress,
            std::ostream &trace) {
  const auto state = dsp.cpu().state();
  const auto &clock = mcu.run_clock();
  trace << name << " mcu-pc=" << hex(mcu.cpu().state().pc) << " x=" << mcu.cpu().state().x
        << " retired=" << progress.census.retired << " cycles=" << clock.bus_cycles()
        << " ticks=" << clock.elapsed_ticks() << " OSC=" << clock.oscillator_cycles()
        << " common=" << common_ticks(mcu) << " dsp-steps=" << link.steps()
        << " dsp-units=" << dsp.run_clock().native_cycle_units()
        << " dsp-ticks=" << dsp.run_clock().elapsed_ticks() << " dsp-pc=" << hex(state.pc, 6)
        << " r4=" << state.r[4] << " lc=" << state.lc << '\n';
}

std::string until_fault(McuMachine &mcu,
                        DspMachine &dsp,
                        ClockedLink &link,
                        Progress &progress,
                        std::ostringstream &trace,
                        std::optional<Replay> *replay) {
  link.attach();
  constexpr unsigned step_limit = 10000;
  unsigned steps = 0;
  try {
    for (; steps < step_limit; ++steps) {
      const auto before = mcu.cpu().state();
      require(before.pc == coupled::sender_end || before.pc == delay_loop,
              "The original DSP frontier must precede completion of the MCU's final delay");
      trace << "MCU-BEGIN pc=" << hex(before.pc) << " x=" << before.x
            << " cycles=" << mcu.run_clock().bus_cycles() << '\n';
      step(mcu, progress);
      link.after_mcu_step();
      if (replay != nullptr && !replay->has_value() && dsp.cpu().state().r[4] >= near_frontier) {
        record("near", mcu, dsp, link, progress, trace);
        *replay = Replay{.mcu = mcu.checkpoint(),
                         .dsp = dsp.checkpoint(),
                         .link = link.checkpoint(),
                         .progress = progress,
                         .offset = trace.str().size()};
      }
    }
  } catch (const Fault &fault) {
    link.detach();
    const std::string error = fault.what();
    require(error.contains("Unmapped DSP memory write at Y:0x002000") &&
                error.contains("mcu-pc=0xb9ad") && error.contains("dsp-pc=") &&
                error.contains("attempted-completion="),
            "The original joint path must fail at its actual reserved write with both PCs: " +
                error);
    require(mcu.timing_faulted() && dsp.timing_faulted(),
            "Both time owners retain a visible fault after interrupted peer execution");
    record("fault", mcu, dsp, link, progress, trace);
    trace << "rejected " << error << '\n';
    return error;
  } catch (...) {
    link.detach();
    throw;
  }
  link.detach();
  throw Fault("Original scheduled continuation exceeded its bound without the reserved write");
}

void reject_faulted_capture(const McuMachine &mcu, const DspMachine &dsp, const ClockedLink &link) {
  const auto rejects = [](const auto &capture) {
    try {
      capture();
    } catch (const Fault &) { return; }
    throw Fault("A partial peer failure must not become a trusted whole-run checkpoint");
  };
  rejects([&] { static_cast<void>(mcu.checkpoint()); });
  rejects([&] { static_cast<void>(dsp.checkpoint()); });
  rejects([&] { static_cast<void>(link.checkpoint()); });
}
}

void verify_scheduled_frontier(McuMachine &mcu,
                               DspMachine &dsp,
                               ClockedLink &link,
                               Progress &progress,
                               std::ostringstream &trace,
                               const std::filesystem::path &prefix,
                               const std::uint64_t epoch) {
  trace.str({});
  trace.clear();
  trace << "epoch ticks=" << epoch << '\n';
  record("root", mcu, dsp, link, progress, trace);
  std::optional<Replay> frozen;
  const auto error = until_fault(mcu, dsp, link, progress, trace, &frozen);
  if (!frozen) {
    throw Fault("Joint continuation must retain a healthy checkpoint near the fault");
  }
  reject_faulted_capture(mcu, dsp, link);
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned address = 0; address < coupled::long_ram_end; ++address) {
      require(dsp.bus().read({.space = space, .address = address}) == 0,
              "Original scheduled stores clear every mapped pair before the rejected write");
    }
  }
  const auto original = trace.str();
  const auto expected_mcu = mcu.cpu().state();
  const auto expected_dsp = dsp.cpu().debug_state();
  const auto expected_ticks = mcu.run_clock().elapsed_ticks();
  const auto expected_dsp_ticks = dsp.run_clock().elapsed_ticks();
  std::ofstream file(prefix.string() + ".frontier.trace");
  file.exceptions(std::ios::failbit | std::ios::badbit);
  file << original;
  progress.census.write(prefix.string() + ".frontier.mcu.jsonl");
  progress.phases.write(prefix.string() + ".frontier.phases.json");
  link.write_census(prefix.string() + ".frontier.dsp.jsonl");
  mcu.restore(frozen->mcu);
  dsp.restore(frozen->dsp);
  progress = frozen->progress;
  trace.str({});
  trace.clear();
  link.restore(frozen->link);
  const auto replay_error = until_fault(mcu, dsp, link, progress, trace, nullptr);
  require(replay_error == error && trace.str() == original.substr(frozen->offset) &&
              owned_clock::same_cpu(mcu.cpu().state(), expected_mcu) &&
              dsp.cpu().debug_state() == expected_dsp &&
              mcu.run_clock().elapsed_ticks() == expected_ticks &&
              dsp.run_clock().elapsed_ticks() == expected_dsp_ticks,
          "Coordinated recovery repeats every event, actual fault and partial native state");
  reject_faulted_capture(mcu, dsp, link);
  std::ofstream replay_file(prefix.string() + ".frontier.replay.trace");
  replay_file.exceptions(std::ios::failbit | std::ios::badbit);
  replay_file << trace.str();
}
}
