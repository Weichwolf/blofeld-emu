// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_run_clock.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_machine.hpp"
#include "blofeld/mcu_reset.hpp"
#include "blofeld/mcu_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_update_forward_fixture.hpp"

#include <array>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <string_view>

namespace blofeld::test::update_reset {
namespace forward = update_forward;
constexpr unsigned reset_edges = 195, reset_ticks = 1024, oscillator_delta = 205;
constexpr unsigned resident_entry = 0x8000, vector_cycles = 5, instruction_limit = 64;
constexpr unsigned strobe = 0x80, all_bits = 0xff, waiting_chunk = 0x2e6b, cursor = 0x144ba;
constexpr unsigned checkpoint_edges = 131, remaining_edges = 64;

class Redirect {
public:
  Redirect(std::ostream &stream, std::ostream &capture)
      : stream_(stream), previous_(stream.rdbuf(capture.rdbuf())) {}

  ~Redirect() { stream_.rdbuf(previous_); }

  Redirect(const Redirect &) = delete;
  Redirect &operator=(const Redirect &) = delete;

private:
  std::ostream &stream_;
  std::streambuf *previous_;
};

struct Continuation {
  explicit Continuation(forward::Forward &owner, const McuFlashImage &image)
      : run(owner),
        nvm(image),
        ticks(run.pair.mcu.run_clock().elapsed_ticks()),
        osc(run.pair.mcu.run_clock().oscillator_cycles()),
        bus(run.pair.mcu.run_clock().bus_cycles()),
        dsp_ticks(run.pair.dsp.run_clock().elapsed_ticks()),
        instructions(run.pair.dsp.cpu().debug_state().native_instructions),
        steps(run.pair.schedule->completed_steps()),
        dsp_epoch(run.pair.dsp.run_clock().native_epoch()) {}

  void start() {
    run.display.reset(); // Finished screenshot fixture cannot retain a replaced MCU bus.
    run.pair.additional_bus_write = {};
    run.pair.schedule->begin_system_reset(McuResetSource::cop, 3, McuDspResetOrder::dsp_before_mcu);
    require(run.pair.mcu.bus().read(hc12::pear) == 0x10,
            "Normal-single-chip reset selects GPIO, not ECLK");
    run.pair.trace << "RESET_BEGIN mcu-epoch=2 dsp-epoch=" << dsp_epoch << '\n';
    run.pair.connect_mcu_pads();
    run.attach_interrupt_pads();
    run.pair.attach_observers();
    run.pair.sync();
    attach();
  }

  void attach() {
    run.pair.mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      if (run.pair.mcu.system_reset_active()) {
        slots += cycle.kind;
        if (cycle.kind == 'V' && cycle.address) { vector = *cycle.address; }
      }
      run.pair.on_cycle(cycle);
    });
    run.pair.mcu.set_reset_period_observer([this](const McuResetPeriod period) {
      run.pair.schedule->begin_reset_period(
          period, [this](const auto &step) { run.pair.before_step(step); }, run.pair.completed());
      ++periods;
    });
  }

  [[nodiscard]] std::array<unsigned, 4> irq_state() const {
    return {run.irq_queries, run.unknown_irq_queries, run.last_irq_known, run.last_irq_levels};
  }

  void finish_pulse() {
    require(run.pair.mcu.advance_system_reset(remaining_edges, McuResetPad::high) ==
                remaining_edges,
            "The remaining64 reset edges and native vector execute after the saved drive interval");
    run.pair.schedule->end_system_reset();
  }

  void replay_pulse() {
    require(run.pair.mcu.advance_system_reset(checkpoint_edges, McuResetPad::unknown) ==
                checkpoint_edges,
            "The native joint reset reaches the exact131-edge drive/settle boundary");
    const auto saved = run.pair.checkpoint();
    const auto irq = irq_state();
    std::ostringstream first;
    {
      const Redirect redirect(run.pair.trace, first);
      finish_pulse();
    }
    const auto cpu = forward::registers(run.pair.mcu.cpu().state());
    const auto dsp = run.pair.dsp.cpu().debug_state();
    const auto mcu_time = run.pair.mcu.run_clock().elapsed_ticks();
    const auto dsp_time = run.pair.dsp.run_clock().elapsed_ticks();
    const auto irq_after = irq_state();
    run.pair.restore_owners(saved);
    run.pair.reconnect(saved);
    run.attach_interrupt_pads();
    run.irq_queries = irq[0];
    run.unknown_irq_queries = irq[1];
    run.last_irq_known = irq[2];
    run.last_irq_levels = irq[3];
    run.pair.sync();
    periods = checkpoint_edges;
    slots.clear();
    vector = 0;
    attach();
    std::ostringstream second;
    {
      const Redirect redirect(run.pair.trace, second);
      finish_pulse();
    }
    require(first.str() == second.str() && forward::registers(run.pair.mcu.cpu().state()) == cpu &&
                run.pair.dsp.cpu().debug_state() == dsp && irq_state() == irq_after &&
                run.pair.mcu.run_clock().elapsed_ticks() == mcu_time &&
                run.pair.dsp.run_clock().elapsed_ticks() == dsp_time,
            "Actual original COP reset replays every remaining native pad/clock/vector event");
    run.pair.trace << second.str();
    run.pair.trace << "RESET_REPLAY cut=131 remaining=64 native=1 pins=1 trace=1\n";
  }

  void storage() const {
    const auto after = run.pair.mcu.bus().power_off_flash_image();
    require(after.bytes == nvm.bytes && after.known == nvm.known && after.fresh == nvm.fresh,
            "Joint reset/startup retain every MCU NVM byte, known bit and erase-history bit");
  }

  void pulse() {
    auto &mcu = run.pair.mcu;
    replay_pulse();
    const auto dsp_steps = run.pair.dsp.cpu().debug_state().native_instructions - instructions;
    require(!mcu.system_reset_active() && !mcu.timing_faulted() && !run.pair.schedule->faulted() &&
                slots == "VfPPP" && vector == hc12::cop_reset_vector && periods == reset_edges &&
                mcu.cpu().state().pc == resident_entry && mcu.cpu().state().sp == hc12::reset_sp &&
                mcu.cpu().state().ccr == hc12::reset_ccr &&
                mcu.run_clock().elapsed_ticks() - ticks == reset_ticks &&
                mcu.run_clock().oscillator_cycles() - osc == oscillator_delta &&
                mcu.run_clock().bus_cycles() - bus == vector_cycles &&
                mcu.run_clock().native_epoch() == 2 && dsp_steps != 0 &&
                dsp_steps == run.pair.schedule->completed_steps() - steps &&
                run.pair.dsp.run_clock().native_epoch() == dsp_epoch &&
                run.pair.dsp.cpu().state().pc == waiting_chunk &&
                run.pair.dsp.cpu().state().r[1] == cursor,
            "The native COP vector restarts MCU while the same DSP executes its original handshake "
            "wait");
    storage();
    run.pair.trace
        << "SYSTEM_RESET edges=195 vector=" << vector
        << " target=32768 ticks=1024 osc=205 bus=5 epoch=2 dsp-held=0 storage=1 joint=1 periods="
        << periods << " dsp-epoch=" << dsp_epoch << " dsp-steps=" << dsp_steps
        << " dsp-before=" << dsp_ticks << " dsp-after=" << run.pair.dsp.run_clock().elapsed_ticks()
        << " common-start="
        << ticks * (run.pair.dsp.run_clock().timebase_hz() / mcu.run_clock().timebase_hz())
        << " common-end=" << run.pair.schedule->end_ticks() << '\n';
  }

  void quarantine() {
    run.pair.mcu.set_reset_period_observer({});
    run.pair.mcu.set_cycle_observer({});
    const auto cpu = forward::registers(run.pair.mcu.cpu().state());
    const auto dsp = run.pair.dsp.cpu().debug_state();
    const auto mcu_time = run.pair.mcu.run_clock().elapsed_ticks();
    const auto dsp_time = run.pair.dsp.run_clock().elapsed_ticks();
    bool mcu_rejected = false;
    bool joint_rejected = false;
    try {
      run.pair.mcu.cpu().step();
    } catch (const Fault &) { mcu_rejected = true; }
    try {
      run.pair.schedule->end_instruction();
    } catch (const Fault &) { joint_rejected = true; }
    require(
        mcu_rejected && joint_rejected && forward::registers(run.pair.mcu.cpu().state()) == cpu &&
            run.pair.dsp.cpu().debug_state() == dsp &&
            run.pair.mcu.run_clock().elapsed_ticks() == mcu_time &&
            run.pair.dsp.run_clock().elapsed_ticks() == dsp_time,
        "A partially committed store cannot resume either native actor or their joint timeline");
  }

  void boot() {
    for (unsigned completed = 0; completed < instruction_limit; ++completed) {
      const auto before = run.pair.mcu.cpu().state().pc;
      try {
        run.native_step();
      } catch (const Fault &error) {
        const std::string_view message(error.what());
        require(message.contains("Unknown DSP GPIO data read at X:0xffffbd mask=0x000080") &&
                    message.contains("conflicts=0x000080") && message.contains("0x002e6b"),
                "Only the actual post-reset shared-pad read may stop native startup: " +
                    std::string(error.what()));
        const auto net = run.pair.net();
        require(
            net.known == all_bits - strobe && net.conflicts == strobe &&
                run.pair.mcu.bus().read(hc12::perq) == all_bits &&
                run.pair.mcu.bus().read(hc12::ppsq) == 0 && run.pair.schedule->faulted() &&
                run.pair.dsp.timing_faulted(),
            "Opposed documented passive pulls remain unqualified when the native DSP consumes Q7");
        storage();
        quarantine();
        run.pair.trace << "RESET_BOOT_FAULT " << error.what() << '\n';
        run.pair.trace << "RESET_BOOT start=32768 attempt=" << before
                       << " stop=" << run.pair.mcu.cpu().state().pc << " completed=" << completed
                       << " pad-known=" << net.known << " pad-conflicts=" << net.conflicts
                       << " perq=255 ppsq=0 dsp-faulted=1 mcu-faulted="
                       << run.pair.mcu.timing_faulted()
                       << " joint=1 mcu-ticks=" << run.pair.mcu.run_clock().elapsed_ticks()
                       << " dsp-ticks=" << run.pair.dsp.run_clock().elapsed_ticks()
                       << " refused=1\n";
        return;
      }
    }
    throw Fault("Joint post-reset resident-loader bound");
  }

  forward::Forward &run;
  const McuFlashImage &nvm;
  std::uint64_t ticks, osc, bus, dsp_ticks, instructions, steps, dsp_epoch;
  unsigned vector = 0, periods = 0;
  std::string slots;
};

inline void execute(forward::Forward &run, const McuFlashImage &nvm) {
  Continuation reset(run, nvm);
  reset.start();
  reset.pulse();
  reset.boot();
}
}
