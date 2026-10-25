// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp_norm_fixture.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_reset_fixture.hpp"
#include "original_update_reset_input_pads.hpp"
#include "original_update_transport_fixture.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace blofeld::test::reset_inputs {
constexpr unsigned application = 0x8200, phy_poll = 0x8049, q_read = 0x80ae;
constexpr unsigned boot_instructions = 90312, boot_cycles = 233742;
constexpr unsigned ram_begin = 0x1200, ram_end = 0x2600, clear_end = 0x807d;
constexpr unsigned returned_pc = 0x8083, pending_vector = 0x10, pending_level = 1;
constexpr unsigned main_payload = 4, main_program = 5;
constexpr unsigned wait_chunk_zero = 0x2e61, wait_chunk_two = 0x2e6b;
constexpr unsigned source_cursor = 0x144ba;

struct Counters {
  std::optional<CpuState> interrupted;
  std::array<unsigned, 9> values;

  explicit Counters(const update_forward::Forward &run)
      : interrupted(run.interrupted),
        values{run.sent,
               run.received,
               run.loaded,
               run.irq_entries,
               run.irq_returns,
               run.irq_queries,
               run.unknown_irq_queries,
               run.last_irq_known,
               run.last_irq_levels} {}

  void restore(update_forward::Forward &run) const {
    run.interrupted = interrupted;
    run.sent = values[0];
    run.received = values[1];
    run.loaded = values[2];
    run.irq_entries = values[3];
    run.irq_returns = values[4];
    run.irq_queries = values[5];
    run.unknown_irq_queries = values[6];
    run.last_irq_known = values[7];
    run.last_irq_levels = values[8];
  }
};

inline auto ram(McuBus &bus) {
  std::array<std::uint8_t, ram_end - ram_begin> result{};
  for (unsigned n = 0; n < result.size(); ++n) {
    result[n] = bus.read(static_cast<std::uint16_t>(ram_begin + n));
  }
  return result;
}

inline void retained(update_forward::Forward &run,
                     const std::vector<update_transport::Block> &blocks) {
  for (const unsigned index : {main_payload, main_program}) {
    const auto &words = blocks.at(index).expected;
    const auto &layout = update_transport::layouts.at(index);
    for (unsigned n = 0; n < words.size(); ++n) {
      require(run.pair.dsp.bus().read({.space = index == main_payload ? DspSpace::y : DspSpace::p,
                                       .address = layout.address + n}) == words[n],
              "Post-reset original native DSP download and real packet padding remain exact");
    }
  }
}

inline void reconnect(update_reset::Continuation &reset,
                      const gpio_pair::PairSnapshot &saved,
                      const Counters &counters,
                      BootstrapPads &pads) {
  auto &pair = reset.run.pair;
  pair.restore_owners(saved);
  counters.restore(reset.run);
  pads.attach();
  pair.schedule->restore(saved.schedule);
  pair.attach_observers();
  reset.attach();
  pair.additional_bus_write = [&pads](const BusEvent &event) {
    if (event.address == hc12::ptj || event.address == hc12::ddrj || event.address == hc12::rdrj) {
      pads.audit_irqa();
    }
  };
  pair.sync();
  pads.audit_irqa();
}

inline void result(update_reset::Continuation &reset,
                   BootstrapPads &pads,
                   const bool high,
                   const unsigned sample,
                   const std::uint64_t cycles) {
  auto &run = reset.run;
  auto &pair = run.pair;
  const auto mcu = pair.mcu.cpu().state();
  const auto dsp = pair.dsp.cpu().state();
  require(mcu.pc == application && mcu.sp == ram_end && mcu.d == 0 &&
              pair.mcu.bus().read(hc12::modrr) == 0 && cycles == boot_cycles &&
              pads.bias_samples() != 0 && pads.irqa_levels() == "101" &&
              pair.dsp.run_clock().native_epoch() == reset.dsp_epoch &&
              dsp.pc == (high ? wait_chunk_zero : wait_chunk_two) &&
              dsp.r[1] == (high ? source_cursor + 1 : source_cursor),
          "Original joint resident bootstrap reaches application with actual PR2 and DSP stream "
          "continuity");
  require(pair.dsp.bus().interrupt_request(0) ==
                  DspInterruptRequest{.vector = pending_vector, .level = pending_level} &&
              !pair.dsp.bus().interrupt_request(3),
          "Native PJ0 transition retains the masked original DSP IRQA edge");
  const auto memory = ram(pair.mcu.bus());
  for (unsigned n = 0; n < memory.size(); ++n) {
    const auto address = ram_begin + n;
    unsigned expected = 0;
    if (address == ram_end - 2) { expected = returned_pc >> 8U; }
    if (address == ram_end - 1) { expected = returned_pc & e_mask; }
    require(memory[n] == expected,
            "Native clear/checksum retains only its actual JSR return bytes");
  }
  reset.storage();
  pair.trace << "RESET_INPUT_RESULT high=" << high << " instructions=" << boot_instructions
             << " cycles=" << cycles << " pc=" << mcu.pc << " sp=" << mcu.sp << " d=" << mcu.d
             << " q=" << sample << " dsp-pc=" << dsp.pc << " cursor=" << dsp.r[1]
             << " irqa=101 pending-vector=" << pending_vector << " pending-level=" << pending_level
             << " passive-samples=" << pads.bias_samples()
             << " raw-unresolved=" << pair.link.unresolved_events
             << " storage=1 ram=1 retained=1\n";
}

inline void boot(update_reset::Continuation &reset, BootstrapPads &pads, const bool high) {
  auto &run = reset.run;
  const auto before_cycles = run.pair.mcu.run_clock().bus_cycles();
  bool phy = false;
  bool cleared = false;
  unsigned sample = 0;
  unsigned reads = 0;
  for (unsigned completed = 0; run.pair.mcu.cpu().state().pc != application; ++completed) {
    require(completed < boot_instructions, "Original joint reset/bootstrap instruction bound");
    const auto pc = run.pair.mcu.cpu().state().pc;
    if (pc == phy_poll && !phy) {
      run.pair.mcu.bus().observe_phy({.edges = 15});
      phy = true;
    }
    if (pc == clear_end) {
      for (const auto value : ram(run.pair.mcu.bus())) {
        require(value == 0, "Native boot cleared every RAM byte before stack/checksum setup");
      }
      cleared = true;
    }
    run.native_step();
    if (pc == q_read) {
      sample = run.pair.mcu.cpu().state().d & e_mask;
      ++reads;
    }
    if (run.pair.mcu.cpu().state().pc == application) {
      require(completed + 1 == boot_instructions && phy && cleared && reads == 1,
              "Native bootstrap executes its entire RAM/checksum and actual Q-read path");
    }
  }
  pads.audit_irqa();
  result(reset, pads, high, sample, run.pair.mcu.run_clock().bus_cycles() - before_cycles);
}

inline void exercise(update_reset::Continuation &reset,
                     const std::vector<update_transport::Block> &blocks,
                     const std::string &path) {
  auto &pair = reset.run.pair;
  const auto saved = pair.checkpoint();
  const Counters counters(reset.run);
  reset.boot(); // Preserve the strict, unqualified baseline before supplying laboratory inputs.
  pair.trace << "RESET_INPUTS_BEGIN\n";
  for (const bool high : {false, true}) {
    DspDebugState expected_dsp;
    CpuState expected_mcu;
    std::array<std::uint8_t, ram_end - ram_begin> expected_ram{};
    std::array<std::uint64_t, 5> expected_clocks{};
    std::array<unsigned, 9> expected_counters{};
    for (const unsigned replay : {0U, 1U}) {
      std::ofstream trace(path + ".q" + (high ? "1" : "0") + "." + std::to_string(replay));
      trace.exceptions(std::ios::failbit | std::ios::badbit);
      const update_reset::Redirect redirect(pair.trace, trace);
      BootstrapPads pads(reset.run, high);
      reconnect(reset, saved, counters, pads);
      boot(reset, pads, high);
      retained(reset.run, blocks);
      const auto memory = ram(pair.mcu.bus());
      const std::array clocks{pair.mcu.run_clock().elapsed_ticks(),
                              pair.mcu.run_clock().bus_cycles(),
                              pair.mcu.run_clock().oscillator_cycles(),
                              pair.dsp.run_clock().elapsed_ticks(),
                              pair.schedule->completed_steps()};
      const Counters after(reset.run);
      if (replay == 0) {
        expected_dsp = pair.dsp.cpu().debug_state();
        expected_mcu = pair.mcu.cpu().state();
        expected_ram = memory;
        expected_clocks = clocks;
        expected_counters = after.values;
      } else {
        require(pair.dsp.cpu().debug_state() == expected_dsp &&
                    update_forward::registers(pair.mcu.cpu().state()) ==
                        update_forward::registers(expected_mcu) &&
                    memory == expected_ram && clocks == expected_clocks &&
                    after.values == expected_counters,
                "Coordinated post-reset snapshot replays actual native state, RAM and both clocks");
      }
    }
    pair.additional_bus_write = {};
  }
  pair.trace << "RESET_INPUTS_END\n";
}
}
